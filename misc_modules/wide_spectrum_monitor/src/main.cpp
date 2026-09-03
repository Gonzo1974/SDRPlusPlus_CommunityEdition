#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <exception>
#include <limits>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <config.h>
#include <core.h>
#include <dsp/sink/handler_sink.h>
#include <dsp/stream.h>
#include <dsp/window/nuttall.h>
#include <fftw3.h>
#include <gui/gui.h>
#include <gui/style.h>
#include <gui/tuner.h>
#include <imgui.h>
#include <module.h>
#include <signal_path/signal_path.h>
#include <utils/flog.h>
#include <volk/volk.h>

SDRPP_MOD_INFO{
    /* Name:            */ "wide_spectrum_monitor",
    /* Description:     */ "Sweep and combine multiple SDR passbands into one wide spectrum",
    /* Author:          */ "SDR++ Community Edition",
    /* Version:         */ 0, 1, 0,
    /* Max instances    */ 1
};

ConfigManager config;

namespace {
    constexpr int WIDE_BIN_COUNT = 2048;
    constexpr int DIRECT_FFT_SIZE = 65536;
    constexpr double DIRECT_FFT_RATE = 20.0;
    constexpr int MIN_IQ_CAPTURE_TIMEOUT_MS = 2000;
    constexpr std::size_t MAX_SEGMENT_COUNT = 4096;
    constexpr double USABLE_BANDWIDTH_RATIO = 0.85;
    constexpr float GRAPH_MIN_DB = -120.0f;
    constexpr float GRAPH_MAX_DB = 0.0f;
    constexpr float MIN_VALID_FFT_DB = -200.0f;
    constexpr float MIN_VALID_SWEEP_PERCENT = 95.0f;
    const float NO_DATA_DBFS = std::numeric_limits<float>::quiet_NaN();

    // Producer FFT diagnostics (V8) is intentionally retired for this module.
    // V9 owns its IQ capture and FFT buffers instead of observing the GUI waterfall.

    struct SweepSettings {
        double startHz = 225000000.0;
        double stopHz = 379000000.0;
        double sampleRateHz = 2400000.0;
        float overlapPercent = 10.0f;
        int tuningTimeMs = 120;
        int fftAveraging = 3;
        float thresholdDbfs = -55.0f;
        bool peakHold = true;
    };

    struct DetectedPeak {
        double frequencyHz = 0.0;
        float levelDbfs = GRAPH_MIN_DB;
    };

    struct SpectrumDebugStats {
        std::uint64_t validSamples = 0;
        std::uint64_t invalidSamples = 0;
        std::uint64_t nanSamples = 0;
        std::uint64_t infSamples = 0;
        std::uint64_t aboveZeroSamples = 0;
        std::uint64_t belowMinus200Samples = 0;
        std::uint64_t zeroDefaultSamples = 0;
        float rawMin = NO_DATA_DBFS;
        float rawMax = NO_DATA_DBFS;
        float convertedMin = NO_DATA_DBFS;
        float convertedMax = NO_DATA_DBFS;
        std::size_t validWideBins = 0;
        std::size_t totalWideBins = 0;
        float validWidePercent = 0.0f;
        float wideMin = NO_DATA_DBFS;
        float wideMax = NO_DATA_DBFS;
        float peakMin = NO_DATA_DBFS;
        float peakMax = NO_DATA_DBFS;
    };

    struct RawFFTDistribution {
        static constexpr std::size_t SAMPLE_COUNT = 9;

        std::size_t totalBins = 0;
        std::size_t finiteBins = 0;
        std::size_t negativeFiniteBins = 0;
        std::size_t positiveFiniteBins = 0;
        std::size_t exactZeroBins = 0;
        std::size_t negativeZeroBins = 0;
        std::size_t subnormalBins = 0;
        std::size_t nanBins = 0;
        std::size_t positiveInfBins = 0;
        std::size_t negativeInfBins = 0;
        std::size_t belowMinus200Bins = 0;
        std::size_t minus200ToMinus160Bins = 0;
        std::size_t minus160ToMinus120Bins = 0;
        std::size_t minus120ToMinus100Bins = 0;
        std::size_t minus100ToMinus80Bins = 0;
        std::size_t minus80ToMinus60Bins = 0;
        std::size_t minus60ToMinus40Bins = 0;
        std::size_t minus40ToMinus20Bins = 0;
        std::size_t minus20ToZeroBins = 0;
        std::size_t zeroToOneBins = 0;
        std::size_t oneTo100Bins = 0;
        std::size_t above100Bins = 0;
        std::size_t firstNonZeroBin = std::numeric_limits<std::size_t>::max();
        std::size_t lastNonZeroBin = 0;
        std::size_t currentZeroRun = 0;
        std::size_t longestZeroRun = 0;
        std::size_t quartileValidBins[4] = {};
        std::size_t quartileZeroBins[4] = {};
        std::size_t quartileNonFiniteBins[4] = {};
        std::size_t quartileOutOfRangeBins[4] = {};
        std::size_t sampleIndices[SAMPLE_COUNT] = {};
        float sampleValues[SAMPLE_COUNT] = {};
        float finiteMin = NO_DATA_DBFS;
        float finiteMax = NO_DATA_DBFS;
    };

    struct SegmentDebugStats {
        int segmentNumber = 0;
        std::size_t rawFftBinCount = 0;
        std::size_t validRawBins = 0;
        std::size_t rejectedZeroBins = 0;
        std::size_t rejectedNonFiniteBins = 0;
        std::size_t rejectedOutOfRangeBins = 0;
        std::size_t wideBinsWritten = 0;
        std::size_t accumulatedUniqueWideBins = 0;
        std::size_t wideBinsTotal = WIDE_BIN_COUNT;
        float coveragePercent = 0.0f;
        double rawStartHz = 0.0;
        double rawStopHz = 0.0;
        double usableStartHz = 0.0;
        double usableStopHz = 0.0;
        std::size_t iqSamplesPerFrame = 0;
        std::uint64_t firstIQFrameGeneration = 0;
        std::uint64_t lastIQFrameGeneration = 0;
        RawFFTDistribution rawDistribution;
    };

    enum class IQCaptureEndReason {
        NONE,
        IN_PROGRESS,
        TIMEOUT,
        CANCELLED,
        COMPLETED,
        INVALID_REQUEST,
        RESULT_MISMATCH
    };

    struct IQCaptureDiagnostics {
        std::uint64_t handlerCalls = 0;
        std::uint64_t handlerSamples = 0;
        std::uint64_t requestedHandlerCalls = 0;
        std::uint64_t requestedHandlerSamples = 0;
        std::uint64_t discardedChunks = 0;
        std::size_t copiedSamples = 0;
        std::size_t targetSamples = 0;
        int lastHandlerChunkSize = 0;
        IQCaptureEndReason endReason = IQCaptureEndReason::NONE;
        std::uint64_t generation = 0;
    };

    enum class PendingTuneReason {
        NONE,
        RESTORE_AFTER_STOP,
        SELECTED_PEAK
    };

    const char* iqCaptureEndReasonText(IQCaptureEndReason reason) {
        switch (reason) {
        case IQCaptureEndReason::IN_PROGRESS:
            return "in progress";
        case IQCaptureEndReason::TIMEOUT:
            return "timeout";
        case IQCaptureEndReason::CANCELLED:
            return "sweep cancellation/shutdown";
        case IQCaptureEndReason::COMPLETED:
            return "successful frame completion";
        case IQCaptureEndReason::INVALID_REQUEST:
            return "invalid capture request";
        case IQCaptureEndReason::RESULT_MISMATCH:
            return "completed generation with sample-count mismatch";
        case IQCaptureEndReason::NONE:
        default:
            return "no capture attempt";
        }
    }

    void includeInRange(float value, float& minimum, float& maximum) {
        if (!std::isfinite(minimum) || value < minimum) {
            minimum = value;
        }
        if (!std::isfinite(maximum) || value > maximum) {
            maximum = value;
        }
    }

    bool validateFFTDbfs(float value, SpectrumDebugStats& stats) {
        if (std::isnan(value)) {
            ++stats.nanSamples;
            ++stats.invalidSamples;
            return false;
        }
        if (std::isinf(value)) {
            ++stats.infSamples;
            ++stats.invalidSamples;
            return false;
        }

        includeInRange(value, stats.rawMin, stats.rawMax);
        if (value > GRAPH_MAX_DB) {
            ++stats.aboveZeroSamples;
            ++stats.invalidSamples;
            return false;
        }
        if (value < MIN_VALID_FFT_DB) {
            ++stats.belowMinus200Samples;
            ++stats.invalidSamples;
            return false;
        }
        // Preserve the V3 validation rule: an exact 0.0 dBFS value is treated as
        // invalid so it can never become a persistent full-scale wide-band spike.
        if (value == 0.0f) {
            ++stats.zeroDefaultSamples;
            ++stats.invalidSamples;
            return false;
        }

        ++stats.validSamples;
        // The private VOLK power-spectrum output is already logarithmic power.
        // "After conversion" is therefore intentionally the same value.
        includeInRange(value, stats.convertedMin, stats.convertedMax);
        return true;
    }

    bool isValidDbfs(float value) {
        return std::isfinite(value) && value >= MIN_VALID_FFT_DB && value < GRAPH_MAX_DB;
    }

    void recordRawFFTValue(float value, std::size_t index, std::size_t totalBins,
                           RawFFTDistribution& distribution) {
        if (totalBins == 0) {
            return;
        }

        const std::size_t quartile = std::min<std::size_t>(3, (index * 4) / totalBins);
        if (!std::isfinite(value)) {
            ++distribution.quartileNonFiniteBins[quartile];
            if (std::isnan(value)) {
                ++distribution.nanBins;
            }
            else if (value > 0.0f) {
                ++distribution.positiveInfBins;
            }
            else {
                ++distribution.negativeInfBins;
            }
        }
        else {
            ++distribution.finiteBins;
            includeInRange(value, distribution.finiteMin, distribution.finiteMax);
            if (std::fpclassify(value) == FP_SUBNORMAL) {
                ++distribution.subnormalBins;
            }

            if (value == 0.0f) {
                ++distribution.exactZeroBins;
                ++distribution.quartileZeroBins[quartile];
                if (std::signbit(value)) {
                    ++distribution.negativeZeroBins;
                }
            }
            else if (value < MIN_VALID_FFT_DB) {
                ++distribution.negativeFiniteBins;
                ++distribution.belowMinus200Bins;
                ++distribution.quartileOutOfRangeBins[quartile];
            }
            else if (value < -160.0f) {
                ++distribution.negativeFiniteBins;
                ++distribution.minus200ToMinus160Bins;
                ++distribution.quartileValidBins[quartile];
            }
            else if (value < -120.0f) {
                ++distribution.negativeFiniteBins;
                ++distribution.minus160ToMinus120Bins;
                ++distribution.quartileValidBins[quartile];
            }
            else if (value < -100.0f) {
                ++distribution.negativeFiniteBins;
                ++distribution.minus120ToMinus100Bins;
                ++distribution.quartileValidBins[quartile];
            }
            else if (value < -80.0f) {
                ++distribution.negativeFiniteBins;
                ++distribution.minus100ToMinus80Bins;
                ++distribution.quartileValidBins[quartile];
            }
            else if (value < -60.0f) {
                ++distribution.negativeFiniteBins;
                ++distribution.minus80ToMinus60Bins;
                ++distribution.quartileValidBins[quartile];
            }
            else if (value < -40.0f) {
                ++distribution.negativeFiniteBins;
                ++distribution.minus60ToMinus40Bins;
                ++distribution.quartileValidBins[quartile];
            }
            else if (value < -20.0f) {
                ++distribution.negativeFiniteBins;
                ++distribution.minus40ToMinus20Bins;
                ++distribution.quartileValidBins[quartile];
            }
            else if (value < 0.0f) {
                ++distribution.negativeFiniteBins;
                ++distribution.minus20ToZeroBins;
                ++distribution.quartileValidBins[quartile];
            }
            else {
                ++distribution.positiveFiniteBins;
                ++distribution.quartileOutOfRangeBins[quartile];
                if (value <= 1.0f) {
                    ++distribution.zeroToOneBins;
                }
                else if (value <= 100.0f) {
                    ++distribution.oneTo100Bins;
                }
                else {
                    ++distribution.above100Bins;
                }
            }
        }

        if (value == 0.0f) {
            ++distribution.currentZeroRun;
            distribution.longestZeroRun = std::max(distribution.longestZeroRun,
                                                   distribution.currentZeroRun);
        }
        else {
            distribution.currentZeroRun = 0;
            if (distribution.firstNonZeroBin == std::numeric_limits<std::size_t>::max()) {
                distribution.firstNonZeroBin = index;
            }
            distribution.lastNonZeroBin = index;
        }
    }
}

class WideSpectrumMonitorModule : public ModuleManager::Instance {
public:
    explicit WideSpectrumMonitorModule(std::string instanceName) : name(std::move(instanceName)) {
        loadConfig();
        iqSink.init(nullptr, iqStreamHandler, this);

        if (initializeDirectFFT()) {
            iqCaptureBuffer.reserve(DIRECT_FFT_SIZE);
        }
        else {
            statusText = "FFT initialization failed";
            lastError = "Could not initialize the private Wide Spectrum Monitor FFT";
            flog::error("Wide Spectrum Monitor: {}", lastError);
        }

        playStateHandler.ctx = this;
        playStateHandler.handler = playStateChanged;
        gui::mainWindow.onPlayStateChange.bindHandler(&playStateHandler);
        gui::menu.registerEntry(name, menuHandler, this, nullptr);

        try {
            workerThread = std::thread(&WideSpectrumMonitorModule::workerLoop, this);
        }
        catch (const std::exception& e) {
            std::lock_guard<std::mutex> lock(displayMutex);
            lastError = std::string("Could not start sweep worker: ") + e.what();
            statusText = "Worker unavailable";
            flog::error("Wide Spectrum Monitor: {}", lastError);
        }
    }

    ~WideSpectrumMonitorModule() override {
        gui::mainWindow.onPlayStateChange.unbindHandler(&playStateHandler);
        sweepRequested.store(false);
        shuttingDown.store(true);
        requestCv.notify_all();
        iqCaptureCv.notify_all();
        if (workerThread.joinable()) {
            workerThread.join();
        }
        stopIQCapturePath();
        releaseDirectFFT();
        gui::menu.removeEntry(name);
    }

    void postInit() override {}

    void enable() override {
        enabled = true;
    }

    void disable() override {
        enabled = false;
        requestStop("Stopped (module disabled)");
    }

    bool isEnabled() override {
        return enabled;
    }

private:
    static void menuHandler(void* ctx) {
        static_cast<WideSpectrumMonitorModule*>(ctx)->drawMenu();
    }

    static void playStateChanged(bool playing, void* ctx) {
        if (!playing) {
            static_cast<WideSpectrumMonitorModule*>(ctx)->requestStop("Stopped (SDR source is not running)");
        }
    }

    static void iqStreamHandler(dsp::complex_t* data, int count, void* ctx) {
        if (!data || count <= 0) {
            return;
        }

        auto* instance = static_cast<WideSpectrumMonitorModule*>(ctx);
        instance->iqHandlerCalls.fetch_add(1, std::memory_order_relaxed);
        instance->iqHandlerSamples.fetch_add(static_cast<std::uint64_t>(count), std::memory_order_relaxed);
        instance->iqLastHandlerChunkSize.store(count, std::memory_order_relaxed);

        std::unique_lock<std::mutex> lock(instance->iqCaptureMutex);
        if (!instance->iqCaptureRequested) {
            return;
        }
        instance->iqRequestedHandlerCalls.fetch_add(1, std::memory_order_relaxed);
        instance->iqRequestedHandlerSamples.fetch_add(static_cast<std::uint64_t>(count), std::memory_order_relaxed);
        if (instance->shuttingDown.load()) {
            return;
        }

        // A stream buffer can already be waiting when the worker requests a
        // capture. Drop that complete chunk so every retained sample is newer
        // than the post-tune capture request.
        if (instance->discardNextIQChunk) {
            instance->discardNextIQChunk = false;
            instance->iqDiscardedChunks.fetch_add(1, std::memory_order_relaxed);
            return;
        }

        const std::size_t available = static_cast<std::size_t>(count);
        const std::size_t needed = instance->iqCaptureTarget - instance->iqCaptureBuffer.size();
        const std::size_t copyCount = std::min(available, needed);
        instance->iqCaptureBuffer.insert(instance->iqCaptureBuffer.end(), data, data + copyCount);
        instance->iqLastCaptureCopiedSamples = instance->iqCaptureBuffer.size();
        if (instance->iqCaptureBuffer.size() != instance->iqCaptureTarget) {
            return;
        }

        instance->iqCaptureRequested = false;
        ++instance->iqFrameGeneration;
        instance->iqLastCaptureEndReason = IQCaptureEndReason::COMPLETED;
        instance->iqLastCaptureGeneration = instance->iqFrameGeneration;
        lock.unlock();
        instance->iqCaptureCv.notify_all();
    }

    IQCaptureDiagnostics snapshotIQCaptureDiagnostics() {
        IQCaptureDiagnostics diagnostics;
        diagnostics.handlerCalls = iqHandlerCalls.load(std::memory_order_relaxed);
        diagnostics.handlerSamples = iqHandlerSamples.load(std::memory_order_relaxed);
        diagnostics.requestedHandlerCalls = iqRequestedHandlerCalls.load(std::memory_order_relaxed);
        diagnostics.requestedHandlerSamples = iqRequestedHandlerSamples.load(std::memory_order_relaxed);
        diagnostics.discardedChunks = iqDiscardedChunks.load(std::memory_order_relaxed);
        diagnostics.lastHandlerChunkSize = iqLastHandlerChunkSize.load(std::memory_order_relaxed);
        {
            std::lock_guard<std::mutex> lock(iqCaptureMutex);
            diagnostics.copiedSamples = iqLastCaptureCopiedSamples;
            diagnostics.targetSamples = iqLastCaptureTargetSamples;
            diagnostics.endReason = iqLastCaptureEndReason;
            diagnostics.generation = iqLastCaptureGeneration;
        }
        return diagnostics;
    }

    void logIQCaptureFailure(const IQCaptureDiagnostics& diagnostics) {
        flog::error(
            "Wide Spectrum Monitor: IQ capture failed: reason={}, handler_calls={}, handler_samples={}, "
            "request_calls={}, request_samples={}, discarded_chunks={}, copied={}/{}, last_chunk={}, generation={}",
            iqCaptureEndReasonText(diagnostics.endReason), diagnostics.handlerCalls,
            diagnostics.handlerSamples, diagnostics.requestedHandlerCalls,
            diagnostics.requestedHandlerSamples, diagnostics.discardedChunks,
            diagnostics.copiedSamples, diagnostics.targetSamples,
            diagnostics.lastHandlerChunkSize, diagnostics.generation);
    }

    bool startIQCapturePath() {
        stopIQCapturePath();

        try {
            iqStream = new dsp::stream<dsp::complex_t>();
            iqSink.setInput(iqStream);
            iqSink.start();
            sigpath::iqFrontEnd.bindIQStream(iqStream);
            iqStreamBound = true;
        }
        catch (const std::exception& e) {
            iqSink.stop();
            delete iqStream;
            iqStream = nullptr;
            flog::error("Wide Spectrum Monitor: Could not start IQ capture path: {}", e.what());
            return false;
        }
        catch (...) {
            iqSink.stop();
            delete iqStream;
            iqStream = nullptr;
            flog::error("Wide Spectrum Monitor: Could not start IQ capture path");
            return false;
        }
        return true;
    }

    void stopIQCapturePath() {
        if (iqStreamBound) {
            sigpath::iqFrontEnd.unbindIQStream(iqStream);
            iqStreamBound = false;
        }
        iqSink.stop();
        delete iqStream;
        iqStream = nullptr;
    }

    bool initializeDirectFFT() {
        directFFTIn = static_cast<fftwf_complex*>(
            fftwf_malloc(sizeof(fftwf_complex) * DIRECT_FFT_SIZE));
        directFFTOut = static_cast<fftwf_complex*>(
            fftwf_malloc(sizeof(fftwf_complex) * DIRECT_FFT_SIZE));
        if (!directFFTIn || !directFFTOut) {
            releaseDirectFFT();
            return false;
        }

        directFFTPlan = fftwf_plan_dft_1d(DIRECT_FFT_SIZE, directFFTIn, directFFTOut,
                                          FFTW_FORWARD, FFTW_ESTIMATE);
        if (!directFFTPlan) {
            releaseDirectFFT();
            return false;
        }
        return true;
    }

    void releaseDirectFFT() {
        if (directFFTPlan) {
            fftwf_destroy_plan(directFFTPlan);
            directFFTPlan = nullptr;
        }
        if (directFFTIn) {
            fftwf_free(directFFTIn);
            directFFTIn = nullptr;
        }
        if (directFFTOut) {
            fftwf_free(directFFTOut);
            directFFTOut = nullptr;
        }
    }

    void loadConfig() {
        config.acquire();
        startFrequencyMHz = config.conf["startFrequencyMHz"];
        stopFrequencyMHz = config.conf["stopFrequencyMHz"];
        overlapPercent = config.conf["overlapPercent"];
        tuningTimeMs = config.conf["tuningTimeMs"];
        fftAveraging = config.conf["fftAveraging"];
        thresholdDbfs = config.conf["thresholdDbfs"];
        peakHoldEnabled = config.conf["peakHold"];
        config.release();

        sanitizeControls();
    }

    void saveConfig() {
        config.acquire();
        config.conf["startFrequencyMHz"] = startFrequencyMHz;
        config.conf["stopFrequencyMHz"] = stopFrequencyMHz;
        config.conf["overlapPercent"] = overlapPercent;
        config.conf["tuningTimeMs"] = tuningTimeMs;
        config.conf["fftAveraging"] = fftAveraging;
        config.conf["thresholdDbfs"] = thresholdDbfs;
        config.conf["peakHold"] = peakHoldEnabled;
        config.release(true);
    }

    void sanitizeControls() {
        startFrequencyMHz = std::max(0.0, startFrequencyMHz);
        stopFrequencyMHz = std::max(startFrequencyMHz + 0.001, stopFrequencyMHz);
        overlapPercent = std::clamp(overlapPercent, 0.0f, 50.0f);
        tuningTimeMs = std::clamp(tuningTimeMs, 10, 2000);
        fftAveraging = std::clamp(fftAveraging, 1, 10);
        thresholdDbfs = std::clamp(thresholdDbfs, -150.0f, 0.0f);
    }

    void drawMenu() {
        applyPendingTune();

        const bool sweepBusy = sweepRequested.load() || workerActive.load();
        bool configChanged = false;

        if (sweepBusy) {
            ImGui::BeginDisabled();
        }

        ImGui::TextUnformatted("Start Frequency (MHz)");
        ImGui::SetNextItemWidth(-1.0f);
        configChanged |= ImGui::InputDouble(("##wsm_start_" + name).c_str(), &startFrequencyMHz, 0.1, 1.0, "%.6f");

        ImGui::TextUnformatted("Stop Frequency (MHz)");
        ImGui::SetNextItemWidth(-1.0f);
        configChanged |= ImGui::InputDouble(("##wsm_stop_" + name).c_str(), &stopFrequencyMHz, 0.1, 1.0, "%.6f");

        ImGui::TextUnformatted("Sweep Overlap");
        ImGui::SetNextItemWidth(-1.0f);
        configChanged |= ImGui::SliderFloat(("##wsm_overlap_" + name).c_str(), &overlapPercent, 0.0f, 50.0f, "%.1f %%");

        ImGui::TextUnformatted("Settling / Tuning Time");
        ImGui::SetNextItemWidth(-1.0f);
        configChanged |= ImGui::SliderInt(("##wsm_settle_" + name).c_str(), &tuningTimeMs, 10, 2000, "%d ms");

        ImGui::TextUnformatted("FFT Averaging");
        ImGui::SetNextItemWidth(-1.0f);
        configChanged |= ImGui::SliderInt(("##wsm_average_" + name).c_str(), &fftAveraging, 1, 10, "%d frames");

        ImGui::TextUnformatted("Detection Threshold");
        ImGui::SetNextItemWidth(-1.0f);
        configChanged |= ImGui::SliderFloat(("##wsm_threshold_" + name).c_str(), &thresholdDbfs, -150.0f, 0.0f, "%.1f dBFS");

        configChanged |= ImGui::Checkbox(("Peak Hold##wsm_peak_hold_" + name).c_str(), &peakHoldEnabled);

        if (sweepBusy) {
            ImGui::EndDisabled();
        }

        if (configChanged) {
            sanitizeControls();
            saveConfig();
        }

        const float width = ImGui::GetContentRegionAvail().x;
        if (!sweepBusy) {
            if (ImGui::Button(("Start Sweep##wsm_start_button_" + name).c_str(), ImVec2(width, 0.0f))) {
                startSweep();
            }
        }
        else if (ImGui::Button(("Stop Sweep##wsm_stop_button_" + name).c_str(), ImVec2(width, 0.0f))) {
            requestStop("Stopped");
            pendingTuneHz = returnFrequencyHz;
            pendingTuneReason = PendingTuneReason::RESTORE_AFTER_STOP;
        }

        if (ImGui::Button(("Clear Peaks##wsm_clear_" + name).c_str(), ImVec2(width, 0.0f))) {
            clearPeaks();
        }

        drawStatus();
        drawSpectrumGraph();
        drawPeakTable();
        drawDebugInfo();

        ImGui::Spacing();
        ImGui::TextWrapped("Uses the active source sample rate and the central 85%% of every FFT segment. RTL AGC and Tuner AGC are never changed.");
    }

    void startSweep() {
        sanitizeControls();
        saveConfig();

        if (!directFFTPlan) {
            setError("The private FFT path is unavailable");
            return;
        }
        if (!workerThread.joinable()) {
            setError("Sweep worker is unavailable");
            return;
        }
        if (!gui::mainWindow.sdrIsRunning()) {
            setError("Start the SDR source before starting a sweep");
            return;
        }
        if (sigpath::sourceManager.getSelectedName().empty()) {
            setError("No active SDR source is selected");
            return;
        }

        const double sampleRateHz = sigpath::iqFrontEnd.getEffectiveSamplerate();
        if (!std::isfinite(sampleRateHz) || sampleRateHz <= 0.0) {
            setError("The active SDR sample rate is invalid");
            return;
        }

        SweepSettings settings;
        settings.startHz = startFrequencyMHz * 1000000.0;
        settings.stopHz = stopFrequencyMHz * 1000000.0;
        settings.sampleRateHz = sampleRateHz;
        settings.overlapPercent = overlapPercent;
        settings.tuningTimeMs = tuningTimeMs;
        settings.fftAveraging = fftAveraging;
        settings.thresholdDbfs = thresholdDbfs;
        settings.peakHold = peakHoldEnabled;

        if (!std::isfinite(settings.startHz) || !std::isfinite(settings.stopHz) || settings.stopHz <= settings.startHz) {
            setError("Stop frequency must be higher than start frequency");
            return;
        }
        if (!startIQCapturePath()) {
            setError("The private IQ capture path could not be started");
            return;
        }

        returnFrequencyHz = gui::waterfall.getCenterFrequency();
        {
            std::lock_guard<std::mutex> lock(requestMutex);
            queuedSettings = settings;
        }
        {
            std::lock_guard<std::mutex> lock(displayMutex);
            lastError.clear();
            statusText = completedSweepValid ? "Sweeping" : "Waiting for first complete sweep";
            activeSampleRateHz = sampleRateHz;
            activeUsableBandwidthHz = sampleRateHz * USABLE_BANDWIDTH_RATIO;
            activeSegmentStepHz = activeUsableBandwidthHz * (1.0 - (overlapPercent / 100.0));
            currentSegment = 0;
            segmentCount = 0;
            currentSegmentDebugStats = {};
        }
        sweepRequested.store(true);
        requestCv.notify_all();
        flog::info("Wide Spectrum Monitor: Starting {:.3f}-{:.3f} MHz at {:.3f} MS/s",
                   settings.startHz / 1e6, settings.stopHz / 1e6, settings.sampleRateHz / 1e6);
    }

    void requestStop(const std::string& status) {
        const bool wasRequested = sweepRequested.exchange(false);
        requestCv.notify_all();
        iqCaptureCv.notify_all();
        if (wasRequested || workerActive.load()) {
            std::lock_guard<std::mutex> lock(displayMutex);
            statusText = status;
        }
    }

    void applyPendingTune() {
        if (pendingTuneHz <= 0.0 || workerActive.load() || sweepRequested.load()) {
            return;
        }

        const double targetHz = pendingTuneHz;
        const PendingTuneReason reason = pendingTuneReason;
        pendingTuneHz = 0.0;
        pendingTuneReason = PendingTuneReason::NONE;
        tuner::centerTuning(gui::waterfall.selectedVFO, targetHz);
        {
            std::lock_guard<std::mutex> lock(displayMutex);
            statusText = (reason == PendingTuneReason::SELECTED_PEAK) ? "Tuned to selected frequency" : "Stopped";
            currentCenterHz = targetHz;
        }
        if (reason == PendingTuneReason::SELECTED_PEAK) {
            flog::info("Wide Spectrum Monitor: Tuned to selected peak {:.6f} MHz", targetHz / 1e6);
        }
        else {
            flog::info("Wide Spectrum Monitor: Restored pre-sweep frequency {:.6f} MHz", targetHz / 1e6);
        }
    }

    void clearPeaks() {
        std::lock_guard<std::mutex> lock(displayMutex);
        peakSpectrum.clear();
        lastDebugStats.peakMin = NO_DATA_DBFS;
        lastDebugStats.peakMax = NO_DATA_DBFS;
    }

    void setError(const std::string& error) {
        sweepRequested.store(false);
        requestCv.notify_all();
        iqCaptureCv.notify_all();
        std::lock_guard<std::mutex> lock(displayMutex);
        lastError = error;
        statusText = "Stopped";
        flog::error("Wide Spectrum Monitor: {}", error);
    }

    void workerLoop() {
        while (!shuttingDown.load()) {
            SweepSettings settings;
            {
                std::unique_lock<std::mutex> lock(requestMutex);
                requestCv.wait(lock, [this]() {
                    return shuttingDown.load() || sweepRequested.load();
                });
                if (shuttingDown.load()) {
                    break;
                }
                settings = queuedSettings;
                workerActive.store(true);
            }

            try {
                runSweeps(settings);
            }
            catch (const std::exception& e) {
                setError(std::string("Sweep worker failed: ") + e.what());
            }
            catch (...) {
                setError("Sweep worker failed with an unknown error");
            }
            stopIQCapturePath();
            workerActive.store(false);
        }
        workerActive.store(false);
    }

    void runSweeps(const SweepSettings& settings) {
        const std::vector<double> centers = buildSegmentCenters(settings);
        if (centers.empty()) {
            setError("Could not create sweep segments");
            return;
        }

        {
            std::lock_guard<std::mutex> lock(displayMutex);
            segmentCount = static_cast<int>(centers.size());
            statusText = completedSweepValid ? "Sweeping" : "Waiting for first complete sweep";
        }

        while (sweepRequested.load() && !shuttingDown.load()) {
            // These buffers live for the entire sweep. Segments only merge into
            // them; neither buffer is reset inside the segment loop.
            std::vector<float> cycleSpectrum(WIDE_BIN_COUNT, NO_DATA_DBFS);
            std::vector<float> cycleQuality(WIDE_BIN_COUNT, -1.0f);
            SpectrumDebugStats cycleDebugStats;
            bool capturedAnySegment = false;
            const auto sweepStartedAt = std::chrono::steady_clock::now();

            for (std::size_t segmentIndex = 0; segmentIndex < centers.size(); ++segmentIndex) {
                if (!sweepRequested.load() || shuttingDown.load()) {
                    break;
                }

                const double centerHz = centers[segmentIndex];
                {
                    std::lock_guard<std::mutex> lock(displayMutex);
                    currentSegment = static_cast<int>(segmentIndex) + 1;
                    currentCenterHz = centerHz;
                    statusText = "Settling";
                }

                sigpath::sourceManager.tune(centerHz);
                if (!waitFor(std::chrono::milliseconds(settings.tuningTimeMs))) {
                    break;
                }

                SegmentDebugStats segmentDebugStats;
                segmentDebugStats.segmentNumber = static_cast<int>(segmentIndex) + 1;
                std::vector<float> averagedFFT;
                if (!captureAveragedFFT(settings.fftAveraging, settings.sampleRateHz,
                                        averagedFFT, cycleDebugStats, segmentDebugStats)) {
                    if (!sweepRequested.load() || shuttingDown.load()) {
                        break;
                    }
                    setError("No FFT data is available from the active source");
                    return;
                }

                segmentDebugStats.wideBinsWritten = mergeSegment(
                    settings, centerHz, averagedFFT, cycleSpectrum, cycleQuality, segmentDebugStats);
                segmentDebugStats.accumulatedUniqueWideBins = static_cast<std::size_t>(std::count_if(
                    cycleSpectrum.begin(), cycleSpectrum.end(), [](float value) { return isValidDbfs(value); }));
                segmentDebugStats.wideBinsTotal = cycleSpectrum.size();
                if (segmentDebugStats.wideBinsTotal > 0) {
                    segmentDebugStats.coveragePercent =
                        100.0f * static_cast<float>(segmentDebugStats.accumulatedUniqueWideBins) /
                        static_cast<float>(segmentDebugStats.wideBinsTotal);
                }
                capturedAnySegment = true;
                {
                    std::lock_guard<std::mutex> lock(displayMutex);
                    currentSegmentDebugStats = segmentDebugStats;
                    statusText = completedSweepValid ? "Sweeping" : "Waiting for first complete sweep";
                }
            }

            if (!sweepRequested.load() || shuttingDown.load()) {
                break;
            }
            if (!capturedAnySegment) {
                setError("Sweep completed without FFT data");
                return;
            }

            const double sweepSeconds = std::chrono::duration<double>(
                                            std::chrono::steady_clock::now() - sweepStartedAt)
                                            .count();
            publishSweep(settings, std::move(cycleSpectrum), sweepSeconds, cycleDebugStats);
        }
    }

    bool waitFor(std::chrono::milliseconds duration) {
        std::unique_lock<std::mutex> lock(requestMutex);
        const bool interrupted = requestCv.wait_for(lock, duration, [this]() {
            return shuttingDown.load() || !sweepRequested.load();
        });
        return !interrupted;
    }

    std::vector<double> buildSegmentCenters(const SweepSettings& settings) const {
        const double usableBandwidth = settings.sampleRateHz * USABLE_BANDWIDTH_RATIO;
        const double range = settings.stopHz - settings.startHz;
        if (!std::isfinite(usableBandwidth) || usableBandwidth <= 0.0 || range <= 0.0) {
            return {};
        }
        if (range <= usableBandwidth) {
            return { (settings.startHz + settings.stopHz) * 0.5 };
        }

        const double step = usableBandwidth * (1.0 - (settings.overlapPercent / 100.0));
        if (step <= 0.0) {
            return {};
        }

        const double firstCenter = settings.startHz + (usableBandwidth * 0.5);
        const double lastCenter = settings.stopHz - (usableBandwidth * 0.5);
        std::vector<double> centers;
        for (double center = firstCenter; center < lastCenter; center += step) {
            if (centers.size() >= MAX_SEGMENT_COUNT) {
                return {};
            }
            centers.push_back(center);
        }
        if (centers.empty() || std::abs(centers.back() - lastCenter) > 1.0) {
            centers.push_back(lastCenter);
        }
        return centers;
    }

    bool captureIQFrame(std::size_t sampleCount, double sampleRateHz,
                        std::vector<dsp::complex_t>& samples,
                        std::uint64_t& generation) {
        if (!iqStreamBound || !iqStream || sampleCount < 2 || sampleCount > DIRECT_FFT_SIZE ||
            !std::isfinite(sampleRateHz) || sampleRateHz <= 0.0) {
            {
                std::lock_guard<std::mutex> lock(iqCaptureMutex);
                iqLastCaptureCopiedSamples = 0;
                iqLastCaptureTargetSamples = sampleCount;
                iqLastCaptureEndReason = IQCaptureEndReason::INVALID_REQUEST;
                iqLastCaptureGeneration = iqFrameGeneration;
            }
            logIQCaptureFailure(snapshotIQCaptureDiagnostics());
            return false;
        }

        const double captureMilliseconds =
            (1000.0 * static_cast<double>(sampleCount)) / sampleRateHz;
        const int timeoutMs = std::max(
            MIN_IQ_CAPTURE_TIMEOUT_MS,
            static_cast<int>(std::ceil((captureMilliseconds * 4.0) + 500.0)));

        std::unique_lock<std::mutex> lock(iqCaptureMutex);
        iqCaptureBuffer.clear();
        iqCaptureTarget = sampleCount;
        iqLastCaptureCopiedSamples = 0;
        iqLastCaptureTargetSamples = sampleCount;
        iqLastCaptureEndReason = IQCaptureEndReason::IN_PROGRESS;
        iqLastCaptureGeneration = iqFrameGeneration;
        discardNextIQChunk = true;
        const std::uint64_t requestedGeneration = iqFrameGeneration + 1;
        iqCaptureRequested = true;

        const bool signaled = iqCaptureCv.wait_for(
            lock, std::chrono::milliseconds(timeoutMs), [this, requestedGeneration]() {
                return iqFrameGeneration >= requestedGeneration || shuttingDown.load() ||
                       !sweepRequested.load();
            });
        if (!signaled || iqFrameGeneration < requestedGeneration ||
            shuttingDown.load() || !sweepRequested.load()) {
            iqLastCaptureCopiedSamples = iqCaptureBuffer.size();
            iqLastCaptureEndReason = (shuttingDown.load() || !sweepRequested.load())
                                         ? IQCaptureEndReason::CANCELLED
                                         : IQCaptureEndReason::TIMEOUT;
            iqLastCaptureGeneration = iqFrameGeneration;
            iqCaptureRequested = false;
            discardNextIQChunk = false;
            iqCaptureTarget = 0;
            iqCaptureBuffer.clear();
            lock.unlock();
            logIQCaptureFailure(snapshotIQCaptureDiagnostics());
            return false;
        }

        samples = iqCaptureBuffer;
        generation = iqFrameGeneration;
        iqLastCaptureCopiedSamples = samples.size();
        iqLastCaptureEndReason = (samples.size() == sampleCount)
                                     ? IQCaptureEndReason::COMPLETED
                                     : IQCaptureEndReason::RESULT_MISMATCH;
        iqLastCaptureGeneration = iqFrameGeneration;
        iqCaptureTarget = 0;
        const bool complete = samples.size() == sampleCount;
        lock.unlock();
        if (!complete) {
            logIQCaptureFailure(snapshotIQCaptureDiagnostics());
        }
        return complete;
    }

    bool computeDirectFFT(const std::vector<dsp::complex_t>& samples,
                          std::vector<float>& fftDbfs) {
        if (!directFFTPlan || !directFFTIn || !directFFTOut || samples.size() < 2 ||
            samples.size() > DIRECT_FFT_SIZE) {
            return false;
        }

        if (directFFTWindow.size() != samples.size()) {
            directFFTWindow.resize(samples.size());
            for (std::size_t i = 0; i < samples.size(); ++i) {
                const float shift = (i % 2) ? -1.0f : 1.0f;
                directFFTWindow[i] = static_cast<float>(
                                         dsp::window::nuttall(static_cast<double>(i),
                                                              static_cast<double>(samples.size()))) *
                                     shift;
            }
        }

        std::memset(directFFTIn, 0, sizeof(fftwf_complex) * DIRECT_FFT_SIZE);
        volk_32fc_32f_multiply_32fc(
            reinterpret_cast<lv_32fc_t*>(directFFTIn),
            reinterpret_cast<const lv_32fc_t*>(samples.data()), directFFTWindow.data(),
            static_cast<unsigned int>(samples.size()));
        fftwf_execute(directFFTPlan);

        fftDbfs.resize(DIRECT_FFT_SIZE);
        volk_32fc_s32f_power_spectrum_32f(
            fftDbfs.data(), reinterpret_cast<const lv_32fc_t*>(directFFTOut),
            static_cast<float>(DIRECT_FFT_SIZE), DIRECT_FFT_SIZE);
        return true;
    }

    bool captureAveragedFFT(int requestedFrames, double sampleRateHz,
                            std::vector<float>& averagedFFT,
                            SpectrumDebugStats& debugStats,
                            SegmentDebugStats& segmentDebugStats) {
        const long long intervalSamples = std::llround(sampleRateHz / DIRECT_FFT_RATE);
        const std::size_t iqSamplesPerFrame = static_cast<std::size_t>(
            std::clamp<long long>(intervalSamples, 2, DIRECT_FFT_SIZE));
        segmentDebugStats.iqSamplesPerFrame = iqSamplesPerFrame;

        std::vector<double> sums(DIRECT_FFT_SIZE, 0.0);
        std::vector<unsigned int> validCounts(DIRECT_FFT_SIZE, 0);
        std::vector<unsigned char> sawZero(DIRECT_FFT_SIZE, 0);
        std::vector<unsigned char> sawNonFinite(DIRECT_FFT_SIZE, 0);
        std::vector<unsigned char> sawOutOfRange(DIRECT_FFT_SIZE, 0);

        for (int capturedFrames = 0; capturedFrames < requestedFrames; ++capturedFrames) {
            std::vector<dsp::complex_t> iqSamples;
            std::uint64_t generation = 0;
            if (!captureIQFrame(iqSamplesPerFrame, sampleRateHz, iqSamples, generation)) {
                return false;
            }

            std::vector<float> fftData;
            if (!computeDirectFFT(iqSamples, fftData) || fftData.size() != DIRECT_FFT_SIZE) {
                return false;
            }

            if (segmentDebugStats.firstIQFrameGeneration == 0) {
                segmentDebugStats.firstIQFrameGeneration = generation;
            }
            segmentDebugStats.lastIQFrameGeneration = generation;

            RawFFTDistribution frameDistribution;
            frameDistribution.totalBins = fftData.size();
            for (std::size_t i = 0; i < fftData.size(); ++i) {
                const float value = fftData[i];
                recordRawFFTValue(value, i, fftData.size(), frameDistribution);
                if (!std::isfinite(value)) {
                    sawNonFinite[i] = 1;
                }
                else if (value > GRAPH_MAX_DB || value < MIN_VALID_FFT_DB) {
                    sawOutOfRange[i] = 1;
                }
                else if (value == 0.0f) {
                    sawZero[i] = 1;
                }
                if (!validateFFTDbfs(value, debugStats)) {
                    continue;
                }
                sums[i] += value;
                ++validCounts[i];
            }

            const std::size_t sampleIndices[RawFFTDistribution::SAMPLE_COUNT] = {
                0,
                1,
                fftData.size() / 8,
                fftData.size() / 4,
                fftData.size() / 2,
                (fftData.size() * 3) / 4,
                (fftData.size() * 7) / 8,
                fftData.size() - 2,
                fftData.size() - 1
            };
            for (std::size_t sample = 0; sample < RawFFTDistribution::SAMPLE_COUNT; ++sample) {
                frameDistribution.sampleIndices[sample] = sampleIndices[sample];
                frameDistribution.sampleValues[sample] = fftData[sampleIndices[sample]];
            }
            segmentDebugStats.rawDistribution = frameDistribution;
        }

        averagedFFT.resize(sums.size());
        for (std::size_t i = 0; i < sums.size(); ++i) {
            averagedFFT[i] = validCounts[i] > 0
                                 ? static_cast<float>(sums[i] / static_cast<double>(validCounts[i]))
                                 : NO_DATA_DBFS;
            if (validCounts[i] > 0) {
                ++segmentDebugStats.validRawBins;
            }
            else {
                segmentDebugStats.rejectedZeroBins += sawZero[i] != 0;
                segmentDebugStats.rejectedNonFiniteBins += sawNonFinite[i] != 0;
                segmentDebugStats.rejectedOutOfRangeBins += sawOutOfRange[i] != 0;
            }
        }
        segmentDebugStats.rawFftBinCount = averagedFFT.size();
        return true;
    }

    std::size_t mergeSegment(const SweepSettings& settings, double centerHz, const std::vector<float>& fft,
                             std::vector<float>& output, std::vector<float>& quality,
                             SegmentDebugStats& segmentDebugStats) const {
        if (fft.size() < 2 || output.empty() || output.size() != quality.size()) {
            return 0;
        }

        const double range = settings.stopHz - settings.startHz;
        const double usableBandwidth = settings.sampleRateHz * USABLE_BANDWIDTH_RATIO;
        const double usableHalf = usableBandwidth * 0.5;
        const double rawStartHz = centerHz - (settings.sampleRateHz * 0.5);
        const double rawStopHz = centerHz + (settings.sampleRateHz * 0.5);
        const double usableStartHz = centerHz - usableHalf;
        const double usableStopHz = centerHz + usableHalf;
        const double effectiveUsableStartHz = std::max(settings.startHz, usableStartHz);
        const double effectiveUsableStopHz = std::min(settings.stopHz, usableStopHz);
        const double rawBinWidthHz = settings.sampleRateHz / static_cast<double>(fft.size());
        const double wideBinWidthHz = range / static_cast<double>(output.size());

        segmentDebugStats.rawStartHz = rawStartHz;
        segmentDebugStats.rawStopHz = rawStopHz;
        segmentDebugStats.usableStartHz = effectiveUsableStartHz;
        segmentDebugStats.usableStopHz = effectiveUsableStopHz;

        // Project every valid raw FFT bin exactly once. A wide bin is much wider
        // than a raw bin, so every raw bin landing in it participates in the max
        // reduction instead of relying on rounded, output-driven index ranges.
        std::vector<float> segmentSpectrum(output.size(), NO_DATA_DBFS);
        for (std::size_t rawIndex = 0; rawIndex < fft.size(); ++rawIndex) {
            const float candidate = fft[rawIndex];
            if (!isValidDbfs(candidate)) {
                continue;
            }

            const double frequencyHz = rawStartHz +
                                       ((static_cast<double>(rawIndex) + 0.5) * rawBinWidthHz);
            if (frequencyHz < effectiveUsableStartHz || frequencyHz >= effectiveUsableStopHz) {
                continue;
            }

            const double widePosition = (frequencyHz - settings.startHz) / wideBinWidthHz;
            if (widePosition < 0.0 || widePosition >= static_cast<double>(output.size())) {
                continue;
            }
            const std::size_t wideIndex = static_cast<std::size_t>(std::floor(widePosition));
            float& segmentLevel = segmentSpectrum[wideIndex];
            if (!std::isfinite(segmentLevel) || candidate > segmentLevel) {
                segmentLevel = candidate;
            }
        }

        std::size_t writtenBins = 0;
        for (std::size_t wideIndex = 0; wideIndex < output.size(); ++wideIndex) {
            const float level = segmentSpectrum[wideIndex];
            if (!isValidDbfs(level)) {
                continue;
            }

            const double frequencyHz = settings.startHz +
                                       ((static_cast<double>(wideIndex) + 0.5) * wideBinWidthHz);
            const float segmentQuality = static_cast<float>(
                1.0 - (std::abs(frequencyHz - centerHz) / usableHalf));
            if (segmentQuality <= quality[wideIndex]) {
                continue;
            }

            output[wideIndex] = level;
            quality[wideIndex] = segmentQuality;
            ++writtenBins;
        }
        return writtenBins;
    }

    void publishSweep(const SweepSettings& settings, std::vector<float> spectrum, double sweepSeconds,
                      SpectrumDebugStats debugStats) {
        debugStats.totalWideBins = spectrum.size();
        for (float level : spectrum) {
            if (!isValidDbfs(level)) {
                continue;
            }
            ++debugStats.validWideBins;
            includeInRange(level, debugStats.wideMin, debugStats.wideMax);
        }
        if (debugStats.totalWideBins > 0) {
            debugStats.validWidePercent = 100.0f * static_cast<float>(debugStats.validWideBins) /
                                          static_cast<float>(debugStats.totalWideBins);
        }

        if (debugStats.validWidePercent < MIN_VALID_SWEEP_PERCENT) {
            char warning[160];
            std::snprintf(warning, sizeof(warning),
                          "Sweep not published: only %.1f%% of wide-spectrum bins contain valid FFT data",
                          debugStats.validWidePercent);
            std::lock_guard<std::mutex> lock(displayMutex);
            lastDebugStats = debugStats;
            lastError = warning;
            statusText = completedSweepValid ? "Sweeping (invalid sweep discarded)" : "Waiting for valid complete sweep";
            return;
        }

        std::vector<DetectedPeak> peaks = detectPeaks(settings, spectrum);
        std::lock_guard<std::mutex> lock(displayMutex);
        const bool rangeChanged = !completedSweepValid ||
                                  std::abs(displayedStartHz - settings.startHz) > 1.0 ||
                                  std::abs(displayedStopHz - settings.stopHz) > 1.0;
        completedSweep = std::move(spectrum);
        detectedPeaks = std::move(peaks);
        lastError.clear();
        displayedStartHz = settings.startHz;
        displayedStopHz = settings.stopHz;
        activeSampleRateHz = settings.sampleRateHz;
        activeUsableBandwidthHz = settings.sampleRateHz * USABLE_BANDWIDTH_RATIO;
        activeSegmentStepHz = activeUsableBandwidthHz * (1.0 - (settings.overlapPercent / 100.0));
        completedSweepValid = true;
        currentSegment = segmentCount;
        lastSweepSeconds = sweepSeconds;
        sweepsPerMinute = (sweepSeconds > 0.0) ? (60.0 / sweepSeconds) : 0.0;
        ++completedSweeps;
        statusText = "Sweeping";

        if (settings.peakHold) {
            if (rangeChanged || peakSpectrum.size() != completedSweep.size()) {
                peakSpectrum.assign(completedSweep.size(), NO_DATA_DBFS);
            }
            for (std::size_t i = 0; i < completedSweep.size(); ++i) {
                if (!isValidDbfs(completedSweep[i])) {
                    continue;
                }
                if (!isValidDbfs(peakSpectrum[i]) || completedSweep[i] > peakSpectrum[i]) {
                    peakSpectrum[i] = completedSweep[i];
                }
            }
            for (float level : peakSpectrum) {
                if (isValidDbfs(level)) {
                    includeInRange(level, debugStats.peakMin, debugStats.peakMax);
                }
            }
        }
        else if (rangeChanged) {
            peakSpectrum.clear();
        }
        lastDebugStats = debugStats;
    }

    std::vector<DetectedPeak> detectPeaks(const SweepSettings& settings, const std::vector<float>& spectrum) const {
        std::vector<DetectedPeak> candidates;
        if (spectrum.size() < 3) {
            return candidates;
        }

        const double binWidthHz = (settings.stopHz - settings.startHz) / spectrum.size();
        for (std::size_t i = 1; i + 1 < spectrum.size(); ++i) {
            const float level = spectrum[i];
            if (!isValidDbfs(spectrum[i - 1]) || !isValidDbfs(level) || !isValidDbfs(spectrum[i + 1])) {
                continue;
            }
            if (level < settings.thresholdDbfs || level < spectrum[i - 1] || level <= spectrum[i + 1]) {
                continue;
            }
            candidates.push_back({ settings.startHz + ((static_cast<double>(i) + 0.5) * binWidthHz), level });
        }

        std::sort(candidates.begin(), candidates.end(), [](const DetectedPeak& lhs, const DetectedPeak& rhs) {
            return lhs.levelDbfs > rhs.levelDbfs;
        });

        const double mergeDistanceHz = std::max(250000.0, binWidthHz * 3.0);
        std::vector<DetectedPeak> strongest;
        strongest.reserve(20);
        for (const DetectedPeak& candidate : candidates) {
            const bool nearExisting = std::any_of(strongest.begin(), strongest.end(), [&](const DetectedPeak& existing) {
                return std::abs(existing.frequencyHz - candidate.frequencyHz) < mergeDistanceHz;
            });
            if (!nearExisting) {
                strongest.push_back(candidate);
                if (strongest.size() == 20) {
                    break;
                }
            }
        }
        return strongest;
    }

    void drawStatus() {
        std::string status;
        std::string error;
        int segment = 0;
        int totalSegments = 0;
        int sweeps = 0;
        double centerHz = 0.0;
        double sampleRateHz = 0.0;
        double sweepSeconds = 0.0;
        double sweepsPerMinuteValue = 0.0;
        {
            std::lock_guard<std::mutex> lock(displayMutex);
            status = statusText;
            error = lastError;
            segment = currentSegment;
            totalSegments = segmentCount;
            sweeps = completedSweeps;
            centerHz = currentCenterHz;
            sampleRateHz = activeSampleRateHz;
            sweepSeconds = lastSweepSeconds;
            sweepsPerMinuteValue = sweepsPerMinute;
        }

        ImGui::Separator();
        ImGui::Text("Status: %s", status.c_str());
        if (totalSegments > 0) {
            ImGui::Text("Segment: %d / %d | Sweeps: %d", segment, totalSegments, sweeps);
        }
        if (centerHz > 0.0) {
            ImGui::Text("Center: %.3f MHz | Rate: %.3f MS/s", centerHz / 1e6, sampleRateHz / 1e6);
        }
        if (sweepSeconds > 0.0) {
            ImGui::Text("Last Sweep: %.1f s | %.2f sweeps/min", sweepSeconds, sweepsPerMinuteValue);
        }
        if (!error.empty()) {
            ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.25f, 1.0f), "%s", error.c_str());
        }
    }

    void drawSpectrumGraph() {
        std::vector<float> spectrum;
        std::vector<float> heldPeaks;
        double startHz = startFrequencyMHz * 1e6;
        double stopHz = stopFrequencyMHz * 1e6;
        {
            std::lock_guard<std::mutex> lock(displayMutex);
            spectrum = completedSweep;
            heldPeaks = peakSpectrum;
            if (displayedStopHz > displayedStartHz) {
                startHz = displayedStartHz;
                stopHz = displayedStopHz;
            }
        }

        ImGui::TextUnformatted("Combined Spectrum");
        const float scale = style::uiScale;
        const ImVec2 canvasPosition = ImGui::GetCursorScreenPos();
        const ImVec2 canvasSize(std::max(120.0f, ImGui::GetContentRegionAvail().x), 220.0f * scale);
        ImGui::InvisibleButton(("##wsm_graph_" + name).c_str(), canvasSize);

        ImDrawList* drawList = ImGui::GetWindowDrawList();
        const ImVec2 canvasEnd(canvasPosition.x + canvasSize.x, canvasPosition.y + canvasSize.y);
        drawList->AddRectFilled(canvasPosition, canvasEnd, IM_COL32(12, 16, 22, 255));
        drawList->AddRect(canvasPosition, canvasEnd, IM_COL32(90, 100, 115, 255));

        const ImVec2 plotMin(canvasPosition.x + (45.0f * scale), canvasPosition.y + (8.0f * scale));
        const ImVec2 plotMax(canvasEnd.x - (8.0f * scale), canvasEnd.y - (24.0f * scale));
        if (plotMax.x <= plotMin.x || plotMax.y <= plotMin.y) {
            return;
        }

        auto levelToY = [&](float level) {
            const float clamped = std::clamp(level, GRAPH_MIN_DB, GRAPH_MAX_DB);
            const float ratio = (clamped - GRAPH_MIN_DB) / (GRAPH_MAX_DB - GRAPH_MIN_DB);
            return plotMax.y - (ratio * (plotMax.y - plotMin.y));
        };

        char label[64];
        for (int tick = 0; tick <= 4; ++tick) {
            const float level = GRAPH_MIN_DB + ((GRAPH_MAX_DB - GRAPH_MIN_DB) * tick / 4.0f);
            const float y = levelToY(level);
            drawList->AddLine(ImVec2(plotMin.x, y), ImVec2(plotMax.x, y), IM_COL32(50, 58, 68, 255));
            std::snprintf(label, sizeof(label), "%.0f", level);
            drawList->AddText(ImVec2(canvasPosition.x + (3.0f * scale), y - (7.0f * scale)), IM_COL32(180, 185, 195, 255), label);
        }

        for (int tick = 0; tick <= 2; ++tick) {
            const float ratio = tick / 2.0f;
            const float x = plotMin.x + (ratio * (plotMax.x - plotMin.x));
            const double frequencyMHz = (startHz + (ratio * (stopHz - startHz))) / 1e6;
            drawList->AddLine(ImVec2(x, plotMin.y), ImVec2(x, plotMax.y), IM_COL32(42, 48, 58, 255));
            std::snprintf(label, sizeof(label), "%.1f MHz", frequencyMHz);
            const ImVec2 textSize = ImGui::CalcTextSize(label);
            drawList->AddText(ImVec2(std::clamp(x - (textSize.x * 0.5f), plotMin.x, plotMax.x - textSize.x), plotMax.y + (4.0f * scale)),
                              IM_COL32(180, 185, 195, 255), label);
        }

        const float thresholdY = levelToY(thresholdDbfs);
        drawList->AddLine(ImVec2(plotMin.x, thresholdY), ImVec2(plotMax.x, thresholdY), IM_COL32(255, 165, 40, 255), 1.5f * scale);
        std::snprintf(label, sizeof(label), "Threshold %.1f dBFS", thresholdDbfs);
        drawList->AddText(ImVec2(plotMin.x + (4.0f * scale), std::max(plotMin.y, thresholdY - (17.0f * scale))),
                          IM_COL32(255, 180, 65, 255), label);

        if (spectrum.empty()) {
            const char* waitingText = "Waiting for first complete sweep";
            const ImVec2 textSize = ImGui::CalcTextSize(waitingText);
            drawList->AddText(ImVec2(plotMin.x + ((plotMax.x - plotMin.x - textSize.x) * 0.5f),
                                     plotMin.y + ((plotMax.y - plotMin.y - textSize.y) * 0.5f)),
                              IM_COL32(205, 210, 220, 255), waitingText);
        }

        auto drawTrace = [&](const std::vector<float>& data, ImU32 color, float thickness) {
            if (data.size() < 2) {
                return;
            }
            bool havePrevious = false;
            ImVec2 previous;
            for (std::size_t i = 0; i < data.size(); ++i) {
                if (!isValidDbfs(data[i])) {
                    havePrevious = false;
                    continue;
                }
                const float xRatio = static_cast<float>(i) / static_cast<float>(data.size() - 1);
                const ImVec2 point(plotMin.x + (xRatio * (plotMax.x - plotMin.x)), levelToY(data[i]));
                if (havePrevious) {
                    drawList->AddLine(previous, point, color, thickness * scale);
                }
                previous = point;
                havePrevious = true;
            }
        };

        drawTrace(spectrum, IM_COL32(40, 220, 150, 255), 1.5f);
        if (peakHoldEnabled) {
            drawTrace(heldPeaks, IM_COL32(255, 215, 65, 220), 1.0f);
        }
    }

    void drawPeakTable() {
        std::vector<DetectedPeak> peaks;
        bool valid = false;
        {
            std::lock_guard<std::mutex> lock(displayMutex);
            peaks = detectedPeaks;
            valid = completedSweepValid;
        }

        ImGui::TextUnformatted("Strongest Detected Peaks");
        if (!valid) {
            ImGui::TextDisabled("Waiting for first complete sweep.");
            return;
        }
        if (peaks.empty()) {
            ImGui::TextDisabled("No peaks above threshold in the last completed sweep.");
            return;
        }

        const ImGuiTableFlags flags = ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                                      ImGuiTableFlags_ScrollY | ImGuiTableFlags_SizingStretchProp;
        if (!ImGui::BeginTable(("##wsm_peak_table_" + name).c_str(), 2, flags, ImVec2(0.0f, 180.0f * style::uiScale))) {
            return;
        }

        ImGui::TableSetupColumn("Frequency MHz", ImGuiTableColumnFlags_WidthStretch, 2.0f);
        ImGui::TableSetupColumn("Level dBFS", ImGuiTableColumnFlags_WidthStretch, 1.0f);
        ImGui::TableHeadersRow();
        for (std::size_t i = 0; i < peaks.size(); ++i) {
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            char frequencyLabel[96];
            std::snprintf(frequencyLabel, sizeof(frequencyLabel), "%.6f##wsm_peak_%zu_%s", peaks[i].frequencyHz / 1e6, i, name.c_str());
            if (ImGui::Selectable(frequencyLabel, false, ImGuiSelectableFlags_SpanAllColumns)) {
                requestStop("Stopped");
                pendingTuneHz = peaks[i].frequencyHz;
                pendingTuneReason = PendingTuneReason::SELECTED_PEAK;
            }
            ImGui::TableSetColumnIndex(1);
            ImGui::Text("%.1f", peaks[i].levelDbfs);
        }
        ImGui::EndTable();
        ImGui::TextDisabled("Tap a row to stop the sweep and tune SDR++.");
    }

    void drawDebugInfo() {
        if (!ImGui::CollapsingHeader(("Debug##wsm_debug_" + name).c_str())) {
            return;
        }

        int segment = 0;
        int totalSegments = 0;
        int completedCount = 0;
        bool completedValid = false;
        std::size_t completedBins = 0;
        double sweepSeconds = 0.0;
        double sampleRateHz = 0.0;
        double usableBandwidthHz = 0.0;
        double segmentStepHz = 0.0;
        SpectrumDebugStats debugStats;
        SegmentDebugStats segmentDebugStats;
        {
            std::lock_guard<std::mutex> lock(displayMutex);
            segment = currentSegment;
            totalSegments = segmentCount;
            completedCount = completedSweeps;
            completedValid = completedSweepValid;
            completedBins = completedSweep.size();
            sweepSeconds = lastSweepSeconds;
            sampleRateHz = activeSampleRateHz;
            usableBandwidthHz = activeUsableBandwidthHz;
            segmentStepHz = activeSegmentStepHz;
            debugStats = lastDebugStats;
            segmentDebugStats = currentSegmentDebugStats;
        }
        const IQCaptureDiagnostics iqDiagnostics = snapshotIQCaptureDiagnostics();

        ImGui::Text("Worker active: %s", workerActive.load() ? "yes" : "no");
        ImGui::Text("Working segment: %d / %d", segment, totalSegments);
        ImGui::Text("Completed sweeps: %d", completedCount);
        ImGui::Text("Completed sweep valid: %s", completedValid ? "yes" : "no");
        ImGui::Text("Completed bins: %zu", completedBins);
        ImGui::Text("Last sweep duration: %.3f s", sweepSeconds);
        ImGui::Text("Sample rate: %.3f MS/s", sampleRateHz / 1e6);
        ImGui::Text("Usable bandwidth: %.3f MHz", usableBandwidthHz / 1e6);
        ImGui::Text("Segment center step: %.3f MHz", segmentStepHz / 1e6);
        ImGui::Separator();
        ImGui::TextUnformatted("IQ capture diagnostic gate");
        ImGui::Text("Handler calls (valid): %llu", static_cast<unsigned long long>(iqDiagnostics.handlerCalls));
        ImGui::Text("IQ samples presented to handler: %llu", static_cast<unsigned long long>(iqDiagnostics.handlerSamples));
        ImGui::Text("Handler calls during capture request: %llu", static_cast<unsigned long long>(iqDiagnostics.requestedHandlerCalls));
        ImGui::Text("IQ samples during capture request: %llu", static_cast<unsigned long long>(iqDiagnostics.requestedHandlerSamples));
        ImGui::Text("Intentionally discarded chunks: %llu", static_cast<unsigned long long>(iqDiagnostics.discardedChunks));
        ImGui::Text("IQ samples copied: %zu / %zu", iqDiagnostics.copiedSamples, iqDiagnostics.targetSamples);
        ImGui::Text("Last handler chunk size: %d", iqDiagnostics.lastHandlerChunkSize);
        ImGui::Text("Last capture result: %s", iqCaptureEndReasonText(iqDiagnostics.endReason));
        ImGui::Text("IQ frame generation at attempt end: %llu", static_cast<unsigned long long>(iqDiagnostics.generation));
        ImGui::Separator();
        ImGui::TextUnformatted("FFT source: private operation-bound IQFrontEnd stream");
        ImGui::Text("Current segment data: %d / %d", segmentDebugStats.segmentNumber, totalSegments);
        ImGui::Text("Direct FFT bin count: %zu", segmentDebugStats.rawFftBinCount);
        ImGui::Text("IQ samples per FFT frame: %zu", segmentDebugStats.iqSamplesPerFrame);
        ImGui::Text("IQ frame generation first/last: %llu / %llu",
                    static_cast<unsigned long long>(segmentDebugStats.firstIQFrameGeneration),
                    static_cast<unsigned long long>(segmentDebugStats.lastIQFrameGeneration));
        ImGui::Text("Valid direct FFT bins: %zu", segmentDebugStats.validRawBins);
        ImGui::Text("Rejected zero bins: %zu", segmentDebugStats.rejectedZeroBins);
        ImGui::Text("Rejected non-finite bins: %zu", segmentDebugStats.rejectedNonFiniteBins);
        ImGui::Text("Rejected out-of-range bins: %zu", segmentDebugStats.rejectedOutOfRangeBins);
        ImGui::Text("Current segment wide bins written: %zu", segmentDebugStats.wideBinsWritten);
        ImGui::Text("Accumulated unique wide bins: %zu", segmentDebugStats.accumulatedUniqueWideBins);
        ImGui::Text("Coverage: %.1f%%", segmentDebugStats.coveragePercent);
        ImGui::Text("Wide bins total: %zu", segmentDebugStats.wideBinsTotal);
        ImGui::Text("Raw FFT frequency start/end: %.6f / %.6f MHz",
                    segmentDebugStats.rawStartHz / 1e6, segmentDebugStats.rawStopHz / 1e6);
        ImGui::Text("Effective usable start/end: %.6f / %.6f MHz",
                    segmentDebugStats.usableStartHz / 1e6, segmentDebugStats.usableStopHz / 1e6);
        drawRawFFTDistribution(segmentDebugStats.rawDistribution);
        ImGui::Separator();
        ImGui::Text("FFT values: direct VOLK logarithmic power (no additional conversion)");
        drawDebugRange("FFT raw min/max", debugStats.rawMin, debugStats.rawMax);
        drawDebugRange("FFT dBFS min/max", debugStats.convertedMin, debugStats.convertedMax);
        ImGui::Text("Valid FFT samples: %llu", static_cast<unsigned long long>(debugStats.validSamples));
        ImGui::Text("Invalid FFT samples: %llu", static_cast<unsigned long long>(debugStats.invalidSamples));
        ImGui::Text("NaN / Inf: %llu / %llu",
                    static_cast<unsigned long long>(debugStats.nanSamples),
                    static_cast<unsigned long long>(debugStats.infSamples));
        ImGui::Text("Above 0 / below -200 dBFS: %llu / %llu",
                    static_cast<unsigned long long>(debugStats.aboveZeroSamples),
                    static_cast<unsigned long long>(debugStats.belowMinus200Samples));
        ImGui::Text("Zero/default samples: %llu",
                    static_cast<unsigned long long>(debugStats.zeroDefaultSamples));
        ImGui::Text("Completed valid bins: %.1f%% (%zu / %zu)",
                    debugStats.validWidePercent, debugStats.validWideBins, debugStats.totalWideBins);
        drawDebugRange("Wide spectrum min/max", debugStats.wideMin, debugStats.wideMax);
        drawDebugRange("Peak Hold min/max", debugStats.peakMin, debugStats.peakMax);
    }

    static void drawDebugRange(const char* label, float minimum, float maximum) {
        if (std::isfinite(minimum) && std::isfinite(maximum)) {
            ImGui::Text("%s: %.1f / %.1f dBFS", label, minimum, maximum);
        }
        else {
            ImGui::Text("%s: no valid data", label);
        }
    }

    static void drawRawCount(const char* label, std::size_t count, std::size_t total) {
        const double percent = total > 0
                                   ? (100.0 * static_cast<double>(count) / static_cast<double>(total))
                                   : 0.0;
        ImGui::Text("%s: %zu (%.3f%%)", label, count, percent);
    }

    static void drawRawFFTDistribution(const RawFFTDistribution& distribution) {
        if (!ImGui::CollapsingHeader("Direct FFT distribution before validation",
                                     ImGuiTreeNodeFlags_DefaultOpen)) {
            return;
        }

        ImGui::Text("Expected format: logarithmic power (dBFS-like), not linear magnitude");
        ImGui::Text("Distribution frame bins: %zu", distribution.totalBins);
        if (std::isfinite(distribution.finiteMin) && std::isfinite(distribution.finiteMax)) {
            ImGui::Text("Direct finite min/max: %.9g / %.9g", distribution.finiteMin,
                        distribution.finiteMax);
        }
        else {
            ImGui::TextUnformatted("Direct finite min/max: no finite data");
        }

        drawRawCount("Finite", distribution.finiteBins, distribution.totalBins);
        drawRawCount("Finite negative", distribution.negativeFiniteBins, distribution.totalBins);
        drawRawCount("Finite positive", distribution.positiveFiniteBins, distribution.totalBins);
        drawRawCount("Exact +0/-0", distribution.exactZeroBins, distribution.totalBins);
        drawRawCount("Negative zero", distribution.negativeZeroBins, distribution.totalBins);
        drawRawCount("Subnormal", distribution.subnormalBins, distribution.totalBins);
        drawRawCount("NaN", distribution.nanBins, distribution.totalBins);
        drawRawCount("+Inf", distribution.positiveInfBins, distribution.totalBins);
        drawRawCount("-Inf", distribution.negativeInfBins, distribution.totalBins);

        ImGui::Separator();
        ImGui::TextUnformatted("Value histogram (one private FFT frame)");
        drawRawCount("< -200", distribution.belowMinus200Bins, distribution.totalBins);
        drawRawCount("[-200, -160)", distribution.minus200ToMinus160Bins, distribution.totalBins);
        drawRawCount("[-160, -120)", distribution.minus160ToMinus120Bins, distribution.totalBins);
        drawRawCount("[-120, -100)", distribution.minus120ToMinus100Bins, distribution.totalBins);
        drawRawCount("[-100, -80)", distribution.minus100ToMinus80Bins, distribution.totalBins);
        drawRawCount("[-80, -60)", distribution.minus80ToMinus60Bins, distribution.totalBins);
        drawRawCount("[-60, -40)", distribution.minus60ToMinus40Bins, distribution.totalBins);
        drawRawCount("[-40, -20)", distribution.minus40ToMinus20Bins, distribution.totalBins);
        drawRawCount("[-20, 0)", distribution.minus20ToZeroBins, distribution.totalBins);
        drawRawCount("= 0", distribution.exactZeroBins, distribution.totalBins);
        drawRawCount("(0, 1]", distribution.zeroToOneBins, distribution.totalBins);
        drawRawCount("(1, 100]", distribution.oneTo100Bins, distribution.totalBins);
        drawRawCount("> 100", distribution.above100Bins, distribution.totalBins);

        ImGui::Separator();
        if (distribution.firstNonZeroBin != std::numeric_limits<std::size_t>::max()) {
            ImGui::Text("First/last non-zero index: %zu / %zu", distribution.firstNonZeroBin,
                        distribution.lastNonZeroBin);
        }
        else {
            ImGui::TextUnformatted("First/last non-zero index: none");
        }
        ImGui::Text("Longest consecutive zero run: %zu", distribution.longestZeroRun);
        for (std::size_t quartile = 0; quartile < 4; ++quartile) {
            ImGui::Text("Q%zu valid/zero/nonfinite/out: %zu / %zu / %zu / %zu", quartile + 1,
                        distribution.quartileValidBins[quartile],
                        distribution.quartileZeroBins[quartile],
                        distribution.quartileNonFiniteBins[quartile],
                        distribution.quartileOutOfRangeBins[quartile]);
        }

        ImGui::Separator();
        ImGui::TextUnformatted("Fixed-index direct FFT samples");
        for (std::size_t sample = 0; sample < RawFFTDistribution::SAMPLE_COUNT; ++sample) {
            ImGui::Text("[%zu] = %.9g", distribution.sampleIndices[sample],
                        distribution.sampleValues[sample]);
        }
    }

    std::string name;
    bool enabled = true;

    double startFrequencyMHz = 225.0;
    double stopFrequencyMHz = 379.0;
    float overlapPercent = 10.0f;
    int tuningTimeMs = 120;
    int fftAveraging = 3;
    float thresholdDbfs = -55.0f;
    bool peakHoldEnabled = true;

    std::atomic<bool> shuttingDown{ false };
    std::atomic<bool> sweepRequested{ false };
    std::atomic<bool> workerActive{ false };
    std::thread workerThread;
    std::mutex requestMutex;
    std::condition_variable requestCv;
    SweepSettings queuedSettings;

    dsp::stream<dsp::complex_t>* iqStream = nullptr;
    dsp::sink::Handler<dsp::complex_t> iqSink;
    bool iqStreamBound = false;
    std::mutex iqCaptureMutex;
    std::condition_variable iqCaptureCv;
    std::atomic<std::uint64_t> iqHandlerCalls{ 0 };
    std::atomic<std::uint64_t> iqHandlerSamples{ 0 };
    std::atomic<std::uint64_t> iqRequestedHandlerCalls{ 0 };
    std::atomic<std::uint64_t> iqRequestedHandlerSamples{ 0 };
    std::atomic<std::uint64_t> iqDiscardedChunks{ 0 };
    std::atomic<int> iqLastHandlerChunkSize{ 0 };
    std::vector<dsp::complex_t> iqCaptureBuffer;
    std::size_t iqCaptureTarget = 0;
    bool iqCaptureRequested = false;
    bool discardNextIQChunk = false;
    std::uint64_t iqFrameGeneration = 0;
    std::size_t iqLastCaptureCopiedSamples = 0;
    std::size_t iqLastCaptureTargetSamples = 0;
    IQCaptureEndReason iqLastCaptureEndReason = IQCaptureEndReason::NONE;
    std::uint64_t iqLastCaptureGeneration = 0;

    fftwf_complex* directFFTIn = nullptr;
    fftwf_complex* directFFTOut = nullptr;
    fftwf_plan directFFTPlan = nullptr;
    std::vector<float> directFFTWindow;

    std::mutex displayMutex;
    std::vector<float> completedSweep;
    std::vector<float> peakSpectrum;
    std::vector<DetectedPeak> detectedPeaks;
    std::string statusText = "Ready";
    std::string lastError;
    double displayedStartHz = 0.0;
    double displayedStopHz = 0.0;
    double activeSampleRateHz = 0.0;
    double activeUsableBandwidthHz = 0.0;
    double activeSegmentStepHz = 0.0;
    double currentCenterHz = 0.0;
    int currentSegment = 0;
    int segmentCount = 0;
    int completedSweeps = 0;
    bool completedSweepValid = false;
    double lastSweepSeconds = 0.0;
    double sweepsPerMinute = 0.0;
    SpectrumDebugStats lastDebugStats;
    SegmentDebugStats currentSegmentDebugStats;

    double returnFrequencyHz = 0.0;
    double pendingTuneHz = 0.0;
    PendingTuneReason pendingTuneReason = PendingTuneReason::NONE;
    EventHandler<bool> playStateHandler;
};

MOD_EXPORT void _INIT_() {
    json defaults = json::object();
    defaults["startFrequencyMHz"] = 225.0;
    defaults["stopFrequencyMHz"] = 379.0;
    defaults["overlapPercent"] = 10.0;
    defaults["tuningTimeMs"] = 120;
    defaults["fftAveraging"] = 3;
    defaults["thresholdDbfs"] = -55.0;
    defaults["peakHold"] = true;

    config.setPath(core::args["root"].s() + "/wide_spectrum_monitor_config.json");
    config.load(defaults);
    config.enableAutoSave();
}

MOD_EXPORT ModuleManager::Instance* _CREATE_INSTANCE_(std::string name) {
    return new WideSpectrumMonitorModule(std::move(name));
}

MOD_EXPORT void _DELETE_INSTANCE_(void* instance) {
    delete static_cast<WideSpectrumMonitorModule*>(instance);
}

MOD_EXPORT void _END_() {
    config.disableAutoSave();
    config.save();
}
