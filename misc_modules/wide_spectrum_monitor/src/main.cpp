#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdio>
#include <exception>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <config.h>
#include <core.h>
#include <gui/gui.h>
#include <gui/style.h>
#include <gui/tuner.h>
#include <imgui.h>
#include <module.h>
#include <signal_path/signal_path.h>
#include <utils/flog.h>

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
    constexpr std::size_t MAX_SEGMENT_COUNT = 4096;
    constexpr double USABLE_BANDWIDTH_RATIO = 0.85;
    constexpr int FFT_FRAME_WAIT_MS = 55;
    constexpr float GRAPH_MIN_DB = -120.0f;
    constexpr float GRAPH_MAX_DB = 0.0f;
    constexpr float EMPTY_BIN_DB = -160.0f;

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
        float levelDbfs = EMPTY_BIN_DB;
    };
}

class WideSpectrumMonitorModule : public ModuleManager::Instance {
public:
    explicit WideSpectrumMonitorModule(std::string instanceName) : name(std::move(instanceName)) {
        loadConfig();

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
        if (workerThread.joinable()) {
            workerThread.join();
        }
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
            requestStop("Stopping sweep...");
            pendingTuneHz = returnFrequencyHz;
        }

        if (ImGui::Button(("Clear Peaks##wsm_clear_" + name).c_str(), ImVec2(width, 0.0f))) {
            clearPeaks();
        }

        drawStatus();
        drawSpectrumGraph();
        drawPeakTable();

        ImGui::Spacing();
        ImGui::TextWrapped("Uses the active source sample rate and the central 85%% of every FFT segment. RTL AGC and Tuner AGC are never changed.");
    }

    void startSweep() {
        sanitizeControls();
        saveConfig();

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

        returnFrequencyHz = gui::waterfall.getCenterFrequency();
        {
            std::lock_guard<std::mutex> lock(requestMutex);
            queuedSettings = settings;
        }
        {
            std::lock_guard<std::mutex> lock(displayMutex);
            lastError.clear();
            statusText = "Starting sweep...";
            activeSampleRateHz = sampleRateHz;
            currentSegment = 0;
            segmentCount = 0;
        }
        sweepRequested.store(true);
        requestCv.notify_all();
        flog::info("Wide Spectrum Monitor: Starting {:.3f}-{:.3f} MHz at {:.3f} MS/s",
                   settings.startHz / 1e6, settings.stopHz / 1e6, settings.sampleRateHz / 1e6);
    }

    void requestStop(const std::string& status) {
        const bool wasRequested = sweepRequested.exchange(false);
        requestCv.notify_all();
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
        pendingTuneHz = 0.0;
        tuner::centerTuning(gui::waterfall.selectedVFO, targetHz);
        {
            std::lock_guard<std::mutex> lock(displayMutex);
            statusText = "Tuned to selected frequency";
            currentCenterHz = targetHz;
        }
        flog::info("Wide Spectrum Monitor: Tuned to selected peak {:.6f} MHz", targetHz / 1e6);
    }

    void clearPeaks() {
        std::lock_guard<std::mutex> lock(displayMutex);
        peakSpectrum.assign(currentSpectrum.size(), EMPTY_BIN_DB);
    }

    void setError(const std::string& error) {
        sweepRequested.store(false);
        requestCv.notify_all();
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
            workerActive.store(false);

            if (!shuttingDown.load() && !sweepRequested.load()) {
                std::lock_guard<std::mutex> lock(displayMutex);
                if (lastError.empty() && statusText == "Stopping sweep...") {
                    statusText = "Stopped";
                }
            }
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
            statusText = "Sweeping";
        }

        while (sweepRequested.load() && !shuttingDown.load()) {
            std::vector<float> cycleSpectrum(WIDE_BIN_COUNT, EMPTY_BIN_DB);
            std::vector<float> cycleQuality(WIDE_BIN_COUNT, -1.0f);
            bool capturedAnySegment = false;

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

                std::vector<float> averagedFFT;
                if (!captureAveragedFFT(settings.fftAveraging, averagedFFT)) {
                    if (!sweepRequested.load() || shuttingDown.load()) {
                        break;
                    }
                    setError("No FFT data is available from the active source");
                    return;
                }

                mergeSegment(settings, centerHz, averagedFFT, cycleSpectrum, cycleQuality);
                capturedAnySegment = true;
                {
                    std::lock_guard<std::mutex> lock(displayMutex);
                    currentSpectrum = cycleSpectrum;
                    displayedStartHz = settings.startHz;
                    displayedStopHz = settings.stopHz;
                    statusText = "Sweeping";
                }
            }

            if (!sweepRequested.load() || shuttingDown.load()) {
                break;
            }
            if (!capturedAnySegment) {
                setError("Sweep completed without FFT data");
                return;
            }

            publishSweep(settings, std::move(cycleSpectrum));
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
            return {(settings.startHz + settings.stopHz) * 0.5};
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

    bool captureAveragedFFT(int requestedFrames, std::vector<float>& averagedFFT) {
        std::vector<double> sums;
        int capturedFrames = 0;
        int attempts = 0;
        const int maxAttempts = requestedFrames + 5;

        while (capturedFrames < requestedFrames && attempts < maxAttempts) {
            if (attempts > 0 && !waitFor(std::chrono::milliseconds(FFT_FRAME_WAIT_MS))) {
                return false;
            }
            ++attempts;

            int fftSize = 0;
            float* fftData = gui::waterfall.acquireRawFFT(fftSize);
            if (!fftData || fftSize <= 0) {
                continue;
            }
            struct RawFFTRelease {
                ~RawFFTRelease() {
                    gui::waterfall.releaseRawFFT();
                }
            } rawFFTRelease;

            if (sums.empty()) {
                sums.assign(fftSize, 0.0);
            }
            if (static_cast<int>(sums.size()) != fftSize) {
                sums.clear();
                capturedFrames = 0;
                continue;
            }

            for (int i = 0; i < fftSize; ++i) {
                sums[i] += fftData[i];
            }
            ++capturedFrames;
        }

        if (capturedFrames == 0 || sums.empty()) {
            return false;
        }

        averagedFFT.resize(sums.size());
        for (std::size_t i = 0; i < sums.size(); ++i) {
            averagedFFT[i] = static_cast<float>(sums[i] / static_cast<double>(capturedFrames));
        }
        return true;
    }

    void mergeSegment(const SweepSettings& settings, double centerHz, const std::vector<float>& fft,
                      std::vector<float>& output, std::vector<float>& quality) const {
        if (fft.size() < 2 || output.empty() || output.size() != quality.size()) {
            return;
        }

        const double range = settings.stopHz - settings.startHz;
        const double usableBandwidth = settings.sampleRateHz * USABLE_BANDWIDTH_RATIO;
        const double usableHalf = usableBandwidth * 0.5;
        const double rawStartHz = centerHz - (settings.sampleRateHz * 0.5);
        const double usableStartHz = centerHz - usableHalf;
        const double usableStopHz = centerHz + usableHalf;

        for (std::size_t i = 0; i < output.size(); ++i) {
            const double frequencyHz = settings.startHz + ((static_cast<double>(i) + 0.5) * range / output.size());
            if (frequencyHz < usableStartHz || frequencyHz > usableStopHz) {
                continue;
            }

            const float segmentQuality = static_cast<float>(1.0 - (std::abs(frequencyHz - centerHz) / usableHalf));
            if (segmentQuality <= quality[i]) {
                continue;
            }

            const double rawPosition = ((frequencyHz - rawStartHz) / settings.sampleRateHz) * static_cast<double>(fft.size());
            const int lower = std::clamp(static_cast<int>(std::floor(rawPosition)), 0, static_cast<int>(fft.size()) - 1);
            const int upper = std::min(lower + 1, static_cast<int>(fft.size()) - 1);
            const float fraction = static_cast<float>(rawPosition - std::floor(rawPosition));
            const float level = fft[lower] + ((fft[upper] - fft[lower]) * fraction);
            if (!std::isfinite(level)) {
                continue;
            }

            output[i] = level;
            quality[i] = segmentQuality;
        }
    }

    void publishSweep(const SweepSettings& settings, std::vector<float> spectrum) {
        std::vector<DetectedPeak> peaks = detectPeaks(settings, spectrum);
        std::lock_guard<std::mutex> lock(displayMutex);
        currentSpectrum = std::move(spectrum);
        detectedPeaks = std::move(peaks);
        displayedStartHz = settings.startHz;
        displayedStopHz = settings.stopHz;
        activeSampleRateHz = settings.sampleRateHz;
        ++completedSweeps;
        statusText = "Sweeping";

        if (peakSpectrum.size() != currentSpectrum.size()) {
            peakSpectrum.assign(currentSpectrum.size(), EMPTY_BIN_DB);
        }
        if (settings.peakHold) {
            for (std::size_t i = 0; i < currentSpectrum.size(); ++i) {
                peakSpectrum[i] = std::max(peakSpectrum[i], currentSpectrum[i]);
            }
        }
        else {
            peakSpectrum = currentSpectrum;
        }
    }

    std::vector<DetectedPeak> detectPeaks(const SweepSettings& settings, const std::vector<float>& spectrum) const {
        std::vector<DetectedPeak> candidates;
        if (spectrum.size() < 3) {
            return candidates;
        }

        const double binWidthHz = (settings.stopHz - settings.startHz) / spectrum.size();
        for (std::size_t i = 1; i + 1 < spectrum.size(); ++i) {
            const float level = spectrum[i];
            if (level < settings.thresholdDbfs || level < spectrum[i - 1] || level <= spectrum[i + 1]) {
                continue;
            }
            candidates.push_back({settings.startHz + ((static_cast<double>(i) + 0.5) * binWidthHz), level});
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
        {
            std::lock_guard<std::mutex> lock(displayMutex);
            status = statusText;
            error = lastError;
            segment = currentSegment;
            totalSegments = segmentCount;
            sweeps = completedSweeps;
            centerHz = currentCenterHz;
            sampleRateHz = activeSampleRateHz;
        }

        ImGui::Separator();
        ImGui::Text("Status: %s", status.c_str());
        if (totalSegments > 0) {
            ImGui::Text("Segment: %d / %d | Sweeps: %d", segment, totalSegments, sweeps);
        }
        if (centerHz > 0.0) {
            ImGui::Text("Center: %.3f MHz | Rate: %.3f MS/s", centerHz / 1e6, sampleRateHz / 1e6);
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
            spectrum = currentSpectrum;
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

        auto drawTrace = [&](const std::vector<float>& data, ImU32 color, float thickness) {
            if (data.size() < 2) {
                return;
            }
            std::vector<ImVec2> points;
            points.reserve(data.size());
            for (std::size_t i = 0; i < data.size(); ++i) {
                const float xRatio = static_cast<float>(i) / static_cast<float>(data.size() - 1);
                points.emplace_back(plotMin.x + (xRatio * (plotMax.x - plotMin.x)), levelToY(data[i]));
            }
            drawList->AddPolyline(points.data(), static_cast<int>(points.size()), color, ImDrawFlags_None, thickness * scale);
        };

        drawTrace(spectrum, IM_COL32(40, 220, 150, 255), 1.5f);
        if (peakHoldEnabled) {
            drawTrace(heldPeaks, IM_COL32(255, 215, 65, 220), 1.0f);
        }
    }

    void drawPeakTable() {
        std::vector<DetectedPeak> peaks;
        {
            std::lock_guard<std::mutex> lock(displayMutex);
            peaks = detectedPeaks;
        }

        ImGui::TextUnformatted("Strongest Detected Peaks");
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
                requestStop("Stopping to tune selected peak...");
                pendingTuneHz = peaks[i].frequencyHz;
            }
            ImGui::TableSetColumnIndex(1);
            ImGui::Text("%.1f", peaks[i].levelDbfs);
        }
        ImGui::EndTable();
        ImGui::TextDisabled("Tap a row to stop the sweep and tune SDR++.");
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

    std::atomic<bool> shuttingDown{false};
    std::atomic<bool> sweepRequested{false};
    std::atomic<bool> workerActive{false};
    std::thread workerThread;
    std::mutex requestMutex;
    std::condition_variable requestCv;
    SweepSettings queuedSettings;

    std::mutex displayMutex;
    std::vector<float> currentSpectrum;
    std::vector<float> peakSpectrum;
    std::vector<DetectedPeak> detectedPeaks;
    std::string statusText = "Ready";
    std::string lastError;
    double displayedStartHz = 0.0;
    double displayedStopHz = 0.0;
    double activeSampleRateHz = 0.0;
    double currentCenterHz = 0.0;
    int currentSegment = 0;
    int segmentCount = 0;
    int completedSweeps = 0;

    double returnFrequencyHz = 0.0;
    double pendingTuneHz = 0.0;
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
