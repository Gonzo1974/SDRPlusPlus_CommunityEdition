#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <exception>
#include <functional>
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
    constexpr std::size_t V12_RAW_GRAPH_POINT_COUNT = 2048;
    constexpr std::size_t V12_FIXED_SAMPLE_COUNT = 15;
    constexpr std::size_t V12_PERIODICITY_COUNT = 8;
    constexpr std::size_t V12_WIDE_SAMPLE_COUNT = 16;
    constexpr std::size_t V13_REDUCER_COUNT = 6;
    constexpr std::size_t V13_TOP_PEAK_COUNT = 10;
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
        std::uint64_t pathCreationAttempts = 0;
        bool privateStreamAllocated = false;
        bool bindCompleted = false;
        bool sinkInitialized = false;
        bool sinkStarted = false;
        bool frontendPlaying = false;
        std::uintptr_t privateStreamAddress = 0;
        std::uintptr_t sinkInputAddress = 0;
        std::uintptr_t boundStreamAddress = 0;
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

    struct SplitterDiagnosticCheckpoint {
        bool captured = false;
        dsp::routing::SplitterDiagnosticsSnapshot snapshot;
    };

    struct RawFFTFixedSample {
        std::size_t bin = 0;
        double offsetHz = 0.0;
        float valueDbfs = NO_DATA_DBFS;
    };

    struct FFTPeriodicityDiagnostic {
        std::size_t periodBins = 0;
        double periodHz = 0.0;
        double score = std::numeric_limits<double>::quiet_NaN();
        double meanAbsoluteDifferenceDb = std::numeric_limits<double>::quiet_NaN();
        std::size_t comparedPairs = 0;
    };

    struct FFTFrameComparison {
        bool available = false;
        std::size_t comparedBins = 0;
        double meanAbsoluteDifferenceDb = std::numeric_limits<double>::quiet_NaN();
        double withinOneDbPercent = 0.0;
        double withinThreeDbPercent = 0.0;
    };

    struct WideBinContributionSample {
        std::size_t wideBinIndex = 0;
        double frequencyHz = 0.0;
        std::size_t contributingRawBins = 0;
        float selectedMaximumDbfs = NO_DATA_DBFS;
        float highestRawDbfs = NO_DATA_DBFS;
        float secondHighestRawDbfs = NO_DATA_DBFS;
    };

    struct SegmentContributionDiagnostics {
        double centerHz = 0.0;
        double rawStartHz = 0.0;
        double rawStopHz = 0.0;
        double usableStartHz = 0.0;
        double usableStopHz = 0.0;
        std::size_t rawBinsUsed = 0;
        std::size_t wideBinsHit = 0;
        std::size_t wideBinsWithMultipleRawBins = 0;
        std::size_t minimumRawBinsPerWideBin = 0;
        std::size_t maximumRawBinsPerWideBin = 0;
        double averageRawBinsPerWideBin = 0.0;
        std::size_t samplesWritten = 0;
        WideBinContributionSample samples[V12_WIDE_SAMPLE_COUNT];
    };

    enum V13ReducerIndex : std::size_t {
        V13_REDUCER_MAX = 0,
        V13_REDUCER_MEAN,
        V13_REDUCER_P95,
        V13_REDUCER_P98,
        V13_REDUCER_TOP4,
        V13_REDUCER_TOP8
    };

    struct WideBinReducerContributor {
        std::size_t contributingRawBins = 0;
        float highestRawDbfs = NO_DATA_DBFS;
        float secondHighestRawDbfs = NO_DATA_DBFS;
        float fourthHighestRawDbfs = NO_DATA_DBFS;
        float eighthHighestRawDbfs = NO_DATA_DBFS;
        float percentile95Dbfs = NO_DATA_DBFS;
        float percentile98Dbfs = NO_DATA_DBFS;
        float meanDbfs = NO_DATA_DBFS;
        float top4MeanDbfs = NO_DATA_DBFS;
        float top8MeanDbfs = NO_DATA_DBFS;
    };

    struct RobustReducerSweepState {
        std::vector<float> mean;
        std::vector<float> percentile95;
        std::vector<float> percentile98;
        std::vector<float> top4Mean;
        std::vector<float> top8Mean;
        std::vector<WideBinReducerContributor> contributors;
    };

    struct ReducerDifferenceStatistics {
        bool available = false;
        std::size_t comparedBins = 0;
        double averageDifferenceDb = std::numeric_limits<double>::quiet_NaN();
        double medianDifferenceDb = std::numeric_limits<double>::quiet_NaN();
        double percentile90DifferenceDb = std::numeric_limits<double>::quiet_NaN();
        double aboveThreeDbPercent = 0.0;
        double aboveSixDbPercent = 0.0;
        double aboveTenDbPercent = 0.0;
    };

    struct PeakPreservationStatistics {
        bool available = false;
        std::size_t measuredPeaks = 0;
        double averageLossDb = std::numeric_limits<double>::quiet_NaN();
        double medianLossDb = std::numeric_limits<double>::quiet_NaN();
        double worstLossDb = std::numeric_limits<double>::quiet_NaN();
        double averageLocalSnrDb = std::numeric_limits<double>::quiet_NaN();
        double withinOneDbPercent = 0.0;
        double withinThreeDbPercent = 0.0;
        double withinSixDbPercent = 0.0;
    };

    struct ReducerDiagnosticScore {
        bool available = false;
        double artifactSuppressionScore = 0.0;
        double peakPreservationScore = 0.0;
        double combinedScore = 0.0;
    };

    struct RobustPeakDiagnostic {
        std::size_t wideBinIndex = 0;
        double frequencyHz = 0.0;
        float maximumBaselineDbfs = NO_DATA_DBFS;
        std::array<float, V13_REDUCER_COUNT> levelsDbfs;
        std::array<double, V13_REDUCER_COUNT> lossesDb;
        std::array<double, V13_REDUCER_COUNT> localSnrDb;
        WideBinReducerContributor contributor;
    };

    struct RobustReducerDiagnostics {
        bool available = false;
        double startHz = 0.0;
        double stopHz = 0.0;
        std::size_t detectedMaximumPeaks = 0;
        std::vector<float> maximum;
        std::vector<float> mean;
        std::vector<float> percentile95;
        std::vector<float> percentile98;
        std::vector<float> top4Mean;
        std::vector<float> top8Mean;
        std::vector<WideBinReducerContributor> contributors;
        ReducerDifferenceStatistics difference[V13_REDUCER_COUNT];
        PeakPreservationStatistics peakPreservation[V13_REDUCER_COUNT];
        ReducerDiagnosticScore score[V13_REDUCER_COUNT];
        std::vector<RobustPeakDiagnostic> strongestPeaks;
    };

    struct SpectrumArtifactDiagnostics {
        bool available = false;
        int segmentNumber = 0;
        double centerHz = 0.0;
        double sampleRateHz = 0.0;
        std::size_t rawBinCount = 0;
        std::size_t validRawBinCount = 0;
        float rawMinimumDbfs = NO_DATA_DBFS;
        float rawMaximumDbfs = NO_DATA_DBFS;
        double rawMeanDbfs = std::numeric_limits<double>::quiet_NaN();
        double rawMedianDbfs = std::numeric_limits<double>::quiet_NaN();
        double rawStandardDeviationDb = std::numeric_limits<double>::quiet_NaN();
        double estimatedNoiseFloorDbfs = std::numeric_limits<double>::quiet_NaN();
        std::size_t localPeaksAboveSixDb = 0;
        std::size_t localPeaksAboveTenDb = 0;
        double averagePeakSpacingBins = std::numeric_limits<double>::quiet_NaN();
        std::size_t dominantPeakSpacingBins = 0;
        double dominantPeakSpacingHz = 0.0;
        std::size_t strongestPeriodBins = 0;
        double strongestPeriodHz = 0.0;
        double strongestPeriodScore = std::numeric_limits<double>::quiet_NaN();
        std::size_t fixedSamplesWritten = 0;
        RawFFTFixedSample fixedSamples[V12_FIXED_SAMPLE_COUNT];
        FFTPeriodicityDiagnostic periodicity[V12_PERIODICITY_COUNT];
        std::size_t consistencyFrameCount = 0;
        FFTFrameComparison frameOneToTwo;
        FFTFrameComparison frameTwoToThree;
        SegmentContributionDiagnostics contribution;
        std::size_t maxMeanComparableBins = 0;
        double averageMaxMinusMeanDb = std::numeric_limits<double>::quiet_NaN();
        double maxMinusMeanAboveThreeDbPercent = 0.0;
        double maxMinusMeanAboveSixDbPercent = 0.0;
        std::vector<float> rawFFTDisplay;
        std::vector<float> mergedWideMaximum;
        std::vector<float> mergedWideMean;
        RobustReducerDiagnostics robustReducers;
    };

    struct SegmentBoundaryDiagnostic {
        double usableStartHz = 0.0;
        double usableStopHz = 0.0;
        bool hasOverlap = false;
        double overlapStartHz = 0.0;
        double overlapStopHz = 0.0;
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

    const char* splitterDiagnosticPhaseText(dsp::routing::SplitterDiagnosticPhase phase) {
        switch (phase) {
        case dsp::routing::SplitterDiagnosticPhase::BEFORE_MEMCPY:
            return "BEFORE_MEMCPY";
        case dsp::routing::SplitterDiagnosticPhase::BEFORE_SWAP:
            return "BEFORE_SWAP";
        case dsp::routing::SplitterDiagnosticPhase::AFTER_SWAP:
            return "AFTER_SWAP";
        case dsp::routing::SplitterDiagnosticPhase::IDLE:
        default:
            return "IDLE";
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
        iqFrontendCurrentlyPlaying.store(gui::mainWindow.sdrIsRunning(), std::memory_order_relaxed);
        sigpath::iqFrontEnd.setSplitterDiagnosticsEnabled(true);

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
        sigpath::iqFrontEnd.setSplitterDiagnosticsEnabled(false);
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
        auto* instance = static_cast<WideSpectrumMonitorModule*>(ctx);
        instance->iqFrontendCurrentlyPlaying.store(playing, std::memory_order_relaxed);
        if (!playing) {
            instance->requestStop("Stopped (SDR source is not running)");
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
        diagnostics.pathCreationAttempts = iqCapturePathCreationAttempts.load(std::memory_order_relaxed);
        diagnostics.privateStreamAllocated = iqLastPathStreamAllocated.load(std::memory_order_relaxed);
        diagnostics.bindCompleted = iqLastPathBindCompleted.load(std::memory_order_relaxed);
        diagnostics.sinkInitialized = iqLastPathSinkInitialized.load(std::memory_order_relaxed);
        diagnostics.sinkStarted = iqLastPathSinkStarted.load(std::memory_order_relaxed);
        diagnostics.frontendPlaying = iqFrontendCurrentlyPlaying.load(std::memory_order_relaxed);
        diagnostics.privateStreamAddress = iqLastPrivateStreamAddress.load(std::memory_order_relaxed);
        diagnostics.sinkInputAddress = iqLastSinkInputAddress.load(std::memory_order_relaxed);
        diagnostics.boundStreamAddress = iqLastBoundStreamAddress.load(std::memory_order_relaxed);
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
            "Wide Spectrum Monitor: IQ capture failed: reason={}, path_attempts={}, allocated={}, bound={}, "
            "sink_initialized={}, sink_started={}, frontend_playing={}, handler_calls={}, handler_samples={}, "
            "request_calls={}, request_samples={}, discarded_chunks={}, copied={}/{}, last_chunk={}, generation={}",
            iqCaptureEndReasonText(diagnostics.endReason), diagnostics.pathCreationAttempts,
            diagnostics.privateStreamAllocated, diagnostics.bindCompleted,
            diagnostics.sinkInitialized, diagnostics.sinkStarted, diagnostics.frontendPlaying,
            diagnostics.handlerCalls, diagnostics.handlerSamples, diagnostics.requestedHandlerCalls,
            diagnostics.requestedHandlerSamples, diagnostics.discardedChunks,
            diagnostics.copiedSamples, diagnostics.targetSamples,
            diagnostics.lastHandlerChunkSize, diagnostics.generation);
    }

    void resetSplitterDiagnosticCheckpoints() {
        std::lock_guard<std::mutex> lock(splitterCheckpointMutex);
        beforePrivateBindCheckpoint = {};
        afterPrivateBindCheckpoint = {};
        beforeFirstTuneCheckpoint = {};
        immediatelyAfterFirstTuneCheckpoint = {};
        afterFirstTuneSettlingCheckpoint = {};
        captureTimeoutCheckpoint = {};
    }

    void captureSplitterDiagnosticCheckpoint(SplitterDiagnosticCheckpoint& checkpoint,
                                             bool onlyIfUnset = false) {
        const dsp::routing::SplitterDiagnosticsSnapshot snapshot =
            sigpath::iqFrontEnd.getSplitterDiagnosticsSnapshot();
        std::lock_guard<std::mutex> lock(splitterCheckpointMutex);
        if (onlyIfUnset && checkpoint.captured) {
            return;
        }
        checkpoint.captured = true;
        checkpoint.snapshot = snapshot;
    }

    bool startIQCapturePath() {
        stopIQCapturePath();
        resetSplitterDiagnosticCheckpoints();
        iqCapturePathCreationAttempts.fetch_add(1, std::memory_order_relaxed);
        iqLastPathStreamAllocated.store(false, std::memory_order_relaxed);
        iqLastPathBindCompleted.store(false, std::memory_order_relaxed);
        iqLastPathSinkInitialized.store(false, std::memory_order_relaxed);
        iqLastPathSinkStarted.store(false, std::memory_order_relaxed);
        iqLastPrivateStreamAddress.store(0, std::memory_order_relaxed);
        iqLastSinkInputAddress.store(0, std::memory_order_relaxed);
        iqLastBoundStreamAddress.store(0, std::memory_order_relaxed);
        iqFrontendCurrentlyPlaying.store(gui::mainWindow.sdrIsRunning(), std::memory_order_relaxed);
        captureSplitterDiagnosticCheckpoint(beforePrivateBindCheckpoint);

        try {
            iqStream = new dsp::stream<dsp::complex_t>();
            iqLastPrivateStreamAddress.store(reinterpret_cast<std::uintptr_t>(iqStream), std::memory_order_relaxed);
            iqLastPathStreamAllocated.store(true, std::memory_order_relaxed);

            iqLastBoundStreamAddress.store(reinterpret_cast<std::uintptr_t>(iqStream), std::memory_order_relaxed);
            sigpath::iqFrontEnd.bindIQStream(iqStream);
            iqStreamBound = true;
            iqLastPathBindCompleted.store(true, std::memory_order_relaxed);
            captureSplitterDiagnosticCheckpoint(afterPrivateBindCheckpoint);

            iqSink = new dsp::sink::Handler<dsp::complex_t>();
            iqLastSinkInputAddress.store(reinterpret_cast<std::uintptr_t>(iqStream), std::memory_order_relaxed);
            iqSink->init(iqStream, iqStreamHandler, this);
            iqSinkInitialized = true;
            iqLastPathSinkInitialized.store(true, std::memory_order_relaxed);

            iqSink->start();
            iqLastPathSinkStarted.store(true, std::memory_order_relaxed);
        }
        catch (const std::exception& e) {
            stopIQCapturePath();
            flog::error("Wide Spectrum Monitor: Could not start IQ capture path: {}", e.what());
            return false;
        }
        catch (...) {
            stopIQCapturePath();
            flog::error("Wide Spectrum Monitor: Could not start IQ capture path");
            return false;
        }
        return true;
    }

    void stopIQCapturePath() {
        if (iqSink && iqSinkInitialized) {
            iqSink->stop();
        }
        if (iqStreamBound) {
            sigpath::iqFrontEnd.unbindIQStream(iqStream);
            iqStreamBound = false;
        }
        delete iqSink;
        iqSink = nullptr;
        iqSinkInitialized = false;
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
        const bool frontendPlaying = gui::mainWindow.sdrIsRunning();
        iqFrontendCurrentlyPlaying.store(frontendPlaying, std::memory_order_relaxed);
        if (!frontendPlaying) {
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
            currentArtifactDiagnostics = {};
            activeSegmentBoundaries.clear();
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
            activeSegmentBoundaries = buildSegmentBoundaries(settings, centers);
            statusText = completedSweepValid ? "Sweeping" : "Waiting for first complete sweep";
        }

        while (sweepRequested.load() && !shuttingDown.load()) {
            // These buffers live for the entire sweep. Segments only merge into
            // them; no reducer trace is reset inside the segment loop.
            std::vector<float> cycleSpectrum(WIDE_BIN_COUNT, NO_DATA_DBFS);
            RobustReducerSweepState cycleReducers;
            cycleReducers.mean.assign(WIDE_BIN_COUNT, NO_DATA_DBFS);
            cycleReducers.percentile95.assign(WIDE_BIN_COUNT, NO_DATA_DBFS);
            cycleReducers.percentile98.assign(WIDE_BIN_COUNT, NO_DATA_DBFS);
            cycleReducers.top4Mean.assign(WIDE_BIN_COUNT, NO_DATA_DBFS);
            cycleReducers.top8Mean.assign(WIDE_BIN_COUNT, NO_DATA_DBFS);
            cycleReducers.contributors.resize(WIDE_BIN_COUNT);
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

                if (segmentIndex == 0) {
                    captureSplitterDiagnosticCheckpoint(beforeFirstTuneCheckpoint, true);
                }
                sigpath::sourceManager.tune(centerHz);
                if (segmentIndex == 0) {
                    captureSplitterDiagnosticCheckpoint(immediatelyAfterFirstTuneCheckpoint, true);
                }
                if (!waitFor(std::chrono::milliseconds(settings.tuningTimeMs))) {
                    break;
                }
                if (segmentIndex == 0) {
                    captureSplitterDiagnosticCheckpoint(afterFirstTuneSettlingCheckpoint, true);
                }

                SegmentDebugStats segmentDebugStats;
                segmentDebugStats.segmentNumber = static_cast<int>(segmentIndex) + 1;
                std::vector<float> averagedFFT;
                SpectrumArtifactDiagnostics artifactDiagnostics;
                if (!captureAveragedFFT(settings.fftAveraging, settings.sampleRateHz,
                                        centerHz, averagedFFT, cycleDebugStats,
                                        segmentDebugStats, artifactDiagnostics)) {
                    if (!sweepRequested.load() || shuttingDown.load()) {
                        break;
                    }
                    setError("No FFT data is available from the active source");
                    return;
                }

                segmentDebugStats.wideBinsWritten = mergeSegment(
                    settings, centerHz, averagedFFT, cycleSpectrum, cycleReducers,
                    cycleQuality, segmentDebugStats, artifactDiagnostics);
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
                    currentArtifactDiagnostics = std::move(artifactDiagnostics);
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

    std::vector<SegmentBoundaryDiagnostic> buildSegmentBoundaries(
        const SweepSettings& settings, const std::vector<double>& centers) const {
        std::vector<SegmentBoundaryDiagnostic> boundaries;
        boundaries.reserve(centers.size());
        const double usableHalf = settings.sampleRateHz * USABLE_BANDWIDTH_RATIO * 0.5;
        for (std::size_t segmentIndex = 0; segmentIndex < centers.size(); ++segmentIndex) {
            SegmentBoundaryDiagnostic boundary;
            boundary.usableStartHz = std::max(settings.startHz, centers[segmentIndex] - usableHalf);
            boundary.usableStopHz = std::min(settings.stopHz, centers[segmentIndex] + usableHalf);
            if (!boundaries.empty()) {
                boundary.overlapStartHz = std::max(
                    boundaries.back().usableStartHz, boundary.usableStartHz);
                boundary.overlapStopHz = std::min(
                    boundaries.back().usableStopHz, boundary.usableStopHz);
                boundary.hasOverlap = boundary.overlapStopHz > boundary.overlapStartHz;
            }
            boundaries.push_back(boundary);
        }
        return boundaries;
    }

    static FFTFrameComparison compareFFTFrames(const std::vector<float>& first,
                                               const std::vector<float>& second) {
        FFTFrameComparison comparison;
        if (first.size() != second.size() || first.empty()) {
            return comparison;
        }

        double absoluteDifferenceSum = 0.0;
        std::size_t withinOneDb = 0;
        std::size_t withinThreeDb = 0;
        for (std::size_t bin = 0; bin < first.size(); ++bin) {
            if (!isValidDbfs(first[bin]) || !isValidDbfs(second[bin])) {
                continue;
            }
            const double difference = std::abs(
                static_cast<double>(first[bin]) - static_cast<double>(second[bin]));
            absoluteDifferenceSum += difference;
            withinOneDb += difference <= 1.0;
            withinThreeDb += difference <= 3.0;
            ++comparison.comparedBins;
        }
        if (comparison.comparedBins == 0) {
            return comparison;
        }

        comparison.available = true;
        comparison.meanAbsoluteDifferenceDb =
            absoluteDifferenceSum / static_cast<double>(comparison.comparedBins);
        comparison.withinOneDbPercent =
            100.0 * static_cast<double>(withinOneDb) /
            static_cast<double>(comparison.comparedBins);
        comparison.withinThreeDbPercent =
            100.0 * static_cast<double>(withinThreeDb) /
            static_cast<double>(comparison.comparedBins);
        return comparison;
    }

    void computeSpectrumArtifactDiagnostics(
        const std::vector<float>& fft,
        const std::vector<std::vector<float>>& consistencyFrames,
        double sampleRateHz, double centerHz, int segmentNumber,
        SpectrumArtifactDiagnostics& diagnostics) const {
        diagnostics = {};
        diagnostics.segmentNumber = segmentNumber;
        diagnostics.centerHz = centerHz;
        diagnostics.sampleRateHz = sampleRateHz;
        diagnostics.rawBinCount = fft.size();
        if (fft.size() < 3 || !std::isfinite(sampleRateHz) || sampleRateHz <= 0.0) {
            return;
        }

        const double rawBinWidthHz = sampleRateHz / static_cast<double>(fft.size());
        std::vector<float> validValues;
        validValues.reserve(fft.size());
        double sum = 0.0;
        for (float value : fft) {
            if (!isValidDbfs(value)) {
                continue;
            }
            validValues.push_back(value);
            sum += value;
            includeInRange(value, diagnostics.rawMinimumDbfs, diagnostics.rawMaximumDbfs);
        }
        diagnostics.validRawBinCount = validValues.size();
        if (validValues.empty()) {
            return;
        }

        diagnostics.rawMeanDbfs = sum / static_cast<double>(validValues.size());
        double squaredDifferenceSum = 0.0;
        for (float value : validValues) {
            const double difference = static_cast<double>(value) - diagnostics.rawMeanDbfs;
            squaredDifferenceSum += difference * difference;
        }
        diagnostics.rawStandardDeviationDb = std::sqrt(
            squaredDifferenceSum / static_cast<double>(validValues.size()));

        std::sort(validValues.begin(), validValues.end());
        const std::size_t middle = validValues.size() / 2;
        diagnostics.rawMedianDbfs = (validValues.size() % 2) != 0
                                        ? validValues[middle]
                                        : (static_cast<double>(validValues[middle - 1]) +
                                           static_cast<double>(validValues[middle])) *
                                              0.5;
        const std::size_t noiseFloorIndex = static_cast<std::size_t>(
            std::floor(0.20 * static_cast<double>(validValues.size() - 1)));
        diagnostics.estimatedNoiseFloorDbfs = validValues[noiseFloorIndex];

        std::vector<std::size_t> sixDbPeakBins;
        for (std::size_t bin = 1; bin + 1 < fft.size(); ++bin) {
            if (!isValidDbfs(fft[bin - 1]) || !isValidDbfs(fft[bin]) ||
                !isValidDbfs(fft[bin + 1])) {
                continue;
            }
            if (!(fft[bin] > fft[bin - 1] && fft[bin] > fft[bin + 1])) {
                continue;
            }
            if (fft[bin] > diagnostics.estimatedNoiseFloorDbfs + 6.0) {
                ++diagnostics.localPeaksAboveSixDb;
                sixDbPeakBins.push_back(bin);
            }
            if (fft[bin] > diagnostics.estimatedNoiseFloorDbfs + 10.0) {
                ++diagnostics.localPeaksAboveTenDb;
            }
        }

        if (sixDbPeakBins.size() > 1) {
            std::vector<std::size_t> spacings;
            spacings.reserve(sixDbPeakBins.size() - 1);
            double spacingSum = 0.0;
            for (std::size_t peak = 1; peak < sixDbPeakBins.size(); ++peak) {
                const std::size_t spacing = sixDbPeakBins[peak] - sixDbPeakBins[peak - 1];
                spacings.push_back(spacing);
                spacingSum += static_cast<double>(spacing);
            }
            diagnostics.averagePeakSpacingBins =
                spacingSum / static_cast<double>(spacings.size());

            std::sort(spacings.begin(), spacings.end());
            std::size_t bestSpacing = spacings.front();
            std::size_t bestCount = 0;
            for (std::size_t begin = 0; begin < spacings.size();) {
                std::size_t end = begin + 1;
                while (end < spacings.size() && spacings[end] == spacings[begin]) {
                    ++end;
                }
                const std::size_t count = end - begin;
                if (count > bestCount) {
                    bestCount = count;
                    bestSpacing = spacings[begin];
                }
                begin = end;
            }
            diagnostics.dominantPeakSpacingBins = bestSpacing;
            diagnostics.dominantPeakSpacingHz =
                static_cast<double>(bestSpacing) * rawBinWidthHz;
        }

        const std::size_t fixedIndices[V12_FIXED_SAMPLE_COUNT] = {
            0, 1, 64, 128, 256, 512, 1024, 2048,
            4096, 8192, 16384, 32768, 49152, 65534, 65535
        };
        for (std::size_t sample = 0; sample < V12_FIXED_SAMPLE_COUNT; ++sample) {
            if (fixedIndices[sample] >= fft.size()) {
                continue;
            }
            RawFFTFixedSample& fixedSample =
                diagnostics.fixedSamples[diagnostics.fixedSamplesWritten++];
            fixedSample.bin = fixedIndices[sample];
            fixedSample.offsetHz =
                -sampleRateHz * 0.5 +
                ((static_cast<double>(fixedIndices[sample]) + 0.5) * rawBinWidthHz);
            fixedSample.valueDbfs = fft[fixedIndices[sample]];
        }

        const std::size_t candidatePeriods[V12_PERIODICITY_COUNT] = {
            8, 16, 32, 64, 128, 256, 512, 1024
        };
        for (std::size_t candidate = 0; candidate < V12_PERIODICITY_COUNT; ++candidate) {
            FFTPeriodicityDiagnostic& periodicity = diagnostics.periodicity[candidate];
            periodicity.periodBins = candidatePeriods[candidate];
            periodicity.periodHz =
                static_cast<double>(periodicity.periodBins) * rawBinWidthHz;
            double sumX = 0.0;
            double sumY = 0.0;
            double sumXX = 0.0;
            double sumYY = 0.0;
            double sumXY = 0.0;
            double absoluteDifferenceSum = 0.0;
            for (std::size_t bin = 0; bin + periodicity.periodBins < fft.size(); ++bin) {
                const float first = fft[bin];
                const float second = fft[bin + periodicity.periodBins];
                if (!isValidDbfs(first) || !isValidDbfs(second)) {
                    continue;
                }
                const double x = first;
                const double y = second;
                sumX += x;
                sumY += y;
                sumXX += x * x;
                sumYY += y * y;
                sumXY += x * y;
                absoluteDifferenceSum += std::abs(x - y);
                ++periodicity.comparedPairs;
            }
            if (periodicity.comparedPairs == 0) {
                continue;
            }
            periodicity.meanAbsoluteDifferenceDb =
                absoluteDifferenceSum / static_cast<double>(periodicity.comparedPairs);
            const double pairCount = static_cast<double>(periodicity.comparedPairs);
            const double covariance = (pairCount * sumXY) - (sumX * sumY);
            const double varianceX = (pairCount * sumXX) - (sumX * sumX);
            const double varianceY = (pairCount * sumYY) - (sumY * sumY);
            const double denominator = std::sqrt(std::max(0.0, varianceX * varianceY));
            if (denominator <= 0.0) {
                continue;
            }
            periodicity.score = covariance / denominator;
            if (!std::isfinite(diagnostics.strongestPeriodScore) ||
                periodicity.score > diagnostics.strongestPeriodScore) {
                diagnostics.strongestPeriodBins = periodicity.periodBins;
                diagnostics.strongestPeriodHz = periodicity.periodHz;
                diagnostics.strongestPeriodScore = periodicity.score;
            }
        }

        diagnostics.consistencyFrameCount = consistencyFrames.size();
        if (consistencyFrames.size() >= 2) {
            diagnostics.frameOneToTwo = compareFFTFrames(
                consistencyFrames[0], consistencyFrames[1]);
        }
        if (consistencyFrames.size() >= 3) {
            diagnostics.frameTwoToThree = compareFFTFrames(
                consistencyFrames[1], consistencyFrames[2]);
        }

        const std::size_t displayPointCount = std::min(
            V12_RAW_GRAPH_POINT_COUNT, fft.size());
        diagnostics.rawFFTDisplay.assign(displayPointCount, NO_DATA_DBFS);
        for (std::size_t displayPoint = 0; displayPoint < displayPointCount; ++displayPoint) {
            const std::size_t firstBin = (displayPoint * fft.size()) / displayPointCount;
            const std::size_t lastBin = std::max(
                firstBin + 1, ((displayPoint + 1) * fft.size()) / displayPointCount);
            float maximum = NO_DATA_DBFS;
            for (std::size_t bin = firstBin; bin < lastBin && bin < fft.size(); ++bin) {
                if (isValidDbfs(fft[bin]) &&
                    (!std::isfinite(maximum) || fft[bin] > maximum)) {
                    maximum = fft[bin];
                }
            }
            diagnostics.rawFFTDisplay[displayPoint] = maximum;
        }
        diagnostics.available = true;
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
            const bool captureTimedOut = iqLastCaptureEndReason == IQCaptureEndReason::TIMEOUT;
            iqLastCaptureGeneration = iqFrameGeneration;
            iqCaptureRequested = false;
            discardNextIQChunk = false;
            iqCaptureTarget = 0;
            iqCaptureBuffer.clear();
            lock.unlock();
            if (captureTimedOut) {
                captureSplitterDiagnosticCheckpoint(captureTimeoutCheckpoint);
            }
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

    bool captureAveragedFFT(int requestedFrames, double sampleRateHz, double centerHz,
                            std::vector<float>& averagedFFT,
                            SpectrumDebugStats& debugStats,
                            SegmentDebugStats& segmentDebugStats,
                            SpectrumArtifactDiagnostics& artifactDiagnostics) {
        const long long intervalSamples = std::llround(sampleRateHz / DIRECT_FFT_RATE);
        const std::size_t iqSamplesPerFrame = static_cast<std::size_t>(
            std::clamp<long long>(intervalSamples, 2, DIRECT_FFT_SIZE));
        segmentDebugStats.iqSamplesPerFrame = iqSamplesPerFrame;

        std::vector<double> sums(DIRECT_FFT_SIZE, 0.0);
        std::vector<unsigned int> validCounts(DIRECT_FFT_SIZE, 0);
        std::vector<unsigned char> sawZero(DIRECT_FFT_SIZE, 0);
        std::vector<unsigned char> sawNonFinite(DIRECT_FFT_SIZE, 0);
        std::vector<unsigned char> sawOutOfRange(DIRECT_FFT_SIZE, 0);
        std::vector<std::vector<float>> consistencyFrames;
        consistencyFrames.reserve(3);
        std::vector<float> lastRawFFT;

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
            if (consistencyFrames.size() < 3) {
                consistencyFrames.push_back(fftData);
            }
            lastRawFFT = fftData;

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
        computeSpectrumArtifactDiagnostics(
            lastRawFFT, consistencyFrames, sampleRateHz, centerHz,
            segmentDebugStats.segmentNumber, artifactDiagnostics);
        return true;
    }

    static const char* robustReducerName(std::size_t reducer) {
        static const char* names[V13_REDUCER_COUNT] = {
            "MAX", "MEAN", "P95", "P98", "TOP4_MEAN", "TOP8_MEAN"
        };
        return reducer < V13_REDUCER_COUNT ? names[reducer] : "unknown";
    }

    static const std::vector<float>* robustReducerTrace(
        const RobustReducerDiagnostics& diagnostics, std::size_t reducer) {
        switch (reducer) {
        case V13_REDUCER_MAX:
            return &diagnostics.maximum;
        case V13_REDUCER_MEAN:
            return &diagnostics.mean;
        case V13_REDUCER_P95:
            return &diagnostics.percentile95;
        case V13_REDUCER_P98:
            return &diagnostics.percentile98;
        case V13_REDUCER_TOP4:
            return &diagnostics.top4Mean;
        case V13_REDUCER_TOP8:
            return &diagnostics.top8Mean;
        default:
            return nullptr;
        }
    }

    static double sortedMedian(const std::vector<double>& sortedValues) {
        if (sortedValues.empty()) {
            return std::numeric_limits<double>::quiet_NaN();
        }
        const std::size_t middle = sortedValues.size() / 2;
        return (sortedValues.size() % 2) != 0
                   ? sortedValues[middle]
                   : (sortedValues[middle - 1] + sortedValues[middle]) * 0.5;
    }

    static double sortedPercentile(const std::vector<double>& sortedValues,
                                   double percentile) {
        if (sortedValues.empty()) {
            return std::numeric_limits<double>::quiet_NaN();
        }
        const std::size_t index = std::min(
            sortedValues.size() - 1,
            static_cast<std::size_t>(std::ceil(
                percentile * static_cast<double>(sortedValues.size()))) -
                1);
        return sortedValues[index];
    }

    static bool localMedianExcludingCenter(const std::vector<float>& trace,
                                           std::size_t center,
                                           float& median) {
        constexpr std::size_t radius = 5;
        if (trace.empty() || center >= trace.size()) {
            return false;
        }
        const std::size_t first = center > radius ? center - radius : 0;
        const std::size_t last = std::min(trace.size() - 1, center + radius);
        std::array<float, radius * 2> neighbors;
        std::size_t count = 0;
        for (std::size_t index = first; index <= last; ++index) {
            if (index != center && isValidDbfs(trace[index])) {
                neighbors[count++] = trace[index];
            }
        }
        if (count == 0) {
            return false;
        }
        std::sort(neighbors.begin(), neighbors.begin() + count);
        const std::size_t middle = count / 2;
        median = (count % 2) != 0
                     ? neighbors[middle]
                     : (neighbors[middle - 1] + neighbors[middle]) * 0.5f;
        return true;
    }

    static void computeRobustReducerDiagnostics(
        const SweepSettings& settings, const std::vector<float>& maximum,
        const RobustReducerSweepState& reducers,
        RobustReducerDiagnostics& diagnostics) {
        diagnostics = {};
        if (maximum.empty() || reducers.mean.size() != maximum.size() ||
            reducers.percentile95.size() != maximum.size() ||
            reducers.percentile98.size() != maximum.size() ||
            reducers.top4Mean.size() != maximum.size() ||
            reducers.top8Mean.size() != maximum.size() ||
            reducers.contributors.size() != maximum.size()) {
            return;
        }

        diagnostics.startHz = settings.startHz;
        diagnostics.stopHz = settings.stopHz;
        diagnostics.maximum = maximum;
        diagnostics.mean = reducers.mean;
        diagnostics.percentile95 = reducers.percentile95;
        diagnostics.percentile98 = reducers.percentile98;
        diagnostics.top4Mean = reducers.top4Mean;
        diagnostics.top8Mean = reducers.top8Mean;
        diagnostics.contributors = reducers.contributors;

        for (std::size_t reducer = 0; reducer < V13_REDUCER_COUNT; ++reducer) {
            const std::vector<float>* trace = robustReducerTrace(diagnostics, reducer);
            if (!trace) {
                continue;
            }
            std::vector<double> differences;
            differences.reserve(maximum.size());
            double sum = 0.0;
            std::size_t aboveThree = 0;
            std::size_t aboveSix = 0;
            std::size_t aboveTen = 0;
            for (std::size_t index = 0; index < maximum.size(); ++index) {
                if (!isValidDbfs(maximum[index]) || !isValidDbfs((*trace)[index])) {
                    continue;
                }
                const double difference = static_cast<double>(maximum[index]) -
                                          static_cast<double>((*trace)[index]);
                differences.push_back(difference);
                sum += difference;
                aboveThree += difference > 3.0;
                aboveSix += difference > 6.0;
                aboveTen += difference > 10.0;
            }
            if (differences.empty()) {
                continue;
            }
            std::sort(differences.begin(), differences.end());
            ReducerDifferenceStatistics& statistics = diagnostics.difference[reducer];
            statistics.available = true;
            statistics.comparedBins = differences.size();
            statistics.averageDifferenceDb = sum / static_cast<double>(differences.size());
            statistics.medianDifferenceDb = sortedMedian(differences);
            statistics.percentile90DifferenceDb = sortedPercentile(differences, 0.90);
            statistics.aboveThreeDbPercent =
                100.0 * static_cast<double>(aboveThree) /
                static_cast<double>(differences.size());
            statistics.aboveSixDbPercent =
                100.0 * static_cast<double>(aboveSix) /
                static_cast<double>(differences.size());
            statistics.aboveTenDbPercent =
                100.0 * static_cast<double>(aboveTen) /
                static_cast<double>(differences.size());
        }

        struct PeakCandidate {
            std::size_t index = 0;
            float baseline = NO_DATA_DBFS;
        };
        std::vector<PeakCandidate> peakCandidates;
        for (std::size_t index = 1; index + 1 < maximum.size(); ++index) {
            if (!isValidDbfs(maximum[index - 1]) || !isValidDbfs(maximum[index]) ||
                !isValidDbfs(maximum[index + 1]) ||
                !(maximum[index] > maximum[index - 1] &&
                  maximum[index] > maximum[index + 1])) {
                continue;
            }
            float baseline = NO_DATA_DBFS;
            if (localMedianExcludingCenter(maximum, index, baseline) &&
                maximum[index] >= baseline + 6.0f) {
                peakCandidates.push_back({ index, baseline });
            }
        }
        diagnostics.detectedMaximumPeaks = peakCandidates.size();

        std::array<std::vector<double>, V13_REDUCER_COUNT> losses;
        std::array<std::vector<double>, V13_REDUCER_COUNT> localSnrs;
        for (const PeakCandidate& peak : peakCandidates) {
            for (std::size_t reducer = 0; reducer < V13_REDUCER_COUNT; ++reducer) {
                const std::vector<float>* trace = robustReducerTrace(diagnostics, reducer);
                if (!trace || !isValidDbfs((*trace)[peak.index])) {
                    continue;
                }
                float baseline = NO_DATA_DBFS;
                if (!localMedianExcludingCenter(*trace, peak.index, baseline)) {
                    continue;
                }
                losses[reducer].push_back(
                    static_cast<double>(maximum[peak.index]) - (*trace)[peak.index]);
                localSnrs[reducer].push_back(
                    static_cast<double>((*trace)[peak.index]) - baseline);
            }
        }

        for (std::size_t reducer = 0; reducer < V13_REDUCER_COUNT; ++reducer) {
            if (losses[reducer].empty()) {
                continue;
            }
            double lossSum = 0.0;
            double snrSum = 0.0;
            std::size_t withinOne = 0;
            std::size_t withinThree = 0;
            std::size_t withinSix = 0;
            for (std::size_t peak = 0; peak < losses[reducer].size(); ++peak) {
                const double loss = losses[reducer][peak];
                lossSum += loss;
                snrSum += localSnrs[reducer][peak];
                withinOne += loss <= 1.0;
                withinThree += loss <= 3.0;
                withinSix += loss <= 6.0;
            }
            std::sort(losses[reducer].begin(), losses[reducer].end());
            PeakPreservationStatistics& statistics =
                diagnostics.peakPreservation[reducer];
            statistics.available = true;
            statistics.measuredPeaks = losses[reducer].size();
            statistics.averageLossDb =
                lossSum / static_cast<double>(losses[reducer].size());
            statistics.medianLossDb = sortedMedian(losses[reducer]);
            statistics.worstLossDb = losses[reducer].back();
            statistics.averageLocalSnrDb =
                snrSum / static_cast<double>(localSnrs[reducer].size());
            statistics.withinOneDbPercent =
                100.0 * static_cast<double>(withinOne) /
                static_cast<double>(losses[reducer].size());
            statistics.withinThreeDbPercent =
                100.0 * static_cast<double>(withinThree) /
                static_cast<double>(losses[reducer].size());
            statistics.withinSixDbPercent =
                100.0 * static_cast<double>(withinSix) /
                static_cast<double>(losses[reducer].size());

            if (diagnostics.difference[reducer].available) {
                ReducerDiagnosticScore& score = diagnostics.score[reducer];
                score.available = true;
                // Diagnostic-only score: 10 dB of average MAX suppression and
                // zero average peak loss each map to 100. The combined score is
                // their equal-weight mean; it never selects a production reducer.
                score.artifactSuppressionScore = std::clamp(
                    diagnostics.difference[reducer].averageDifferenceDb * 10.0,
                    0.0, 100.0);
                score.peakPreservationScore = std::clamp(
                    100.0 - (statistics.averageLossDb * (100.0 / 6.0)),
                    0.0, 100.0);
                score.combinedScore =
                    (score.artifactSuppressionScore + score.peakPreservationScore) * 0.5;
            }
        }

        std::sort(peakCandidates.begin(), peakCandidates.end(),
                  [&maximum](const PeakCandidate& first,
                             const PeakCandidate& second) {
                      return maximum[first.index] > maximum[second.index];
                  });
        const double wideBinWidthHz =
            (settings.stopHz - settings.startHz) /
            static_cast<double>(maximum.size());
        const std::size_t reportedPeaks = std::min(
            V13_TOP_PEAK_COUNT, peakCandidates.size());
        diagnostics.strongestPeaks.reserve(reportedPeaks);
        for (std::size_t peakIndex = 0; peakIndex < reportedPeaks; ++peakIndex) {
            const PeakCandidate& candidate = peakCandidates[peakIndex];
            RobustPeakDiagnostic peak;
            peak.levelsDbfs.fill(NO_DATA_DBFS);
            peak.lossesDb.fill(std::numeric_limits<double>::quiet_NaN());
            peak.localSnrDb.fill(std::numeric_limits<double>::quiet_NaN());
            peak.wideBinIndex = candidate.index;
            peak.frequencyHz = settings.startHz +
                               ((static_cast<double>(candidate.index) + 0.5) *
                                wideBinWidthHz);
            peak.maximumBaselineDbfs = candidate.baseline;
            peak.contributor = reducers.contributors[candidate.index];
            for (std::size_t reducer = 0; reducer < V13_REDUCER_COUNT; ++reducer) {
                const std::vector<float>* trace = robustReducerTrace(diagnostics, reducer);
                if (!trace || !isValidDbfs((*trace)[candidate.index])) {
                    continue;
                }
                peak.levelsDbfs[reducer] = (*trace)[candidate.index];
                peak.lossesDb[reducer] =
                    static_cast<double>(maximum[candidate.index]) -
                    (*trace)[candidate.index];
                float baseline = NO_DATA_DBFS;
                if (localMedianExcludingCenter(*trace, candidate.index, baseline)) {
                    peak.localSnrDb[reducer] =
                        static_cast<double>((*trace)[candidate.index]) - baseline;
                }
            }
            diagnostics.strongestPeaks.push_back(peak);
        }
        diagnostics.available = true;
    }

    std::size_t mergeSegment(const SweepSettings& settings, double centerHz,
                             const std::vector<float>& fft,
                             std::vector<float>& output,
                             RobustReducerSweepState& diagnosticReducers,
                             std::vector<float>& quality,
                             SegmentDebugStats& segmentDebugStats,
                             SpectrumArtifactDiagnostics& artifactDiagnostics) const {
        if (fft.size() < 2 || output.empty() || output.size() != quality.size() ||
            diagnosticReducers.mean.size() != output.size() ||
            diagnosticReducers.percentile95.size() != output.size() ||
            diagnosticReducers.percentile98.size() != output.size() ||
            diagnosticReducers.top4Mean.size() != output.size() ||
            diagnosticReducers.top8Mean.size() != output.size() ||
            diagnosticReducers.contributors.size() != output.size()) {
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

        SegmentContributionDiagnostics contribution;
        contribution.centerHz = centerHz;
        contribution.rawStartHz = rawStartHz;
        contribution.rawStopHz = rawStopHz;
        contribution.usableStartHz = effectiveUsableStartHz;
        contribution.usableStopHz = effectiveUsableStopHz;

        // Project every valid raw FFT bin exactly once. A wide bin is much wider
        // than a raw bin, so every raw bin landing in it participates in the max
        // reduction instead of relying on rounded, output-driven index ranges.
        std::vector<float> segmentSpectrum(output.size(), NO_DATA_DBFS);
        std::vector<double> segmentSums(output.size(), 0.0);
        std::vector<std::size_t> segmentCounts(output.size(), 0);
        std::vector<float> segmentHighest(output.size(), NO_DATA_DBFS);
        std::vector<float> segmentSecondHighest(output.size(), NO_DATA_DBFS);
        std::vector<std::vector<float>> segmentValues(output.size());
        const std::size_t expectedRawBinsPerWideBin = std::max<std::size_t>(
            8, static_cast<std::size_t>(std::ceil(wideBinWidthHz / rawBinWidthHz)) + 2);
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
            ++contribution.rawBinsUsed;
            segmentSums[wideIndex] += candidate;
            ++segmentCounts[wideIndex];
            std::vector<float>& values = segmentValues[wideIndex];
            if (values.empty()) {
                // Reserve once per touched wide bin. Subsequent raw-bin inserts
                // remain allocation-free at the expected FFT/wide-bin ratio.
                values.reserve(expectedRawBinsPerWideBin);
            }
            values.push_back(candidate);
            if (!std::isfinite(segmentHighest[wideIndex]) ||
                candidate > segmentHighest[wideIndex]) {
                segmentSecondHighest[wideIndex] = segmentHighest[wideIndex];
                segmentHighest[wideIndex] = candidate;
            }
            else if (!std::isfinite(segmentSecondHighest[wideIndex]) ||
                     candidate > segmentSecondHighest[wideIndex]) {
                segmentSecondHighest[wideIndex] = candidate;
            }
            float& segmentLevel = segmentSpectrum[wideIndex];
            if (!std::isfinite(segmentLevel) || candidate > segmentLevel) {
                segmentLevel = candidate;
            }
        }

        std::vector<std::size_t> hitWideBins;
        hitWideBins.reserve(output.size());
        std::size_t rawBinsAcrossHitWideBins = 0;
        contribution.minimumRawBinsPerWideBin = std::numeric_limits<std::size_t>::max();
        for (std::size_t wideIndex = 0; wideIndex < output.size(); ++wideIndex) {
            const std::size_t contributingBins = segmentCounts[wideIndex];
            if (contributingBins == 0) {
                continue;
            }
            hitWideBins.push_back(wideIndex);
            rawBinsAcrossHitWideBins += contributingBins;
            ++contribution.wideBinsHit;
            contribution.wideBinsWithMultipleRawBins += contributingBins > 1;
            contribution.minimumRawBinsPerWideBin = std::min(
                contribution.minimumRawBinsPerWideBin, contributingBins);
            contribution.maximumRawBinsPerWideBin = std::max(
                contribution.maximumRawBinsPerWideBin, contributingBins);
        }
        if (contribution.wideBinsHit > 0) {
            contribution.averageRawBinsPerWideBin =
                static_cast<double>(rawBinsAcrossHitWideBins) /
                static_cast<double>(contribution.wideBinsHit);
        }
        else {
            contribution.minimumRawBinsPerWideBin = 0;
        }

        std::vector<WideBinReducerContributor> segmentContributors(output.size());
        for (std::size_t wideIndex : hitWideBins) {
            std::vector<float>& values = segmentValues[wideIndex];
            const std::size_t count = values.size();
            if (count == 0) {
                continue;
            }

            WideBinReducerContributor& reducer = segmentContributors[wideIndex];
            reducer.contributingRawBins = count;
            reducer.meanDbfs = static_cast<float>(
                segmentSums[wideIndex] / static_cast<double>(count));

            const std::size_t percentile95Index = std::min(
                count - 1,
                static_cast<std::size_t>(std::ceil(0.95 * static_cast<double>(count))) - 1);
            std::nth_element(values.begin(), values.begin() + percentile95Index,
                             values.end());
            reducer.percentile95Dbfs = values[percentile95Index];
            const std::size_t percentile98Index = std::min(
                count - 1,
                static_cast<std::size_t>(std::ceil(0.98 * static_cast<double>(count))) - 1);
            std::nth_element(values.begin(), values.begin() + percentile98Index,
                             values.end());
            reducer.percentile98Dbfs = values[percentile98Index];

            const std::size_t topCount = std::min<std::size_t>(8, count);
            std::partial_sort(values.begin(), values.begin() + topCount, values.end(),
                              std::greater<float>());
            reducer.highestRawDbfs = values[0];
            reducer.secondHighestRawDbfs = count >= 2 ? values[1] : NO_DATA_DBFS;
            reducer.fourthHighestRawDbfs = count >= 4 ? values[3] : NO_DATA_DBFS;
            reducer.eighthHighestRawDbfs = count >= 8 ? values[7] : NO_DATA_DBFS;

            double top4Sum = 0.0;
            const std::size_t top4Count = std::min<std::size_t>(4, count);
            for (std::size_t index = 0; index < top4Count; ++index) {
                top4Sum += values[index];
            }
            double top8Sum = top4Sum;
            for (std::size_t index = top4Count; index < topCount; ++index) {
                top8Sum += values[index];
            }
            reducer.top4MeanDbfs = static_cast<float>(
                top4Sum / static_cast<double>(top4Count));
            reducer.top8MeanDbfs = static_cast<float>(
                top8Sum / static_cast<double>(topCount));
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
            diagnosticReducers.mean[wideIndex] =
                segmentContributors[wideIndex].meanDbfs;
            diagnosticReducers.percentile95[wideIndex] =
                segmentContributors[wideIndex].percentile95Dbfs;
            diagnosticReducers.percentile98[wideIndex] =
                segmentContributors[wideIndex].percentile98Dbfs;
            diagnosticReducers.top4Mean[wideIndex] =
                segmentContributors[wideIndex].top4MeanDbfs;
            diagnosticReducers.top8Mean[wideIndex] =
                segmentContributors[wideIndex].top8MeanDbfs;
            diagnosticReducers.contributors[wideIndex] =
                segmentContributors[wideIndex];
            quality[wideIndex] = segmentQuality;
            ++writtenBins;
        }

        const std::size_t requestedSamples = std::min(
            V12_WIDE_SAMPLE_COUNT, hitWideBins.size());
        for (std::size_t sample = 0; sample < requestedSamples; ++sample) {
            const std::size_t hitPosition = requestedSamples == 1
                                                ? 0
                                                : (sample * (hitWideBins.size() - 1)) /
                                                      (requestedSamples - 1);
            const std::size_t wideIndex = hitWideBins[hitPosition];
            WideBinContributionSample& contributionSample =
                contribution.samples[contribution.samplesWritten++];
            contributionSample.wideBinIndex = wideIndex;
            contributionSample.frequencyHz = settings.startHz +
                                             ((static_cast<double>(wideIndex) + 0.5) *
                                              wideBinWidthHz);
            contributionSample.contributingRawBins = segmentCounts[wideIndex];
            contributionSample.selectedMaximumDbfs = segmentSpectrum[wideIndex];
            contributionSample.highestRawDbfs = segmentHighest[wideIndex];
            contributionSample.secondHighestRawDbfs = segmentSecondHighest[wideIndex];
        }

        artifactDiagnostics.contribution = contribution;
        artifactDiagnostics.mergedWideMaximum = output;
        artifactDiagnostics.mergedWideMean = diagnosticReducers.mean;
        double maxMinusMeanSum = 0.0;
        std::size_t aboveThreeDb = 0;
        std::size_t aboveSixDb = 0;
        for (std::size_t wideIndex = 0;
             wideIndex < output.size() && wideIndex < diagnosticReducers.mean.size();
             ++wideIndex) {
            if (!isValidDbfs(output[wideIndex]) ||
                !isValidDbfs(diagnosticReducers.mean[wideIndex])) {
                continue;
            }
            const double difference = static_cast<double>(output[wideIndex]) -
                                      static_cast<double>(diagnosticReducers.mean[wideIndex]);
            maxMinusMeanSum += difference;
            aboveThreeDb += difference > 3.0;
            aboveSixDb += difference > 6.0;
            ++artifactDiagnostics.maxMeanComparableBins;
        }
        if (artifactDiagnostics.maxMeanComparableBins > 0) {
            artifactDiagnostics.averageMaxMinusMeanDb =
                maxMinusMeanSum /
                static_cast<double>(artifactDiagnostics.maxMeanComparableBins);
            artifactDiagnostics.maxMinusMeanAboveThreeDbPercent =
                100.0 * static_cast<double>(aboveThreeDb) /
                static_cast<double>(artifactDiagnostics.maxMeanComparableBins);
            artifactDiagnostics.maxMinusMeanAboveSixDbPercent =
                100.0 * static_cast<double>(aboveSixDb) /
                static_cast<double>(artifactDiagnostics.maxMeanComparableBins);
        }
        computeRobustReducerDiagnostics(
            settings, output, diagnosticReducers,
            artifactDiagnostics.robustReducers);
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
        std::vector<SegmentBoundaryDiagnostic> segmentBoundaries;
        double startHz = startFrequencyMHz * 1e6;
        double stopHz = stopFrequencyMHz * 1e6;
        {
            std::lock_guard<std::mutex> lock(displayMutex);
            spectrum = completedSweep;
            heldPeaks = peakSpectrum;
            segmentBoundaries = activeSegmentBoundaries;
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

        auto frequencyToX = [&](double frequencyHz) {
            const double ratio = (frequencyHz - startHz) / (stopHz - startHz);
            return plotMin.x +
                   (static_cast<float>(std::clamp(ratio, 0.0, 1.0)) *
                    (plotMax.x - plotMin.x));
        };
        auto drawDashedVertical = [&](float x, ImU32 color) {
            const float dashLength = 4.0f * scale;
            const float gapLength = 3.0f * scale;
            for (float y = plotMin.y; y < plotMax.y; y += dashLength + gapLength) {
                drawList->AddLine(
                    ImVec2(x, y), ImVec2(x, std::min(plotMax.y, y + dashLength)),
                    color, 1.0f * scale);
            }
        };
        for (const SegmentBoundaryDiagnostic& boundary : segmentBoundaries) {
            if (boundary.usableStopHz <= startHz || boundary.usableStartHz >= stopHz) {
                continue;
            }
            drawList->AddLine(
                ImVec2(frequencyToX(boundary.usableStartHz), plotMin.y),
                ImVec2(frequencyToX(boundary.usableStartHz), plotMax.y),
                IM_COL32(175, 105, 255, 130), 1.0f * scale);
            drawList->AddLine(
                ImVec2(frequencyToX(boundary.usableStopHz), plotMin.y),
                ImVec2(frequencyToX(boundary.usableStopHz), plotMax.y),
                IM_COL32(175, 105, 255, 130), 1.0f * scale);
            if (boundary.hasOverlap) {
                drawDashedVertical(frequencyToX(boundary.overlapStartHz),
                                   IM_COL32(255, 135, 55, 210));
                drawDashedVertical(frequencyToX(boundary.overlapStopHz),
                                   IM_COL32(255, 135, 55, 210));
            }
        }
        if (!segmentBoundaries.empty()) {
            drawList->AddText(
                ImVec2(plotMin.x + (4.0f * scale), plotMax.y - (16.0f * scale)),
                IM_COL32(205, 185, 225, 220), "solid: usable | dashed: overlap");
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

    static const char* splitterOutputRole(std::uintptr_t streamAddress,
                                          std::uintptr_t fftInputAddress,
                                          std::uintptr_t privateStreamAddress) {
        if (streamAddress != 0 && streamAddress == fftInputAddress) {
            return "fftIn";
        }
        if (streamAddress != 0 && streamAddress == privateStreamAddress) {
            return "WSM private";
        }
        return "VFO/other";
    }

    static void drawSplitterDiagnosticCheckpoint(
        const char* label, const SplitterDiagnosticCheckpoint& checkpoint,
        std::uintptr_t fftInputAddress, std::uintptr_t privateStreamAddress) {
        if (!ImGui::TreeNodeEx(label, ImGuiTreeNodeFlags_DefaultOpen)) {
            return;
        }
        if (!checkpoint.captured) {
            ImGui::TextUnformatted("Snapshot not captured");
            ImGui::TreePop();
            return;
        }

        const dsp::routing::SplitterDiagnosticsSnapshot& snapshot = checkpoint.snapshot;
        ImGui::Text("Diagnostics enabled: %s", snapshot.enabled ? "yes" : "no");
        ImGui::Text("Splitter runs / input samples: %llu / %llu",
                    static_cast<unsigned long long>(snapshot.runCalls),
                    static_cast<unsigned long long>(snapshot.inputSamples));
        ImGui::Text("Registered / tracked outputs: %zu / %zu", snapshot.outputCount,
                    snapshot.trackedOutputCount);
        ImGui::Text("Completed runs / last completed run: %llu / %llu",
                    static_cast<unsigned long long>(snapshot.completedRuns),
                    static_cast<unsigned long long>(snapshot.lastCompletedRun));
        ImGui::Text("Current output: %d @ 0x%llX (%s)", snapshot.currentOutputIndex,
                    static_cast<unsigned long long>(snapshot.currentOutputAddress),
                    splitterDiagnosticPhaseText(snapshot.phase));
        ImGui::Text("Last incomplete run/output: %llu / %d @ 0x%llX",
                    static_cast<unsigned long long>(snapshot.lastIncompleteRun),
                    snapshot.lastIncompleteOutputIndex,
                    static_cast<unsigned long long>(snapshot.lastIncompleteOutputAddress));

        const std::size_t trackedCount = std::min(
            snapshot.trackedOutputCount, dsp::routing::SPLITTER_DIAGNOSTIC_MAX_OUTPUTS);
        for (std::size_t outputIndex = 0; outputIndex < trackedCount; ++outputIndex) {
            const dsp::routing::SplitterOutputDiagnosticsSnapshot& output =
                snapshot.outputs[outputIndex];
            ImGui::TextWrapped(
                "Output %zu [%s] 0x%llX: attempts=%llu, swaps=%llu, samples=%llu",
                outputIndex,
                splitterOutputRole(output.streamAddress, fftInputAddress, privateStreamAddress),
                static_cast<unsigned long long>(output.streamAddress),
                static_cast<unsigned long long>(output.writeAttempts),
                static_cast<unsigned long long>(output.successfulSwaps),
                static_cast<unsigned long long>(output.samplesOffered));
        }
        ImGui::TreePop();
    }

    void drawV12SpectrumGraph(const char* title, const char* id,
                              const std::vector<float>& primary,
                              const std::vector<float>* secondary = nullptr,
                              const char* secondaryLabel = "wideMean") {
        ImGui::TextUnformatted(title);
        const float scale = style::uiScale;
        const ImVec2 canvasPosition = ImGui::GetCursorScreenPos();
        const ImVec2 canvasSize(
            std::max(120.0f, ImGui::GetContentRegionAvail().x), 130.0f * scale);
        ImGui::InvisibleButton(
            ("##wsm_v12_" + std::string(id) + "_" + name).c_str(), canvasSize);

        ImDrawList* drawList = ImGui::GetWindowDrawList();
        const ImVec2 canvasEnd(
            canvasPosition.x + canvasSize.x, canvasPosition.y + canvasSize.y);
        drawList->AddRectFilled(canvasPosition, canvasEnd, IM_COL32(12, 16, 22, 255));
        drawList->AddRect(canvasPosition, canvasEnd, IM_COL32(90, 100, 115, 255));
        const ImVec2 plotMin(
            canvasPosition.x + (42.0f * scale), canvasPosition.y + (7.0f * scale));
        const ImVec2 plotMax(
            canvasEnd.x - (7.0f * scale), canvasEnd.y - (7.0f * scale));
        if (plotMax.x <= plotMin.x || plotMax.y <= plotMin.y) {
            return;
        }

        auto levelToY = [&](float level) {
            const float clamped = std::clamp(level, GRAPH_MIN_DB, GRAPH_MAX_DB);
            const float ratio =
                (clamped - GRAPH_MIN_DB) / (GRAPH_MAX_DB - GRAPH_MIN_DB);
            return plotMax.y - (ratio * (plotMax.y - plotMin.y));
        };
        char label[32];
        for (int tick = 0; tick <= 4; ++tick) {
            const float level = GRAPH_MIN_DB +
                                ((GRAPH_MAX_DB - GRAPH_MIN_DB) * tick / 4.0f);
            const float y = levelToY(level);
            drawList->AddLine(
                ImVec2(plotMin.x, y), ImVec2(plotMax.x, y),
                IM_COL32(48, 55, 65, 255));
            std::snprintf(label, sizeof(label), "%.0f", level);
            drawList->AddText(
                ImVec2(canvasPosition.x + (2.0f * scale), y - (7.0f * scale)),
                IM_COL32(175, 180, 190, 255), label);
        }

        auto drawTrace = [&](const std::vector<float>& data, ImU32 color,
                             float thickness) {
            if (data.size() < 2) {
                return;
            }
            bool havePrevious = false;
            ImVec2 previous;
            for (std::size_t index = 0; index < data.size(); ++index) {
                if (!isValidDbfs(data[index])) {
                    havePrevious = false;
                    continue;
                }
                const float xRatio = static_cast<float>(index) /
                                     static_cast<float>(data.size() - 1);
                const ImVec2 point(
                    plotMin.x + (xRatio * (plotMax.x - plotMin.x)),
                    levelToY(data[index]));
                if (havePrevious) {
                    drawList->AddLine(previous, point, color, thickness * scale);
                }
                previous = point;
                havePrevious = true;
            }
        };

        drawTrace(primary, IM_COL32(50, 225, 155, 255), 1.2f);
        if (secondary) {
            drawTrace(*secondary, IM_COL32(80, 155, 255, 235), 1.0f);
            char legend[96];
            std::snprintf(legend, sizeof(legend),
                          "green: MAX | blue: %s", secondaryLabel);
            drawList->AddText(
                ImVec2(plotMin.x + (4.0f * scale), plotMin.y + (2.0f * scale)),
                IM_COL32(205, 215, 225, 255), legend);
        }
        if (primary.empty()) {
            drawList->AddText(
                ImVec2(plotMin.x + (8.0f * scale), plotMin.y + (8.0f * scale)),
                IM_COL32(190, 195, 205, 255), "Waiting for diagnostic spectrum");
        }
    }

    static void drawFrameComparison(const char* label,
                                    const FFTFrameComparison& comparison) {
        if (!comparison.available) {
            ImGui::Text("%s: unavailable", label);
            return;
        }
        ImGui::Text(
            "%s: mean |dB| %.3f, <=1 dB %.1f%%, <=3 dB %.1f%% (%zu bins)",
            label, comparison.meanAbsoluteDifferenceDb,
            comparison.withinOneDbPercent, comparison.withinThreeDbPercent,
            comparison.comparedBins);
    }

    static void drawReducerContributor(
        const WideBinReducerContributor& contributor) {
        ImGui::TextWrapped(
            "contributors=%zu | highest/2nd/4th/8th %.3f / %.3f / %.3f / %.3f "
            "dBFS | P95/P98/mean %.3f / %.3f / %.3f dBFS",
            contributor.contributingRawBins,
            contributor.highestRawDbfs,
            contributor.secondHighestRawDbfs,
            contributor.fourthHighestRawDbfs,
            contributor.eighthHighestRawDbfs,
            contributor.percentile95Dbfs,
            contributor.percentile98Dbfs,
            contributor.meanDbfs);
    }

    void drawRobustReducerDiagnostics(
        const RobustReducerDiagnostics& diagnostics) {
        if (!ImGui::CollapsingHeader("V13 Robust reducer diagnostic",
                                     ImGuiTreeNodeFlags_DefaultOpen)) {
            return;
        }
        if (!diagnostics.available) {
            ImGui::TextDisabled("Waiting for mapped direct FFT data.");
            return;
        }

        ImGui::TextUnformatted(
            "Production Combined Spectrum remains MAX; comparison is diagnostic only.");
        int comparisonSelection = std::clamp(
            v13ComparisonReducer - static_cast<int>(V13_REDUCER_MEAN), 0, 4);
        if (ImGui::Combo(("Comparison reducer##wsm_v13_reducer_" + name).c_str(),
                         &comparisonSelection,
                         "MEAN\0P95\0P98\0TOP4_MEAN\0TOP8_MEAN\0")) {
            v13ComparisonReducer = comparisonSelection +
                                   static_cast<int>(V13_REDUCER_MEAN);
        }
        const std::size_t selectedReducer = static_cast<std::size_t>(
            std::clamp(v13ComparisonReducer,
                       static_cast<int>(V13_REDUCER_MEAN),
                       static_cast<int>(V13_REDUCER_TOP8)));
        const std::vector<float>* selectedTrace =
            robustReducerTrace(diagnostics, selectedReducer);
        const std::string graphTitle =
            std::string("MAX vs ") + robustReducerName(selectedReducer) +
            " (same wide bins)";
        drawV12SpectrumGraph(
            graphTitle.c_str(), "v13_reducer", diagnostics.maximum,
            selectedTrace, robustReducerName(selectedReducer));

        const ImGuiTableFlags tableFlags =
            ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
            ImGuiTableFlags_SizingStretchProp;
        ImGui::TextUnformatted("MAX minus reducer across mapped wide bins");
        if (ImGui::BeginTable("##wsm_v13_differences", 8, tableFlags)) {
            ImGui::TableSetupColumn("Reducer");
            ImGui::TableSetupColumn("Mean");
            ImGui::TableSetupColumn("Median");
            ImGui::TableSetupColumn("P90");
            ImGui::TableSetupColumn(">3%");
            ImGui::TableSetupColumn(">6%");
            ImGui::TableSetupColumn(">10%");
            ImGui::TableSetupColumn("Bins");
            ImGui::TableHeadersRow();
            for (std::size_t reducer = 0; reducer < V13_REDUCER_COUNT; ++reducer) {
                const ReducerDifferenceStatistics& statistics =
                    diagnostics.difference[reducer];
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::TextUnformatted(robustReducerName(reducer));
                if (!statistics.available) {
                    ImGui::TableSetColumnIndex(1);
                    ImGui::TextUnformatted("n/a");
                    continue;
                }
                ImGui::TableSetColumnIndex(1);
                ImGui::Text("%.2f", statistics.averageDifferenceDb);
                ImGui::TableSetColumnIndex(2);
                ImGui::Text("%.2f", statistics.medianDifferenceDb);
                ImGui::TableSetColumnIndex(3);
                ImGui::Text("%.2f", statistics.percentile90DifferenceDb);
                ImGui::TableSetColumnIndex(4);
                ImGui::Text("%.1f", statistics.aboveThreeDbPercent);
                ImGui::TableSetColumnIndex(5);
                ImGui::Text("%.1f", statistics.aboveSixDbPercent);
                ImGui::TableSetColumnIndex(6);
                ImGui::Text("%.1f", statistics.aboveTenDbPercent);
                ImGui::TableSetColumnIndex(7);
                ImGui::Text("%zu", statistics.comparedBins);
            }
            ImGui::EndTable();
        }

        ImGui::Text("MAX local peaks >=6 dB over median baseline (+/-5): %zu",
                    diagnostics.detectedMaximumPeaks);
        if (ImGui::BeginTable("##wsm_v13_peaks", 9, tableFlags)) {
            ImGui::TableSetupColumn("Reducer");
            ImGui::TableSetupColumn("Avg loss");
            ImGui::TableSetupColumn("Median");
            ImGui::TableSetupColumn("Worst");
            ImGui::TableSetupColumn("Avg SNR");
            ImGui::TableSetupColumn("<=1%");
            ImGui::TableSetupColumn("<=3%");
            ImGui::TableSetupColumn("<=6%");
            ImGui::TableSetupColumn("Peaks");
            ImGui::TableHeadersRow();
            for (std::size_t reducer = 0; reducer < V13_REDUCER_COUNT; ++reducer) {
                const PeakPreservationStatistics& statistics =
                    diagnostics.peakPreservation[reducer];
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::TextUnformatted(robustReducerName(reducer));
                if (!statistics.available) {
                    ImGui::TableSetColumnIndex(1);
                    ImGui::TextUnformatted("n/a");
                    continue;
                }
                ImGui::TableSetColumnIndex(1);
                ImGui::Text("%.2f", statistics.averageLossDb);
                ImGui::TableSetColumnIndex(2);
                ImGui::Text("%.2f", statistics.medianLossDb);
                ImGui::TableSetColumnIndex(3);
                ImGui::Text("%.2f", statistics.worstLossDb);
                ImGui::TableSetColumnIndex(4);
                ImGui::Text("%.2f", statistics.averageLocalSnrDb);
                ImGui::TableSetColumnIndex(5);
                ImGui::Text("%.1f", statistics.withinOneDbPercent);
                ImGui::TableSetColumnIndex(6);
                ImGui::Text("%.1f", statistics.withinThreeDbPercent);
                ImGui::TableSetColumnIndex(7);
                ImGui::Text("%.1f", statistics.withinSixDbPercent);
                ImGui::TableSetColumnIndex(8);
                ImGui::Text("%zu", statistics.measuredPeaks);
            }
            ImGui::EndTable();
        }

        ImGui::TextUnformatted("Descriptive reducer scores (no automatic selection)");
        ImGui::TextDisabled(
            "Suppression=min(100, mean difference/10 dB*100); preservation="
            "max(0, 100-average peak loss/6 dB*100); combined=equal-weight mean.");
        if (ImGui::BeginTable("##wsm_v13_scores", 4, tableFlags)) {
            ImGui::TableSetupColumn("Reducer");
            ImGui::TableSetupColumn("Suppression");
            ImGui::TableSetupColumn("Preservation");
            ImGui::TableSetupColumn("Combined");
            ImGui::TableHeadersRow();
            for (std::size_t reducer = 0; reducer < V13_REDUCER_COUNT; ++reducer) {
                const ReducerDiagnosticScore& score = diagnostics.score[reducer];
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::TextUnformatted(robustReducerName(reducer));
                ImGui::TableSetColumnIndex(1);
                if (!score.available) {
                    ImGui::TextUnformatted("n/a");
                    continue;
                }
                ImGui::Text("%.1f", score.artifactSuppressionScore);
                ImGui::TableSetColumnIndex(2);
                ImGui::Text("%.1f", score.peakPreservationScore);
                ImGui::TableSetColumnIndex(3);
                ImGui::Text("%.1f", score.combinedScore);
            }
            ImGui::EndTable();
        }

        ImGui::Separator();
        ImGui::InputDouble(
            ("Reference frequency MHz##wsm_v13_reference_" + name).c_str(),
            &v13ReferenceFrequencyMHz, 0.001, 0.01, "%.6f");
        const double referenceHz = v13ReferenceFrequencyMHz * 1e6;
        if (referenceHz >= diagnostics.startHz && referenceHz < diagnostics.stopHz &&
            !diagnostics.maximum.empty()) {
            const double binWidthHz =
                (diagnostics.stopHz - diagnostics.startHz) /
                static_cast<double>(diagnostics.maximum.size());
            const std::size_t centerBin = std::min(
                diagnostics.maximum.size() - 1,
                static_cast<std::size_t>(
                    std::floor((referenceHz - diagnostics.startHz) / binWidthHz)));
            ImGui::Text("Reference wide bin: %zu (no tuner action)", centerBin);
            if (ImGui::BeginTable("##wsm_v13_reference_bins", 5, tableFlags)) {
                ImGui::TableSetupColumn("Bin / MHz");
                ImGui::TableSetupColumn("Reducer");
                ImGui::TableSetupColumn("Level");
                ImGui::TableSetupColumn("MAX diff");
                ImGui::TableSetupColumn("Offset");
                ImGui::TableHeadersRow();
                const std::size_t firstBin = centerBin > 0 ? centerBin - 1 : centerBin;
                const std::size_t lastBin = std::min(
                    diagnostics.maximum.size() - 1, centerBin + 1);
                for (std::size_t bin = firstBin; bin <= lastBin; ++bin) {
                    const double frequencyHz = diagnostics.startHz +
                                               ((static_cast<double>(bin) + 0.5) *
                                                binWidthHz);
                    for (std::size_t reducer = 0; reducer < V13_REDUCER_COUNT;
                         ++reducer) {
                        const std::vector<float>* trace =
                            robustReducerTrace(diagnostics, reducer);
                        ImGui::TableNextRow();
                        ImGui::TableSetColumnIndex(0);
                        ImGui::Text("%zu / %.6f", bin, frequencyHz / 1e6);
                        ImGui::TableSetColumnIndex(1);
                        ImGui::TextUnformatted(robustReducerName(reducer));
                        ImGui::TableSetColumnIndex(2);
                        if (!trace || !isValidDbfs((*trace)[bin])) {
                            ImGui::TextUnformatted("n/a");
                            continue;
                        }
                        ImGui::Text("%.3f", (*trace)[bin]);
                        ImGui::TableSetColumnIndex(3);
                        ImGui::Text("%.3f",
                                    diagnostics.maximum[bin] - (*trace)[bin]);
                        ImGui::TableSetColumnIndex(4);
                        ImGui::Text("%+.0f", static_cast<double>(bin) -
                                                 static_cast<double>(centerBin));
                    }
                }
                ImGui::EndTable();
            }
            ImGui::TextUnformatted("Reference-bin raw contributors");
            drawReducerContributor(diagnostics.contributors[centerBin]);
        }
        else {
            ImGui::TextDisabled(
                "Enter a frequency inside the current sweep range for bin details.");
        }

        if (ImGui::TreeNode("Strongest 10 MAX peaks and contributors")) {
            for (std::size_t peakIndex = 0;
                 peakIndex < diagnostics.strongestPeaks.size(); ++peakIndex) {
                const RobustPeakDiagnostic& peak =
                    diagnostics.strongestPeaks[peakIndex];
                ImGui::Text("#%zu bin %zu @ %.6f MHz | MAX baseline %.3f dBFS",
                            peakIndex + 1, peak.wideBinIndex,
                            peak.frequencyHz / 1e6, peak.maximumBaselineDbfs);
                drawReducerContributor(peak.contributor);
                for (std::size_t reducer = 0; reducer < V13_REDUCER_COUNT;
                     ++reducer) {
                    ImGui::TextWrapped(
                        "  %s level %.3f dBFS | MAX loss %.3f dB | local SNR %.3f dB",
                        robustReducerName(reducer), peak.levelsDbfs[reducer],
                        peak.lossesDb[reducer], peak.localSnrDb[reducer]);
                }
                ImGui::Separator();
            }
            if (diagnostics.strongestPeaks.empty()) {
                ImGui::TextDisabled("No qualifying MAX peaks in current data.");
            }
            ImGui::TreePop();
        }
    }

    void drawSpectrumArtifactDiagnostics(
        const SpectrumArtifactDiagnostics& diagnostics) {
        if (!ImGui::CollapsingHeader("V12 Spectrum artefact diagnostic",
                                     ImGuiTreeNodeFlags_DefaultOpen)) {
            return;
        }
        if (!diagnostics.available) {
            ImGui::TextDisabled("Waiting for the first valid direct FFT frame.");
            return;
        }

        ImGui::Text("Current diagnostic segment: %d | center %.6f MHz",
                    diagnostics.segmentNumber, diagnostics.centerHz / 1e6);
        ImGui::Text("Raw valid bins: %zu / %zu", diagnostics.validRawBinCount,
                    diagnostics.rawBinCount);
        ImGui::Text("Raw min / max: %.3f / %.3f dBFS",
                    diagnostics.rawMinimumDbfs, diagnostics.rawMaximumDbfs);
        ImGui::Text("Raw mean / median: %.3f / %.3f dBFS",
                    diagnostics.rawMeanDbfs, diagnostics.rawMedianDbfs);
        ImGui::Text("Raw standard deviation: %.3f dB",
                    diagnostics.rawStandardDeviationDb);
        ImGui::Text("Estimated noise floor (20th percentile): %.3f dBFS",
                    diagnostics.estimatedNoiseFloorDbfs);
        ImGui::Text("Local peaks above floor +6 / +10 dB: %zu / %zu",
                    diagnostics.localPeaksAboveSixDb,
                    diagnostics.localPeaksAboveTenDb);
        if (std::isfinite(diagnostics.averagePeakSpacingBins)) {
            const double rawBinWidthHz = diagnostics.rawBinCount > 0
                                             ? diagnostics.sampleRateHz /
                                                   static_cast<double>(diagnostics.rawBinCount)
                                             : 0.0;
            ImGui::Text("Average peak spacing: %.3f bins / %.3f Hz",
                        diagnostics.averagePeakSpacingBins,
                        diagnostics.averagePeakSpacingBins * rawBinWidthHz);
            ImGui::Text("Dominant peak spacing: %zu bins / %.3f Hz",
                        diagnostics.dominantPeakSpacingBins,
                        diagnostics.dominantPeakSpacingHz);
        }
        else {
            ImGui::TextUnformatted("Peak spacing: insufficient +6 dB local peaks");
        }
        if (diagnostics.strongestPeriodBins > 0 &&
            std::isfinite(diagnostics.strongestPeriodScore)) {
            ImGui::Text("Strongest periodicity: %zu bins / %.3f Hz | score %.6f",
                        diagnostics.strongestPeriodBins,
                        diagnostics.strongestPeriodHz,
                        diagnostics.strongestPeriodScore);
        }
        else {
            ImGui::TextUnformatted("Strongest periodicity: unavailable");
        }

        ImGui::Separator();
        ImGui::TextUnformatted("Raw-vs-merged indicator (descriptive only)");
        if (diagnostics.maxMeanComparableBins > 0) {
            ImGui::Text("wideMax - wideMean average: %.3f dB (%zu bins)",
                        diagnostics.averageMaxMinusMeanDb,
                        diagnostics.maxMeanComparableBins);
            ImGui::Text("Difference >3 / >6 dB: %.1f%% / %.1f%%",
                        diagnostics.maxMinusMeanAboveThreeDbPercent,
                        diagnostics.maxMinusMeanAboveSixDbPercent);
        }
        else {
            ImGui::TextUnformatted("wideMax vs wideMean: unavailable");
        }
        ImGui::TextDisabled(
            "No automatic conclusion: compare raw and merged traces manually.");

        const SegmentContributionDiagnostics& contribution = diagnostics.contribution;
        ImGui::Separator();
        ImGui::TextUnformatted("Current segment contribution");
        ImGui::Text("Center: %.6f MHz", contribution.centerHz / 1e6);
        ImGui::Text("Raw start / stop: %.6f / %.6f MHz",
                    contribution.rawStartHz / 1e6, contribution.rawStopHz / 1e6);
        ImGui::Text("Usable start / stop: %.6f / %.6f MHz",
                    contribution.usableStartHz / 1e6,
                    contribution.usableStopHz / 1e6);
        ImGui::Text("Raw bins used: %zu", contribution.rawBinsUsed);
        ImGui::Text("Wide bins hit / multiple contributors: %zu / %zu",
                    contribution.wideBinsHit,
                    contribution.wideBinsWithMultipleRawBins);
        ImGui::Text("Raw bins per wide bin min / max / mean: %zu / %zu / %.3f",
                    contribution.minimumRawBinsPerWideBin,
                    contribution.maximumRawBinsPerWideBin,
                    contribution.averageRawBinsPerWideBin);

        ImGui::Separator();
        ImGui::Text("Frame consistency: %zu consecutive captured frame(s)",
                    diagnostics.consistencyFrameCount);
        drawFrameComparison("Frame 1 -> 2", diagnostics.frameOneToTwo);
        drawFrameComparison("Frame 2 -> 3", diagnostics.frameTwoToThree);

        ImGui::Separator();
        drawV12SpectrumGraph(
            "Raw FFT - current segment (2048-point max envelope, display only)",
            "raw_fft", diagnostics.rawFFTDisplay);
        drawV12SpectrumGraph(
            "Merged wide spectrum - wideMax vs wideMean (diagnostic)",
            "max_mean", diagnostics.mergedWideMaximum,
            &diagnostics.mergedWideMean);

        if (ImGui::TreeNode("Periodicity candidates (Pearson score)")) {
            const ImGuiTableFlags tableFlags =
                ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                ImGuiTableFlags_SizingStretchProp;
            if (ImGui::BeginTable("##wsm_v12_periodicity", 4, tableFlags)) {
                ImGui::TableSetupColumn("Bins");
                ImGui::TableSetupColumn("Hz");
                ImGui::TableSetupColumn("Score");
                ImGui::TableSetupColumn("Mean |dB|");
                ImGui::TableHeadersRow();
                for (const FFTPeriodicityDiagnostic& periodicity :
                     diagnostics.periodicity) {
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::Text("%zu", periodicity.periodBins);
                    ImGui::TableSetColumnIndex(1);
                    ImGui::Text("%.1f", periodicity.periodHz);
                    ImGui::TableSetColumnIndex(2);
                    if (std::isfinite(periodicity.score)) {
                        ImGui::Text("%.6f", periodicity.score);
                    }
                    else {
                        ImGui::TextUnformatted("n/a");
                    }
                    ImGui::TableSetColumnIndex(3);
                    if (std::isfinite(periodicity.meanAbsoluteDifferenceDb)) {
                        ImGui::Text("%.3f", periodicity.meanAbsoluteDifferenceDb);
                    }
                    else {
                        ImGui::TextUnformatted("n/a");
                    }
                }
                ImGui::EndTable();
            }
            ImGui::TreePop();
        }

        if (ImGui::TreeNode("Fixed raw FFT samples")) {
            const ImGuiTableFlags tableFlags =
                ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                ImGuiTableFlags_SizingStretchProp;
            if (ImGui::BeginTable("##wsm_v12_fixed_samples", 3, tableFlags)) {
                ImGui::TableSetupColumn("Bin");
                ImGui::TableSetupColumn("Offset kHz");
                ImGui::TableSetupColumn("dBFS");
                ImGui::TableHeadersRow();
                for (std::size_t sample = 0;
                     sample < diagnostics.fixedSamplesWritten; ++sample) {
                    const RawFFTFixedSample& fixedSample =
                        diagnostics.fixedSamples[sample];
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::Text("%zu", fixedSample.bin);
                    ImGui::TableSetColumnIndex(1);
                    ImGui::Text("%.3f", fixedSample.offsetHz / 1e3);
                    ImGui::TableSetColumnIndex(2);
                    ImGui::Text("%.6f", fixedSample.valueDbfs);
                }
                ImGui::EndTable();
            }
            ImGui::TreePop();
        }

        if (ImGui::TreeNode("Sampled wide-bin contributions")) {
            for (std::size_t sample = 0;
                 sample < contribution.samplesWritten; ++sample) {
                const WideBinContributionSample& contributionSample =
                    contribution.samples[sample];
                ImGui::TextWrapped(
                    "[%zu] %.6f MHz: n=%zu, max=%.3f, top=%.3f, second=%.3f dBFS",
                    contributionSample.wideBinIndex,
                    contributionSample.frequencyHz / 1e6,
                    contributionSample.contributingRawBins,
                    contributionSample.selectedMaximumDbfs,
                    contributionSample.highestRawDbfs,
                    contributionSample.secondHighestRawDbfs);
            }
            ImGui::TreePop();
        }
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
        SpectrumArtifactDiagnostics artifactDiagnostics;
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
            artifactDiagnostics = currentArtifactDiagnostics;
        }
        const IQCaptureDiagnostics iqDiagnostics = snapshotIQCaptureDiagnostics();
        SplitterDiagnosticCheckpoint beforePrivateBind;
        SplitterDiagnosticCheckpoint afterPrivateBind;
        SplitterDiagnosticCheckpoint beforeFirstTune;
        SplitterDiagnosticCheckpoint immediatelyAfterFirstTune;
        SplitterDiagnosticCheckpoint afterFirstTuneSettling;
        SplitterDiagnosticCheckpoint captureTimeout;
        {
            std::lock_guard<std::mutex> lock(splitterCheckpointMutex);
            beforePrivateBind = beforePrivateBindCheckpoint;
            afterPrivateBind = afterPrivateBindCheckpoint;
            beforeFirstTune = beforeFirstTuneCheckpoint;
            immediatelyAfterFirstTune = immediatelyAfterFirstTuneCheckpoint;
            afterFirstTuneSettling = afterFirstTuneSettlingCheckpoint;
            captureTimeout = captureTimeoutCheckpoint;
        }
        const std::uintptr_t fftInputAddress = sigpath::iqFrontEnd.getFFTInputStreamAddress();

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
        ImGui::TextUnformatted("V10 IQ capture lifecycle");
        ImGui::Text("Capture path creation attempts: %llu", static_cast<unsigned long long>(iqDiagnostics.pathCreationAttempts));
        ImGui::Text("Private stream allocated: %s", iqDiagnostics.privateStreamAllocated ? "yes" : "no");
        ImGui::Text("bindIQStream completed: %s", iqDiagnostics.bindCompleted ? "yes" : "no");
        ImGui::Text("Sink initialized with private stream: %s", iqDiagnostics.sinkInitialized ? "yes" : "no");
        ImGui::Text("Sink started: %s", iqDiagnostics.sinkStarted ? "yes" : "no");
        ImGui::Text("Frontend currently playing: %s", iqDiagnostics.frontendPlaying ? "yes" : "no");
        ImGui::Text("Private IQ stream pointer: 0x%llX", static_cast<unsigned long long>(iqDiagnostics.privateStreamAddress));
        ImGui::Text("Sink input pointer: 0x%llX", static_cast<unsigned long long>(iqDiagnostics.sinkInputAddress));
        ImGui::Text("bindIQStream pointer: 0x%llX", static_cast<unsigned long long>(iqDiagnostics.boundStreamAddress));
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
        ImGui::TextUnformatted("V11 IQFrontend splitter blocking diagnostic");
        ImGui::Text("Known fftIn stream pointer: 0x%llX",
                    static_cast<unsigned long long>(fftInputAddress));
        ImGui::Text("Known WSM stream pointer: 0x%llX",
                    static_cast<unsigned long long>(iqDiagnostics.privateStreamAddress));
        drawSplitterDiagnosticCheckpoint("Before private bind", beforePrivateBind,
                                         fftInputAddress, iqDiagnostics.privateStreamAddress);
        drawSplitterDiagnosticCheckpoint("After private bind", afterPrivateBind,
                                         fftInputAddress, iqDiagnostics.privateStreamAddress);
        drawSplitterDiagnosticCheckpoint("Before first sweep tune", beforeFirstTune,
                                         fftInputAddress, iqDiagnostics.privateStreamAddress);
        drawSplitterDiagnosticCheckpoint("Immediately after first sweep tune",
                                         immediatelyAfterFirstTune, fftInputAddress,
                                         iqDiagnostics.privateStreamAddress);
        drawSplitterDiagnosticCheckpoint("After first sweep tune settling",
                                         afterFirstTuneSettling, fftInputAddress,
                                         iqDiagnostics.privateStreamAddress);
        drawSplitterDiagnosticCheckpoint("At capture timeout", captureTimeout,
                                         fftInputAddress, iqDiagnostics.privateStreamAddress);
        ImGui::Separator();
        drawSpectrumArtifactDiagnostics(artifactDiagnostics);
        ImGui::Separator();
        drawRobustReducerDiagnostics(artifactDiagnostics.robustReducers);
        ImGui::Separator();
        ImGui::TextUnformatted("FFT source: private IQFrontEnd stream (V10 lifecycle)");
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
    dsp::sink::Handler<dsp::complex_t>* iqSink = nullptr;
    bool iqSinkInitialized = false;
    bool iqStreamBound = false;
    std::mutex splitterCheckpointMutex;
    SplitterDiagnosticCheckpoint beforePrivateBindCheckpoint;
    SplitterDiagnosticCheckpoint afterPrivateBindCheckpoint;
    SplitterDiagnosticCheckpoint beforeFirstTuneCheckpoint;
    SplitterDiagnosticCheckpoint immediatelyAfterFirstTuneCheckpoint;
    SplitterDiagnosticCheckpoint afterFirstTuneSettlingCheckpoint;
    SplitterDiagnosticCheckpoint captureTimeoutCheckpoint;
    std::mutex iqCaptureMutex;
    std::condition_variable iqCaptureCv;
    std::atomic<std::uint64_t> iqCapturePathCreationAttempts{ 0 };
    std::atomic<bool> iqLastPathStreamAllocated{ false };
    std::atomic<bool> iqLastPathBindCompleted{ false };
    std::atomic<bool> iqLastPathSinkInitialized{ false };
    std::atomic<bool> iqLastPathSinkStarted{ false };
    std::atomic<bool> iqFrontendCurrentlyPlaying{ false };
    std::atomic<std::uintptr_t> iqLastPrivateStreamAddress{ 0 };
    std::atomic<std::uintptr_t> iqLastSinkInputAddress{ 0 };
    std::atomic<std::uintptr_t> iqLastBoundStreamAddress{ 0 };
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
    SpectrumArtifactDiagnostics currentArtifactDiagnostics;
    std::vector<SegmentBoundaryDiagnostic> activeSegmentBoundaries;
    int v13ComparisonReducer = V13_REDUCER_P98;
    double v13ReferenceFrequencyMHz = 0.0;

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
