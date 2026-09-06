#pragma once
#include "../sink.h"
#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace dsp::routing {
    constexpr std::size_t SPLITTER_DIAGNOSTIC_MAX_OUTPUTS = 64;

    enum class SplitterDiagnosticPhase : std::uint8_t {
        IDLE,
        BEFORE_MEMCPY,
        BEFORE_SWAP,
        AFTER_SWAP
    };

    struct SplitterOutputDiagnosticsSnapshot {
        std::uintptr_t streamAddress = 0;
        std::uint64_t writeAttempts = 0;
        std::uint64_t successfulSwaps = 0;
        std::uint64_t samplesOffered = 0;
    };

    struct SplitterDiagnosticsSnapshot {
        bool enabled = false;
        std::uint64_t runCalls = 0;
        std::uint64_t inputSamples = 0;
        std::size_t outputCount = 0;
        std::size_t trackedOutputCount = 0;
        int currentOutputIndex = -1;
        std::uintptr_t currentOutputAddress = 0;
        SplitterDiagnosticPhase phase = SplitterDiagnosticPhase::IDLE;
        std::uint64_t completedRuns = 0;
        std::uint64_t lastCompletedRun = 0;
        std::uint64_t lastIncompleteRun = 0;
        int lastIncompleteOutputIndex = -1;
        std::uintptr_t lastIncompleteOutputAddress = 0;
        SplitterOutputDiagnosticsSnapshot outputs[SPLITTER_DIAGNOSTIC_MAX_OUTPUTS];
    };

    struct SplitterOutputDiagnosticsState {
        std::atomic<std::uintptr_t> streamAddress{ 0 };
        std::atomic<std::uint64_t> writeAttempts{ 0 };
        std::atomic<std::uint64_t> successfulSwaps{ 0 };
        std::atomic<std::uint64_t> samplesOffered{ 0 };
    };

    struct SplitterDiagnosticsState {
        std::atomic<bool> enabled{ false };
        std::atomic<std::uint64_t> runCalls{ 0 };
        std::atomic<std::uint64_t> inputSamples{ 0 };
        std::atomic<std::size_t> outputCount{ 0 };
        std::atomic<std::size_t> trackedOutputCount{ 0 };
        std::atomic<int> currentOutputIndex{ -1 };
        std::atomic<std::uintptr_t> currentOutputAddress{ 0 };
        std::atomic<SplitterDiagnosticPhase> phase{ SplitterDiagnosticPhase::IDLE };
        std::atomic<std::uint64_t> completedRuns{ 0 };
        std::atomic<std::uint64_t> lastCompletedRun{ 0 };
        std::atomic<std::uint64_t> lastIncompleteRun{ 0 };
        std::atomic<int> lastIncompleteOutputIndex{ -1 };
        std::atomic<std::uintptr_t> lastIncompleteOutputAddress{ 0 };
        SplitterOutputDiagnosticsState outputs[SPLITTER_DIAGNOSTIC_MAX_OUTPUTS];

        void reset() {
            runCalls.store(0, std::memory_order_relaxed);
            inputSamples.store(0, std::memory_order_relaxed);
            outputCount.store(0, std::memory_order_relaxed);
            trackedOutputCount.store(0, std::memory_order_relaxed);
            currentOutputIndex.store(-1, std::memory_order_relaxed);
            currentOutputAddress.store(0, std::memory_order_relaxed);
            phase.store(SplitterDiagnosticPhase::IDLE, std::memory_order_relaxed);
            completedRuns.store(0, std::memory_order_relaxed);
            lastCompletedRun.store(0, std::memory_order_relaxed);
            lastIncompleteRun.store(0, std::memory_order_relaxed);
            lastIncompleteOutputIndex.store(-1, std::memory_order_relaxed);
            lastIncompleteOutputAddress.store(0, std::memory_order_relaxed);
            for (std::size_t i = 0; i < SPLITTER_DIAGNOSTIC_MAX_OUTPUTS; ++i) {
                outputs[i].streamAddress.store(0, std::memory_order_relaxed);
                outputs[i].writeAttempts.store(0, std::memory_order_relaxed);
                outputs[i].successfulSwaps.store(0, std::memory_order_relaxed);
                outputs[i].samplesOffered.store(0, std::memory_order_relaxed);
            }
        }

        SplitterDiagnosticsSnapshot snapshot() const {
            SplitterDiagnosticsSnapshot result;
            result.phase = phase.load(std::memory_order_acquire);
            result.enabled = enabled.load(std::memory_order_relaxed);
            result.runCalls = runCalls.load(std::memory_order_relaxed);
            result.inputSamples = inputSamples.load(std::memory_order_relaxed);
            result.outputCount = outputCount.load(std::memory_order_relaxed);
            result.trackedOutputCount = trackedOutputCount.load(std::memory_order_relaxed);
            result.currentOutputIndex = currentOutputIndex.load(std::memory_order_relaxed);
            result.currentOutputAddress = currentOutputAddress.load(std::memory_order_relaxed);
            result.completedRuns = completedRuns.load(std::memory_order_relaxed);
            result.lastCompletedRun = lastCompletedRun.load(std::memory_order_relaxed);
            result.lastIncompleteRun = lastIncompleteRun.load(std::memory_order_relaxed);
            result.lastIncompleteOutputIndex = lastIncompleteOutputIndex.load(std::memory_order_relaxed);
            result.lastIncompleteOutputAddress = lastIncompleteOutputAddress.load(std::memory_order_relaxed);
            for (std::size_t i = 0; i < SPLITTER_DIAGNOSTIC_MAX_OUTPUTS; ++i) {
                result.outputs[i].streamAddress = outputs[i].streamAddress.load(std::memory_order_relaxed);
                result.outputs[i].writeAttempts = outputs[i].writeAttempts.load(std::memory_order_relaxed);
                result.outputs[i].successfulSwaps = outputs[i].successfulSwaps.load(std::memory_order_relaxed);
                result.outputs[i].samplesOffered = outputs[i].samplesOffered.load(std::memory_order_relaxed);
            }
            return result;
        }
    };

    template <class T>
    class Splitter : public Sink<T> {
        using base_type = Sink<T>;

    public:
        Splitter() {}

        Splitter(stream<T>* in) { base_type::init(in); }

        void setDiagnosticsState(SplitterDiagnosticsState* state) {
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            diagnostics = state;
            if (diagnostics) {
                diagnostics->enabled.store(false, std::memory_order_relaxed);
                diagnostics->reset();
                refreshDiagnosticOutputs();
            }
        }

        void setDiagnosticsEnabled(bool enabled) {
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            if (diagnostics) {
                diagnostics->enabled.store(false, std::memory_order_release);
                if (enabled) {
                    diagnostics->reset();
                    refreshDiagnosticOutputs();
                    diagnostics->enabled.store(true, std::memory_order_release);
                }
            }
        }

        SplitterDiagnosticsSnapshot getDiagnosticsSnapshot() const {
            return diagnostics ? diagnostics->snapshot() : SplitterDiagnosticsSnapshot{};
        }

        void bindStream(stream<T>* stream) {
            assert(base_type::_block_init);
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);

            // Check that the stream isn't already bound
            if (std::find(streams.begin(), streams.end(), stream) != streams.end()) {
                throw std::runtime_error("[Splitter] Tried to bind stream to that is already bound");
            }

            // Add to the list
            base_type::tempStop();
            base_type::registerOutput(stream);
            streams.push_back(stream);
            refreshDiagnosticOutputs();
            base_type::tempStart();
        }

        void unbindStream(stream<T>* stream) {
            assert(base_type::_block_init);
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);

            // Check that the stream is bound
            auto sit = std::find(streams.begin(), streams.end(), stream);
            if (sit == streams.end()) {
                throw std::runtime_error("[Splitter] Tried to unbind stream to that isn't bound");
            }

            // Add to the list
            base_type::tempStop();
            streams.erase(sit);
            base_type::unregisterOutput(stream);
            refreshDiagnosticOutputs();
            base_type::tempStart();
        }

        int run() {
            SplitterDiagnosticsState* activeDiagnostics = diagnostics;
            const bool diagnosticsEnabled = activeDiagnostics &&
                                            activeDiagnostics->enabled.load(std::memory_order_acquire);
            std::uint64_t runSequence = 0;
            if (diagnosticsEnabled) {
                runSequence = activeDiagnostics->runCalls.fetch_add(1, std::memory_order_relaxed) + 1;
                activeDiagnostics->currentOutputIndex.store(-1, std::memory_order_relaxed);
                activeDiagnostics->currentOutputAddress.store(0, std::memory_order_relaxed);
                activeDiagnostics->phase.store(SplitterDiagnosticPhase::IDLE, std::memory_order_release);
            }

            int count = base_type::_in->read();
            if (count < 0) { return -1; }

            if (diagnosticsEnabled) {
                activeDiagnostics->inputSamples.fetch_add(static_cast<std::uint64_t>(count), std::memory_order_relaxed);
                activeDiagnostics->outputCount.store(streams.size(), std::memory_order_relaxed);
                activeDiagnostics->trackedOutputCount.store(
                    std::min(streams.size(), SPLITTER_DIAGNOSTIC_MAX_OUTPUTS), std::memory_order_relaxed);
            }

            for (std::size_t outputIndex = 0; outputIndex < streams.size(); ++outputIndex) {
                stream<T>* stream = streams[outputIndex];
                const std::uintptr_t streamAddress = reinterpret_cast<std::uintptr_t>(stream);
                SplitterOutputDiagnosticsState* outputDiagnostics = nullptr;
                if (diagnosticsEnabled) {
                    activeDiagnostics->currentOutputIndex.store(static_cast<int>(outputIndex), std::memory_order_relaxed);
                    activeDiagnostics->currentOutputAddress.store(streamAddress, std::memory_order_relaxed);
                    activeDiagnostics->phase.store(SplitterDiagnosticPhase::BEFORE_MEMCPY, std::memory_order_release);
                    if (outputIndex < SPLITTER_DIAGNOSTIC_MAX_OUTPUTS) {
                        outputDiagnostics = &activeDiagnostics->outputs[outputIndex];
                        outputDiagnostics->writeAttempts.fetch_add(1, std::memory_order_relaxed);
                        outputDiagnostics->samplesOffered.fetch_add(static_cast<std::uint64_t>(count), std::memory_order_relaxed);
                    }
                }

                memcpy(stream->writeBuf, base_type::_in->readBuf, count * sizeof(T));

                if (diagnosticsEnabled) {
                    activeDiagnostics->lastIncompleteRun.store(runSequence, std::memory_order_relaxed);
                    activeDiagnostics->lastIncompleteOutputIndex.store(static_cast<int>(outputIndex), std::memory_order_relaxed);
                    activeDiagnostics->lastIncompleteOutputAddress.store(streamAddress, std::memory_order_relaxed);
                    activeDiagnostics->phase.store(SplitterDiagnosticPhase::BEFORE_SWAP, std::memory_order_release);
                }
                if (!stream->swap(count)) {
                    if (diagnosticsEnabled) {
                        activeDiagnostics->lastIncompleteRun.store(0, std::memory_order_relaxed);
                        activeDiagnostics->lastIncompleteOutputIndex.store(-1, std::memory_order_relaxed);
                        activeDiagnostics->lastIncompleteOutputAddress.store(0, std::memory_order_relaxed);
                        activeDiagnostics->currentOutputIndex.store(-1, std::memory_order_relaxed);
                        activeDiagnostics->currentOutputAddress.store(0, std::memory_order_relaxed);
                        activeDiagnostics->phase.store(SplitterDiagnosticPhase::IDLE, std::memory_order_release);
                    }
                    base_type::_in->flush();
                    return -1;
                }

                if (diagnosticsEnabled) {
                    if (outputDiagnostics) {
                        outputDiagnostics->successfulSwaps.fetch_add(1, std::memory_order_relaxed);
                    }
                    activeDiagnostics->lastIncompleteRun.store(0, std::memory_order_relaxed);
                    activeDiagnostics->lastIncompleteOutputIndex.store(-1, std::memory_order_relaxed);
                    activeDiagnostics->lastIncompleteOutputAddress.store(0, std::memory_order_relaxed);
                    activeDiagnostics->phase.store(SplitterDiagnosticPhase::AFTER_SWAP, std::memory_order_release);
                }
            }

            base_type::_in->flush();

            if (diagnosticsEnabled) {
                activeDiagnostics->completedRuns.fetch_add(1, std::memory_order_relaxed);
                activeDiagnostics->lastCompletedRun.store(runSequence, std::memory_order_relaxed);
                activeDiagnostics->currentOutputIndex.store(-1, std::memory_order_relaxed);
                activeDiagnostics->currentOutputAddress.store(0, std::memory_order_relaxed);
                activeDiagnostics->phase.store(SplitterDiagnosticPhase::IDLE, std::memory_order_release);
            }

            return count;
        }

    protected:
        void refreshDiagnosticOutputs() {
            if (!diagnostics) { return; }

            const std::size_t trackedCount = std::min(streams.size(), SPLITTER_DIAGNOSTIC_MAX_OUTPUTS);
            diagnostics->outputCount.store(streams.size(), std::memory_order_relaxed);
            diagnostics->trackedOutputCount.store(trackedCount, std::memory_order_relaxed);
            for (std::size_t i = 0; i < SPLITTER_DIAGNOSTIC_MAX_OUTPUTS; ++i) {
                const std::uintptr_t streamAddress = (i < trackedCount)
                                                         ? reinterpret_cast<std::uintptr_t>(streams[i])
                                                         : 0;
                if (diagnostics->outputs[i].streamAddress.load(std::memory_order_relaxed) == streamAddress) {
                    continue;
                }
                diagnostics->outputs[i].streamAddress.store(streamAddress, std::memory_order_relaxed);
                diagnostics->outputs[i].writeAttempts.store(0, std::memory_order_relaxed);
                diagnostics->outputs[i].successfulSwaps.store(0, std::memory_order_relaxed);
                diagnostics->outputs[i].samplesOffered.store(0, std::memory_order_relaxed);
            }
        }

        std::vector<stream<T>*> streams;
        SplitterDiagnosticsState* diagnostics = nullptr;
    };
}
