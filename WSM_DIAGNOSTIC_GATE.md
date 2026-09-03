# Wide Spectrum Monitor — IQ capture diagnostic gate

Target branch: `android-wide-spectrum-monitor`
Baseline commit: `cf4d2cf8c5d343857492d9e56df180b1094faecb`

## Purpose

This is a diagnostic-only step. Do **not** change the IQ/FFT architecture, stream binding order, splitter implementation, tuning behavior, timeout values, FFT code, sweep mapping, or validation rules. Do not claim a fix.

The only question to answer is why `captureIQFrame()` fails before a complete IQ frame is published.

## Source facts already established

- `IQFrontEnd::bindIQStream()` delegates to `Splitter::bindStream()`.
- `Splitter::bindStream()` supports runtime binding using `tempStop()` / `registerOutput()` / `tempStart()`.
- `Splitter::run()` copies every input block to every bound output stream and calls `swap(count)`.
- A fresh `dsp::stream` starts with `canSwap=true`, `readerStop=false`, `writerStop=false`.
- `dsp::sink::Handler::run()` reads, calls its callback and flushes the stream.
- IQ Exporter uses the same `bindIQStream()` API successfully.
- Android RTL-SDR continuously produces IQ blocks into its source stream.
- The existing WSM debug display is misleading on a failed first capture: `SegmentDebugStats` is only published after `captureAveragedFFT()` succeeds, so displayed zeroes do not prove the callback saw zero samples.

## Required instrumentation

Change only `misc_modules/wide_spectrum_monitor/src/main.cpp` and add diagnostic counters/state sufficient to show, even after a failed first capture:

1. Total valid invocations of `iqStreamHandler()` (`data != nullptr && count > 0`).
2. Total IQ samples presented to `iqStreamHandler()`.
3. Handler invocations while `iqCaptureRequested == true`.
4. IQ samples presented while `iqCaptureRequested == true`.
5. Number of chunks intentionally discarded by `discardNextIQChunk`.
6. Number of samples actually copied into `iqCaptureBuffer` for the current/last capture attempt.
7. Capture target sample count.
8. Last handler chunk size.
9. Whether the last `captureIQFrame()` ended by timeout, sweep cancellation/shutdown, or successful frame completion.
10. `iqFrameGeneration` at the end of the last capture attempt.

The counters that are written from `iqStreamHandler()` and read by the GUI must be thread-safe. Prefer atomics for monotonic counters. For per-attempt state already protected by `iqCaptureMutex`, snapshot it safely for display.

## Critical publication requirement

The diagnostic values must be visible in the existing `Debug` section **even when `captureIQFrame()` returns false on the first segment**. Do not store these only inside `SegmentDebugStats`, because that is exactly what made the previous display ambiguous.

Also log one concise failure line from `captureIQFrame()` containing the same essential values. Do not log every IQ chunk.

## Diagnostic interpretation

The tablet result must allow these cases to be distinguished without inference:

- Handler calls = 0: the WSM sink never receives a block.
- Handler calls > 0 but capture-request calls = 0: callback runs, but request timing/state is wrong.
- Capture-request calls > 0 and copied samples remain 0: discard/copy gating is wrong.
- Copied samples > 0 but below target until timeout: partial capture / feed stalls.
- Copied samples reaches target but generation does not advance: completion/notification logic is wrong.
- Generation advances but caller still reports failure: wait predicate/cancellation/result handling is wrong.

## Build gate

Before building:

- Show the exact diff.
- Verify that no functional IQ, FFT, tuning, timeout, splitter, sample-rate, merge, validation, or AGC behavior changed.
- Verify that the branch head used as baseline is `cf4d2cf8c5d343857492d9e56df180b1094faecb` or a descendant containing no unrelated WSM functional changes.

After those checks, build the Android APK through the existing GitHub Actions workflow.

The build is a **diagnostic APK**, not V10 and not a claimed fix.

## Tablet test

One failed first segment is enough. Open `Debug` and capture the diagnostic values listed above. Do not run repeated sweeps merely to see whether it sometimes works.
