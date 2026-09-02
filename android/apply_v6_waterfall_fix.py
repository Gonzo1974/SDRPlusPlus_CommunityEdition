#!/usr/bin/env python3
from pathlib import Path
import runpy

root = Path(__file__).resolve().parents[1]
source = root / "core" / "src" / "gui" / "widgets" / "waterfall.cpp"
text = source.read_text(encoding="utf-8")

old = """    void WaterFall::onResize() {\n        std::lock_guard<std::recursive_mutex> lck(latestFFTMtx);\n        std::lock_guard<std::mutex> lck2(smoothingBufMtx);\n"""
new = """    void WaterFall::onResize() {\n        // V6: rawFFTs is also reallocated in this function. Protect it with\n        // the same recursive mutex used by getFFTBuffer()/acquireRawFFT().\n        // latestFFTMtx and smoothingBufMtx are acquired together to avoid\n        // introducing another lock-order inversion.\n        std::lock_guard<std::recursive_mutex> rawLck(buf_mtx);\n        std::scoped_lock displayLck(latestFFTMtx, smoothingBufMtx);\n"""

if new in text:
    print("V6 waterfall race fix already applied")
elif old in text:
    source.write_text(text.replace(old, new, 1), encoding="utf-8")
    print("Applied V6 waterfall raw-FFT resize race fix")
else:
    raise SystemExit("Could not find expected WaterFall::onResize() lock block; refusing to patch")

# V7 producer-level FFT diagnostics are applied in the same Android build step,
# so the existing GitHub Actions workflow does not need another dedicated step.
v7 = Path(__file__).with_name("apply_v7_fft_diagnostics.py")
if not v7.exists():
    raise SystemExit("V7 diagnostics patch script is missing")
runpy.run_path(str(v7), run_name="__main__")
