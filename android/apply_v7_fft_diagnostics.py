#!/usr/bin/env python3
from pathlib import Path

root = Path(__file__).resolve().parents[1]
hdr = root / "core" / "src" / "signal_path" / "iq_frontend.h"
src = root / "core" / "src" / "signal_path" / "iq_frontend.cpp"
wsm = root / "misc_modules" / "wide_spectrum_monitor" / "src" / "main.cpp"

# --- iq_frontend.h ---------------------------------------------------------
text = hdr.read_text(encoding="utf-8")
if "struct FFTProducerDiagnostics" not in text:
    text = text.replace('#include <fftw3.h>\n', '#include <fftw3.h>\n#include <cstdint>\n#include <mutex>\n')
    marker = """    enum FFTWindow {\n        RECTANGULAR,\n        BLACKMAN,\n        NUTTALL\n    };\n\n"""
    insert = marker + """    struct FFTProducerDiagnostics {\n        static constexpr int SAMPLE_COUNT = 9;\n        std::uint64_t frameCounter = 0;\n        int fftSize = 0;\n        int nzFFTSize = 0;\n        int inputCount = 0;\n        std::uint64_t fftFiniteComplex = 0;\n        std::uint64_t fftZeroComplex = 0;\n        std::uint64_t fftNonFiniteComplex = 0;\n        float fftMagnitudeMin = 0.0f;\n        float fftMagnitudeMax = 0.0f;\n        bool outputAvailable = false;\n        std::uint64_t volkFinite = 0;\n        std::uint64_t volkZero = 0;\n        std::uint64_t volkNan = 0;\n        std::uint64_t volkInf = 0;\n        std::uint64_t volkNegative = 0;\n        std::uint64_t volkPositive = 0;\n        float volkFiniteMin = 0.0f;\n        float volkFiniteMax = 0.0f;\n        int sampleIndices[SAMPLE_COUNT] = {};\n        float fftSampleReal[SAMPLE_COUNT] = {};\n        float fftSampleImag[SAMPLE_COUNT] = {};\n        float volkSample[SAMPLE_COUNT] = {};\n    };\n\n"""
    if marker not in text:
        raise SystemExit("V7 header: FFTWindow marker not found")
    text = text.replace(marker, insert, 1)
    method_marker = "    double getEffectiveSamplerate();\n"
    text = text.replace(method_marker, method_marker + "    FFTProducerDiagnostics getFFTProducerDiagnostics();\n", 1)
    member_marker = "    double effectiveSr;\n\n"
    text = text.replace(member_marker, member_marker + "    std::mutex fftDiagMtx;\n    FFTProducerDiagnostics fftDiag;\n\n", 1)
    hdr.write_text(text, encoding="utf-8")
else:
    print("V7 header diagnostics already present")

# --- iq_frontend.cpp -------------------------------------------------------
text = src.read_text(encoding="utf-8")
if "getFFTProducerDiagnostics" not in text:
    text = text.replace('#include <core.h>\n', '#include <core.h>\n#include <algorithm>\n#include <cmath>\n#include <limits>\n')

    getter_marker = """double IQFrontEnd::getEffectiveSamplerate() {\n    return effectiveSr;\n}\n\n"""
    getter_insert = getter_marker + """IQFrontEnd::FFTProducerDiagnostics IQFrontEnd::getFFTProducerDiagnostics() {\n    std::lock_guard<std::mutex> lock(fftDiagMtx);\n    return fftDiag;\n}\n\n"""
    if getter_marker not in text:
        raise SystemExit("V7 source: getEffectiveSamplerate marker not found")
    text = text.replace(getter_marker, getter_insert, 1)

    old_handler = """void IQFrontEnd::handler(dsp::complex_t* data, int count, void* ctx) {\n    IQFrontEnd* _this = (IQFrontEnd*)ctx;\n\n    // Apply window\n    volk_32fc_32f_multiply_32fc((lv_32fc_t*)_this->fftInBuf, (lv_32fc_t*)data, _this->fftWindowBuf, _this->_nzFFTSize);\n\n    // Execute FFT\n    fftwf_execute(_this->fftwPlan);\n\n    // Aquire buffer\n    float* fftBuf = _this->_acquireFFTBuffer(_this->_fftCtx);\n\n    // Convert the complex output of the FFT to dB amplitude\n    if (fftBuf) {\n        volk_32fc_s32f_power_spectrum_32f(fftBuf, (lv_32fc_t*)_this->fftOutBuf, _this->_fftSize, _this->_fftSize);\n    }\n\n    // Release buffer\n    _this->_releaseFFTBuffer(_this->_fftCtx);\n}\n"""
    new_handler = r'''void IQFrontEnd::handler(dsp::complex_t* data, int count, void* ctx) {
    IQFrontEnd* _this = (IQFrontEnd*)ctx;

    // Apply window
    volk_32fc_32f_multiply_32fc((lv_32fc_t*)_this->fftInBuf, (lv_32fc_t*)data, _this->fftWindowBuf, _this->_nzFFTSize);

    // Execute FFT
    fftwf_execute(_this->fftwPlan);

    // V7 producer-level diagnostics: inspect FFTW output before VOLK so we can
    // distinguish an FFT/input problem from a VOLK power-spectrum problem.
    FFTProducerDiagnostics diag;
    {
        std::lock_guard<std::mutex> lock(_this->fftDiagMtx);
        diag.frameCounter = _this->fftDiag.frameCounter + 1;
    }
    diag.fftSize = _this->_fftSize;
    diag.nzFFTSize = _this->_nzFFTSize;
    diag.inputCount = count;

    float fftMagMin = std::numeric_limits<float>::infinity();
    float fftMagMax = -std::numeric_limits<float>::infinity();
    for (int i = 0; i < _this->_fftSize; ++i) {
        const float re = _this->fftOutBuf[i][0];
        const float im = _this->fftOutBuf[i][1];
        if (!std::isfinite(re) || !std::isfinite(im)) {
            ++diag.fftNonFiniteComplex;
            continue;
        }
        ++diag.fftFiniteComplex;
        if (re == 0.0f && im == 0.0f) {
            ++diag.fftZeroComplex;
        }
        const float mag = std::hypot(re, im);
        if (std::isfinite(mag)) {
            fftMagMin = std::min(fftMagMin, mag);
            fftMagMax = std::max(fftMagMax, mag);
        }
    }
    if (std::isfinite(fftMagMin)) { diag.fftMagnitudeMin = fftMagMin; }
    if (std::isfinite(fftMagMax)) { diag.fftMagnitudeMax = fftMagMax; }

    const int sampleIndices[FFTProducerDiagnostics::SAMPLE_COUNT] = {
        0, 1, _this->_fftSize / 8, _this->_fftSize / 4, _this->_fftSize / 2,
        (_this->_fftSize * 3) / 4, (_this->_fftSize * 7) / 8,
        std::max(0, _this->_fftSize - 2), std::max(0, _this->_fftSize - 1)
    };
    for (int s = 0; s < FFTProducerDiagnostics::SAMPLE_COUNT; ++s) {
        const int idx = std::clamp(sampleIndices[s], 0, std::max(0, _this->_fftSize - 1));
        diag.sampleIndices[s] = idx;
        diag.fftSampleReal[s] = _this->fftOutBuf[idx][0];
        diag.fftSampleImag[s] = _this->fftOutBuf[idx][1];
    }

    // Acquire buffer
    float* fftBuf = _this->_acquireFFTBuffer(_this->_fftCtx);

    // Convert the complex output of the FFT to dB amplitude
    if (fftBuf) {
        diag.outputAvailable = true;
        volk_32fc_s32f_power_spectrum_32f(fftBuf, (lv_32fc_t*)_this->fftOutBuf, _this->_fftSize, _this->_fftSize);

        float volkMin = std::numeric_limits<float>::infinity();
        float volkMax = -std::numeric_limits<float>::infinity();
        for (int i = 0; i < _this->_fftSize; ++i) {
            const float value = fftBuf[i];
            if (std::isnan(value)) {
                ++diag.volkNan;
            }
            else if (std::isinf(value)) {
                ++diag.volkInf;
            }
            else {
                ++diag.volkFinite;
                volkMin = std::min(volkMin, value);
                volkMax = std::max(volkMax, value);
                if (value == 0.0f) { ++diag.volkZero; }
                else if (value < 0.0f) { ++diag.volkNegative; }
                else { ++diag.volkPositive; }
            }
        }
        if (std::isfinite(volkMin)) { diag.volkFiniteMin = volkMin; }
        if (std::isfinite(volkMax)) { diag.volkFiniteMax = volkMax; }
        for (int s = 0; s < FFTProducerDiagnostics::SAMPLE_COUNT; ++s) {
            diag.volkSample[s] = fftBuf[diag.sampleIndices[s]];
        }
    }

    {
        std::lock_guard<std::mutex> lock(_this->fftDiagMtx);
        _this->fftDiag = diag;
    }

    // Release buffer
    _this->_releaseFFTBuffer(_this->_fftCtx);
}
'''
    if old_handler not in text:
        raise SystemExit("V7 source: handler block not found")
    text = text.replace(old_handler, new_handler, 1)
    src.write_text(text, encoding="utf-8")
else:
    print("V7 source diagnostics already present")

# --- wide_spectrum_monitor UI ---------------------------------------------
text = wsm.read_text(encoding="utf-8")
if "Producer FFT diagnostics (V7)" not in text:
    marker = """        drawRawFFTDistribution(segmentDebugStats.rawDistribution);\n        ImGui::Separator();\n"""
    ui = r'''        drawRawFFTDistribution(segmentDebugStats.rawDistribution);
        ImGui::Separator();
        if (ImGui::CollapsingHeader("Producer FFT diagnostics (V7)", ImGuiTreeNodeFlags_DefaultOpen)) {
            const auto producer = sigpath::iqFrontEnd.getFFTProducerDiagnostics();
            ImGui::Text("Producer frame: %llu", static_cast<unsigned long long>(producer.frameCounter));
            ImGui::Text("FFT size / non-zero input / handler count: %d / %d / %d",
                        producer.fftSize, producer.nzFFTSize, producer.inputCount);
            ImGui::Text("FFTW finite complex: %llu", static_cast<unsigned long long>(producer.fftFiniteComplex));
            ImGui::Text("FFTW exact 0+0j: %llu", static_cast<unsigned long long>(producer.fftZeroComplex));
            ImGui::Text("FFTW non-finite complex: %llu", static_cast<unsigned long long>(producer.fftNonFiniteComplex));
            ImGui::Text("FFTW magnitude min/max: %.9g / %.9g",
                        producer.fftMagnitudeMin, producer.fftMagnitudeMax);
            ImGui::Text("VOLK output available: %s", producer.outputAvailable ? "yes" : "no");
            ImGui::Text("VOLK finite / zero: %llu / %llu",
                        static_cast<unsigned long long>(producer.volkFinite),
                        static_cast<unsigned long long>(producer.volkZero));
            ImGui::Text("VOLK NaN / Inf: %llu / %llu",
                        static_cast<unsigned long long>(producer.volkNan),
                        static_cast<unsigned long long>(producer.volkInf));
            ImGui::Text("VOLK negative / positive: %llu / %llu",
                        static_cast<unsigned long long>(producer.volkNegative),
                        static_cast<unsigned long long>(producer.volkPositive));
            ImGui::Text("VOLK finite min/max: %.9g / %.9g",
                        producer.volkFiniteMin, producer.volkFiniteMax);
            ImGui::TextUnformatted("FFTW -> VOLK samples:");
            for (int s = 0; s < IQFrontEnd::FFTProducerDiagnostics::SAMPLE_COUNT; ++s) {
                ImGui::Text("[%d] (%.7g, %.7g) -> %.9g",
                            producer.sampleIndices[s], producer.fftSampleReal[s],
                            producer.fftSampleImag[s], producer.volkSample[s]);
            }
        }
        ImGui::Separator();
'''
    if marker not in text:
        raise SystemExit("V7 WSM: debug UI marker not found")
    text = text.replace(marker, ui, 1)
    wsm.write_text(text, encoding="utf-8")
else:
    print("V7 WSM diagnostics UI already present")

print("Applied V7 producer-level FFT diagnostics")
