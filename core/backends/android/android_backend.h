#pragma once
#include <cstdint>
#include <string>
#include <vector>
#include <stdint.h>

namespace backend {
    struct DevVIDPID {
        uint16_t vid;
        uint16_t pid;
    };

    extern const std::vector<DevVIDPID> AIRSPY_VIDPIDS;
    extern const std::vector<DevVIDPID> AIRSPYHF_VIDPIDS;
    extern const std::vector<DevVIDPID> HACKRF_VIDPIDS;
    extern const std::vector<DevVIDPID> RTL_SDR_VIDPIDS;

    int getDeviceFD(int& vid, int& pid, const std::vector<DevVIDPID>& allowedVidPids);

    enum class AndroidDocumentPickerStatus : int {
        IDLE = 0,
        PENDING = 1,
        SELECTED = 2,
        CANCELLED = 3,
        ERROR = 4
    };

    bool openDocumentPicker();
    AndroidDocumentPickerStatus getDocumentPickerStatus();
    std::string consumeDocumentPickerResult();

    struct AndroidCompatibilitySnapshot {
        int sdkInt = -1;
        int targetSdk = -1;
        std::string androidRelease = "unknown";
        int displayWidth = 0;
        int displayHeight = 0;
        float density = 0.0f;
        int insetLeft = 0;
        int insetTop = 0;
        int insetRight = 0;
        int insetBottom = 0;
        int inputSurfaceWidth = 0;
        int inputSurfaceHeight = 0;
        int drawableWidth = 0;
        int drawableHeight = 0;
        float lastTouchX = -1.0f;
        float lastTouchY = -1.0f;
        int lastTouchAction = -1;
        std::string lastActivatedControl = "none";
        std::string requestedModulation = "none";
        std::string activeModulation = "unknown";
        float guiVolume = -1.0f;
        float dspVolumeGain = -1.0f;
        std::string audioBackend = "unknown";
        int audioBackendResult = 0;
        std::string documentPickerStatus = "idle";
        std::uint64_t importedByteCount = 0;
        std::string lastImportError;
    };

    void refreshAndroidCompatibilityInfo();
    AndroidCompatibilitySnapshot getAndroidCompatibilitySnapshot();
    void setAndroidCompatibilityControl(const std::string& control);
    void setAndroidModulationDiagnostic(const std::string& requested, const std::string& active);
    void setAndroidVolumeDiagnostic(float guiVolume, float dspVolumeGain);
    void setAndroidAudioBackendDiagnostic(const std::string& backendName, int result);
    void setAndroidDocumentPickerDiagnostic(const std::string& status, std::uint64_t importedBytes, const std::string& error);
    std::uint64_t getDocumentPickerImportedBytes();
}
