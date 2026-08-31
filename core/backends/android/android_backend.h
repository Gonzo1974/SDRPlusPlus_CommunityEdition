#pragma once
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
}
