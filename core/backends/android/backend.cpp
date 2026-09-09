#include <backend.h>log
#include "android_backend.h"
#include <core.h>
#include <gui/gui.h>
#include "imgui.h"
#include "imgui_impl_android.h"
#include "imgui_impl_opengl3.h"
#include <android/log.h>
#include <android/input.h>
#include <android_native_app_glue.h>
#include <android/asset_manager.h>
#include <EGL/egl.h>
#include <GLES3/gl3.h>
#include <jni.h>
#include <stdint.h>
#include <gui/icons.h>
#include <gui/style.h>
#include <gui/menus/theme.h>
#include <algorithm>
#include <filesystem>
#include <mutex>

// Credit to the ImGui android OpenGL3 example for a lot of this code!

namespace backend {
    struct android_app* app = NULL;
    EGLDisplay _EglDisplay = EGL_NO_DISPLAY;
    EGLSurface _EglSurface = EGL_NO_SURFACE;
    EGLContext _EglContext = EGL_NO_CONTEXT;
    bool _Initialized = false;
    char _LogTag[] = "SDR++ CE";
    bool initialized = false;
    bool pauseRendering = false;
    bool exited = false;

    // Forward declaration
    int ShowSoftKeyboardInput();
    int PollUnicodeChars();

    namespace {
        std::mutex androidCompatibilityMutex;
        AndroidCompatibilitySnapshot androidCompatibility;
        int lastDocumentPickerStatus = -1;

        JNIEnv* acquireJavaEnvironment(bool& detachWhenDone) {
            detachWhenDone = false;
            if (!app || !app->activity || !app->activity->vm) { return nullptr; }

            JNIEnv* env = nullptr;
            JavaVM* javaVm = app->activity->vm;
            jint result = javaVm->GetEnv((void**)&env, JNI_VERSION_1_6);
            if (result == JNI_OK) { return env; }
            if (result != JNI_EDETACHED) { return nullptr; }
            if (javaVm->AttachCurrentThread(&env, nullptr) != JNI_OK) { return nullptr; }

            detachWhenDone = true;
            return env;
        }

        void releaseJavaEnvironment(bool detachWhenDone) {
            if (detachWhenDone && app && app->activity && app->activity->vm) {
                app->activity->vm->DetachCurrentThread();
            }
        }

        bool clearJavaException(JNIEnv* env) {
            if (!env || !env->ExceptionCheck()) { return false; }
            env->ExceptionClear();
            return true;
        }

        bool callIntMethod(JNIEnv* env, jclass activityClass, const char* name, int& value) {
            jmethodID method = env->GetMethodID(activityClass, name, "()I");
            if (!method || clearJavaException(env)) { return false; }
            jint result = env->CallIntMethod(app->activity->clazz, method);
            if (clearJavaException(env)) { return false; }
            value = result;
            return true;
        }

        bool callFloatMethod(JNIEnv* env, jclass activityClass, const char* name, float& value) {
            jmethodID method = env->GetMethodID(activityClass, name, "()F");
            if (!method || clearJavaException(env)) { return false; }
            jfloat result = env->CallFloatMethod(app->activity->clazz, method);
            if (clearJavaException(env)) { return false; }
            value = result;
            return true;
        }

        bool callStringMethod(JNIEnv* env, jclass activityClass, const char* name, std::string& value) {
            jmethodID method = env->GetMethodID(activityClass, name, "()Ljava/lang/String;");
            if (!method || clearJavaException(env)) { return false; }
            jstring result = (jstring)env->CallObjectMethod(app->activity->clazz, method);
            if (!result || clearJavaException(env)) { return false; }
            const char* chars = env->GetStringUTFChars(result, nullptr);
            if (!chars || clearJavaException(env)) {
                env->DeleteLocalRef(result);
                return false;
            }
            value = chars;
            env->ReleaseStringUTFChars(result, chars);
            env->DeleteLocalRef(result);
            return true;
        }

        const char* documentPickerStatusName(AndroidDocumentPickerStatus status) {
            switch (status) {
            case AndroidDocumentPickerStatus::IDLE: return "idle";
            case AndroidDocumentPickerStatus::PENDING: return "pending";
            case AndroidDocumentPickerStatus::SELECTED: return "selected";
            case AndroidDocumentPickerStatus::CANCELLED: return "cancelled";
            case AndroidDocumentPickerStatus::ERROR: return "error";
            }
            return "invalid";
        }
    }

    void refreshAndroidCompatibilityInfo() {
        bool detachWhenDone = false;
        JNIEnv* env = acquireJavaEnvironment(detachWhenDone);
        if (!env) { return; }

        AndroidCompatibilitySnapshot updated;
        {
            std::lock_guard<std::mutex> lock(androidCompatibilityMutex);
            updated = androidCompatibility;
        }

        jclass activityClass = env->GetObjectClass(app->activity->clazz);
        if (activityClass) {
            callIntMethod(env, activityClass, "getAndroidSdkInt", updated.sdkInt);
            callIntMethod(env, activityClass, "getTargetSdkVersionValue", updated.targetSdk);
            callStringMethod(env, activityClass, "getAndroidReleaseValue", updated.androidRelease);
            callIntMethod(env, activityClass, "getDisplayWidthValue", updated.displayWidth);
            callIntMethod(env, activityClass, "getDisplayHeightValue", updated.displayHeight);
            callFloatMethod(env, activityClass, "getDisplayDensityValue", updated.density);
            callIntMethod(env, activityClass, "getInsetLeftValue", updated.insetLeft);
            callIntMethod(env, activityClass, "getInsetTopValue", updated.insetTop);
            callIntMethod(env, activityClass, "getInsetRightValue", updated.insetRight);
            callIntMethod(env, activityClass, "getInsetBottomValue", updated.insetBottom);
            env->DeleteLocalRef(activityClass);
        }
        clearJavaException(env);
        releaseJavaEnvironment(detachWhenDone);

        std::lock_guard<std::mutex> lock(androidCompatibilityMutex);
        androidCompatibility = updated;
    }

    AndroidCompatibilitySnapshot getAndroidCompatibilitySnapshot() {
        std::lock_guard<std::mutex> lock(androidCompatibilityMutex);
        return androidCompatibility;
    }

    void setAndroidCompatibilityControl(const std::string& control) {
        std::lock_guard<std::mutex> lock(androidCompatibilityMutex);
        androidCompatibility.lastActivatedControl = control;
    }

    void setAndroidModulationDiagnostic(const std::string& requested, const std::string& active) {
        std::lock_guard<std::mutex> lock(androidCompatibilityMutex);
        androidCompatibility.requestedModulation = requested;
        androidCompatibility.activeModulation = active;
    }

    void setAndroidVolumeDiagnostic(float guiVolume, float dspVolumeGain) {
        std::lock_guard<std::mutex> lock(androidCompatibilityMutex);
        androidCompatibility.guiVolume = guiVolume;
        androidCompatibility.dspVolumeGain = dspVolumeGain;
    }

    void setAndroidAudioBackendDiagnostic(const std::string& backendName, int result) {
        std::lock_guard<std::mutex> lock(androidCompatibilityMutex);
        androidCompatibility.audioBackend = backendName;
        androidCompatibility.audioBackendResult = result;
    }

    void setAndroidDocumentPickerDiagnostic(const std::string& status, std::uint64_t importedBytes, const std::string& error) {
        std::lock_guard<std::mutex> lock(androidCompatibilityMutex);
        androidCompatibility.documentPickerStatus = status;
        androidCompatibility.importedByteCount = importedBytes;
        androidCompatibility.lastImportError = error;
    }

    void doPartialInit() {
        std::string root = (std::string)core::args["root"];
        backend::init();
        style::loadFonts(root + "/res"); // TODO: Don't hardcode, use config
        icons::load(root + "/res");
        thememenu::applyTheme();
        ImGui::GetStyle().ScaleAllSizes(style::uiScale);
        gui::mainWindow.setFirstMenuRender();
    }

    void handleAppCmd(struct android_app* app, int32_t appCmd) {
        switch (appCmd) {
        case APP_CMD_SAVE_STATE:
            flog::warn("APP_CMD_SAVE_STATE");
            break;
        case APP_CMD_INIT_WINDOW:
            flog::warn("APP_CMD_INIT_WINDOW");
            if (pauseRendering && !exited) {
                doPartialInit();
                pauseRendering = false;
            }
            exited = false;
            break;
        case APP_CMD_TERM_WINDOW:
            flog::warn("APP_CMD_TERM_WINDOW");
            pauseRendering = true;
            backend::end();
            break;
        case APP_CMD_GAINED_FOCUS:
            flog::warn("APP_CMD_GAINED_FOCUS");
            if (ImGui::GetCurrentContext()) { ImGui::GetIO().AddFocusEvent(true); }
            refreshAndroidCompatibilityInfo();
            break;
        case APP_CMD_LOST_FOCUS:
            flog::warn("APP_CMD_LOST_FOCUS");
            if (ImGui::GetCurrentContext()) {
                ImGui::GetIO().AddMouseButtonEvent(0, false);
                ImGui::GetIO().AddFocusEvent(false);
            }
            break;
        }
    }

    int32_t handleInputEvent(struct android_app* app, AInputEvent* inputEvent) {
        if (AInputEvent_getType(inputEvent) == AINPUT_EVENT_TYPE_MOTION) {
            int32_t rawAction = AMotionEvent_getAction(inputEvent);
            int32_t action = rawAction & AMOTION_EVENT_ACTION_MASK;
            std::size_t pointerCount = AMotionEvent_getPointerCount(inputEvent);
            std::size_t pointerIndex = (rawAction & AMOTION_EVENT_ACTION_POINTER_INDEX_MASK) >> AMOTION_EVENT_ACTION_POINTER_INDEX_SHIFT;
            float x = -1.0f;
            float y = -1.0f;
            if (pointerCount > 0) {
                pointerIndex = std::min(pointerIndex, pointerCount - 1);
                x = AMotionEvent_getX(inputEvent, pointerIndex);
                y = AMotionEvent_getY(inputEvent, pointerIndex);
            }
            {
                std::lock_guard<std::mutex> lock(androidCompatibilityMutex);
                androidCompatibility.lastTouchX = x;
                androidCompatibility.lastTouchY = y;
                androidCompatibility.lastTouchAction = action;
            }
            if (action == AMOTION_EVENT_ACTION_DOWN) {
                flog::info("SDRPP_ANDROID15: touch down x={0:.1f} y={1:.1f} pointers={2}", x, y, pointerCount);
            }
            else if (action == AMOTION_EVENT_ACTION_CANCEL) {
                flog::info("SDRPP_ANDROID15: touch cancelled x={0:.1f} y={1:.1f}", x, y);
            }
        }
        return ImGui_ImplAndroid_HandleInputEvent(inputEvent);
    }

    int aquireWindow() {
        while (!app->window) {
            flog::warn("Waiting on the shitty window thing"); std::this_thread::sleep_for(std::chrono::milliseconds(30));
            int out_events;
            struct android_poll_source* out_data;

            while (ALooper_pollAll(0, NULL, &out_events, (void**)&out_data) >= 0) {
                // Process one event
                if (out_data != NULL) { out_data->process(app, out_data); }

                // Exit the app by returning from within the infinite loop
                if (app->destroyRequested != 0) {
                    return -1;
                }
            }
        }
        ANativeWindow_acquire(app->window);
        return 0;
    }

    int init(std::string resDir) {
        flog::warn("Backend init");

        // Get window
        aquireWindow();

        // EGL Init
        {
            _EglDisplay = eglGetDisplay(EGL_DEFAULT_DISPLAY);
            if (_EglDisplay == EGL_NO_DISPLAY)
                __android_log_print(ANDROID_LOG_ERROR, _LogTag, "%s", "eglGetDisplay(EGL_DEFAULT_DISPLAY) returned EGL_NO_DISPLAY");

            if (eglInitialize(_EglDisplay, 0, 0) != EGL_TRUE)
                __android_log_print(ANDROID_LOG_ERROR, _LogTag, "%s", "eglInitialize() returned with an error");

            const EGLint egl_attributes[] = { EGL_BLUE_SIZE, 8, EGL_GREEN_SIZE, 8, EGL_RED_SIZE, 8, EGL_DEPTH_SIZE, 24, EGL_SURFACE_TYPE, EGL_WINDOW_BIT, EGL_NONE };
            EGLint num_configs = 0;
            if (eglChooseConfig(_EglDisplay, egl_attributes, nullptr, 0, &num_configs) != EGL_TRUE)
                __android_log_print(ANDROID_LOG_ERROR, _LogTag, "%s", "eglChooseConfig() returned with an error");
            if (num_configs == 0)
                __android_log_print(ANDROID_LOG_ERROR, _LogTag, "%s", "eglChooseConfig() returned 0 matching config");

            // Get the first matching config
            EGLConfig egl_config;
            eglChooseConfig(_EglDisplay, egl_attributes, &egl_config, 1, &num_configs);
            EGLint egl_format;
            eglGetConfigAttrib(_EglDisplay, egl_config, EGL_NATIVE_VISUAL_ID, &egl_format);
            ANativeWindow_setBuffersGeometry(app->window, 0, 0, egl_format);

            const EGLint egl_context_attributes[] = { EGL_CONTEXT_CLIENT_VERSION, 3, EGL_NONE };
            _EglContext = eglCreateContext(_EglDisplay, egl_config, EGL_NO_CONTEXT, egl_context_attributes);

            if (_EglContext == EGL_NO_CONTEXT)
                __android_log_print(ANDROID_LOG_ERROR, _LogTag, "%s", "eglCreateContext() returned EGL_NO_CONTEXT");

            _EglSurface = eglCreateWindowSurface(_EglDisplay, egl_config, app->window, NULL);
            eglMakeCurrent(_EglDisplay, _EglSurface, _EglSurface, _EglContext);
        }

        // Setup Dear ImGui context
        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        ImGuiIO& io = ImGui::GetIO();
        (void)io;

        // Disable loading/saving of .ini file from disk.
        // FIXME: Consider using LoadIniSettingsFromMemory() / SaveIniSettingsToMemory() to save in appropriate location for Android.
        io.IniFilename = NULL;

        // Setup Platform/Renderer backends
        ImGui_ImplAndroid_Init(app->window);
        ImGui_ImplOpenGL3_Init("#version 300 es");

        refreshAndroidCompatibilityInfo();
        AndroidCompatibilitySnapshot compatibility = getAndroidCompatibilitySnapshot();
        flog::info("SDRPP_ANDROID15: native startup api={0} release={1} targetSdk={2} display={3}x{4} density={5:.2f} insets={6},{7},{8},{9}",
            compatibility.sdkInt, compatibility.androidRelease, compatibility.targetSdk,
            compatibility.displayWidth, compatibility.displayHeight, compatibility.density,
            compatibility.insetLeft, compatibility.insetTop, compatibility.insetRight, compatibility.insetBottom);

        return 0;
    }

    void beginFrame() {
        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplAndroid_NewFrame();
        int drawableWidth = 0;
        int drawableHeight = 0;
        if (_EglDisplay != EGL_NO_DISPLAY && _EglSurface != EGL_NO_SURFACE) {
            eglQuerySurface(_EglDisplay, _EglSurface, EGL_WIDTH, &drawableWidth);
            eglQuerySurface(_EglDisplay, _EglSurface, EGL_HEIGHT, &drawableHeight);
        }
        {
            std::lock_guard<std::mutex> lock(androidCompatibilityMutex);
            androidCompatibility.inputSurfaceWidth = app && app->window ? ANativeWindow_getWidth(app->window) : 0;
            androidCompatibility.inputSurfaceHeight = app && app->window ? ANativeWindow_getHeight(app->window) : 0;
            androidCompatibility.drawableWidth = drawableWidth;
            androidCompatibility.drawableHeight = drawableHeight;
        }
        ImGui::NewFrame();
    }

    void render(bool vsync) {
        // Rendering
        ImGui::Render();
        auto dSize = ImGui::GetIO().DisplaySize;
        glViewport(0, 0, dSize.x, dSize.y);
        glClearColor(gui::themeManager.clearColor.x, gui::themeManager.clearColor.y, gui::themeManager.clearColor.z, gui::themeManager.clearColor.w);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        eglSwapBuffers(_EglDisplay, _EglSurface);
    }

    // No screen pos to detect
    void getMouseScreenPos(double& x, double& y) { x = 0; y = 0; }
    void setMouseScreenPos(double x, double y) {}

    int renderLoop() {
        while (true) {
            int out_events;
            struct android_poll_source* out_data;

            while (ALooper_pollAll(0, NULL, &out_events, (void**)&out_data) >= 0) {
                // Process one event
                if (out_data != NULL) { out_data->process(app, out_data); }

                // Exit the app by returning from within the infinite loop
                if (app->destroyRequested != 0) {
                    flog::warn("ASKED TO EXIT");
                    exited = true;

                    // Stop SDR
                    gui::mainWindow.setPlayState(false);
                    return 0;
                }
            }

            if (_EglDisplay == EGL_NO_DISPLAY) { continue; }

            if (!pauseRendering) {
                // Initiate a new frame
                ImGuiIO& io = ImGui::GetIO();
                auto dsize = io.DisplaySize;

                // Poll Unicode characters via JNI
                // FIXME: do not call this every frame because of JNI overhead
                PollUnicodeChars();

                // Open on-screen (soft) input if requested by Dear ImGui
                static bool WantTextInputLast = false;
                if (io.WantTextInput && !WantTextInputLast)
                ShowSoftKeyboardInput();
                WantTextInputLast = io.WantTextInput;

                // Render
                beginFrame();
                
                if (dsize.x > 0 && dsize.y > 0) {
                    ImGui::SetNextWindowPos(ImVec2(0, 0));
                    ImGui::SetNextWindowSize(ImVec2(dsize.x, dsize.y));
                    gui::mainWindow.draw();
                }
                render();
            }
            else {
                std::this_thread::sleep_for(std::chrono::milliseconds(30));
            }
        }

        return 0;
    }

    int end() {
        // Cleanup
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplAndroid_Shutdown();
        ImGui::DestroyContext();

        // Destroy all
        if (_EglDisplay != EGL_NO_DISPLAY) {
            eglMakeCurrent(_EglDisplay, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
            if (_EglContext != EGL_NO_CONTEXT) { eglDestroyContext(_EglDisplay, _EglContext); }
            if (_EglSurface != EGL_NO_SURFACE) { eglDestroySurface(_EglDisplay, _EglSurface); }
            eglTerminate(_EglDisplay);
        }

        _EglDisplay = EGL_NO_DISPLAY;
        _EglContext = EGL_NO_CONTEXT;
        _EglSurface = EGL_NO_SURFACE;

        if (app->window) { ANativeWindow_release(app->window); }

        return 0;
    }

    int ShowSoftKeyboardInput() {
        JavaVM* java_vm = app->activity->vm;
        JNIEnv* java_env = NULL;

        jint jni_return = java_vm->GetEnv((void**)&java_env, JNI_VERSION_1_6);
        if (jni_return == JNI_ERR)
            return -1;

        jni_return = java_vm->AttachCurrentThread(&java_env, NULL);
        if (jni_return != JNI_OK)
            return -2;

        jclass native_activity_clazz = java_env->GetObjectClass(app->activity->clazz);
        if (native_activity_clazz == NULL)
            return -3;

        jmethodID method_id = java_env->GetMethodID(native_activity_clazz, "showSoftInput", "()V");
        if (method_id == NULL)
            return -4;

        java_env->CallVoidMethod(app->activity->clazz, method_id);

        jni_return = java_vm->DetachCurrentThread();
        if (jni_return != JNI_OK)
            return -5;

        return 0;
    }

    int getDeviceFD(int& vid, int& pid, const std::vector<DevVIDPID>& allowedVidPids) {
        JavaVM* java_vm = app->activity->vm;
        JNIEnv* java_env = NULL;

        jint jni_return = java_vm->GetEnv((void**)&java_env, JNI_VERSION_1_6);
        if (jni_return == JNI_ERR)
            return -1;

        jni_return = java_vm->AttachCurrentThread(&java_env, NULL);
        if (jni_return != JNI_OK)
            return -1;

        jclass native_activity_clazz = java_env->GetObjectClass(app->activity->clazz);
        if (native_activity_clazz == NULL)
            return -1;

        jfieldID fd_field_id = java_env->GetFieldID(native_activity_clazz, "SDR_FD", "I");
        jfieldID vid_field_id = java_env->GetFieldID(native_activity_clazz, "SDR_VID", "I");
        jfieldID pid_field_id = java_env->GetFieldID(native_activity_clazz, "SDR_PID", "I");
        
        if (!vid_field_id || !vid_field_id || !pid_field_id)
            return -1;

        int fd = java_env->GetIntField(app->activity->clazz, fd_field_id);
        vid = java_env->GetIntField(app->activity->clazz, vid_field_id);
        pid = java_env->GetIntField(app->activity->clazz, pid_field_id);

        jni_return = java_vm->DetachCurrentThread();
        if (jni_return != JNI_OK)
            return -1;

        // If no vid/pid was given, just return successfully
        if (allowedVidPids.empty()) {
            return fd;
        }

        // Otherwise, check that the vid/pid combo is allowed
        for (auto const& vp : allowedVidPids) {
            if (vp.vid != vid || vp.pid != pid) { continue; }
            return fd;
        }

        return -1;
    }

    // Unfortunately, the native KeyEvent implementation has no getUnicodeChar() function.
    // Therefore, we implement the processing of KeyEvents in MainActivity.kt and poll
    // the resulting Unicode characters here via JNI and send them to Dear ImGui.
    int PollUnicodeChars() {
        JavaVM* java_vm = app->activity->vm;
        JNIEnv* java_env = NULL;

        jint jni_return = java_vm->GetEnv((void**)&java_env, JNI_VERSION_1_6);
        if (jni_return == JNI_ERR)
            return -1;

        jni_return = java_vm->AttachCurrentThread(&java_env, NULL);
        if (jni_return != JNI_OK)
            return -2;

        jclass native_activity_clazz = java_env->GetObjectClass(app->activity->clazz);
        if (native_activity_clazz == NULL)
            return -3;

        jmethodID method_id = java_env->GetMethodID(native_activity_clazz, "pollUnicodeChar", "()I");
        if (method_id == NULL)
            return -4;

        // Send the actual characters to Dear ImGui
        ImGuiIO& io = ImGui::GetIO();
        jint unicode_character;
        while ((unicode_character = java_env->CallIntMethod(app->activity->clazz, method_id)) != 0)
            io.AddInputCharacter(unicode_character);

        jni_return = java_vm->DetachCurrentThread();
        if (jni_return != JNI_OK)
            return -5;

        return 0;
    }

    bool openDocumentPicker() {
        flog::info("SDRPP_ANDROID15: native openDocumentPicker JNI request");
        bool detachWhenDone = false;
        JNIEnv* env = acquireJavaEnvironment(detachWhenDone);
        if (!env) {
            setAndroidDocumentPickerDiagnostic("JNI environment unavailable", 0, "Could not acquire JNI environment");
            return false;
        }

        bool opened = false;
        jclass activityClass = env->GetObjectClass(app->activity->clazz);
        if (activityClass) {
            jmethodID method = env->GetMethodID(activityClass, "openDocumentPicker", "()Z");
            if (method) {
                opened = env->CallBooleanMethod(app->activity->clazz, method) == JNI_TRUE;
            }
            env->DeleteLocalRef(activityClass);
        }

        if (clearJavaException(env)) { opened = false; }
        releaseJavaEnvironment(detachWhenDone);
        flog::info("SDRPP_ANDROID15: native openDocumentPicker returned {0}", opened);
        if (!opened) {
            setAndroidDocumentPickerDiagnostic("picker start failed", 0, "Kotlin openDocumentPicker returned false");
        }
        return opened;
    }

    AndroidDocumentPickerStatus getDocumentPickerStatus() {
        bool detachWhenDone = false;
        JNIEnv* env = acquireJavaEnvironment(detachWhenDone);
        if (!env) { return AndroidDocumentPickerStatus::ERROR; }

        jint status = (jint)AndroidDocumentPickerStatus::ERROR;
        jclass activityClass = env->GetObjectClass(app->activity->clazz);
        if (activityClass) {
            jmethodID method = env->GetMethodID(activityClass, "getDocumentPickerStatus", "()I");
            if (method) {
                status = env->CallIntMethod(app->activity->clazz, method);
            }
            env->DeleteLocalRef(activityClass);
        }

        if (clearJavaException(env)) {
            status = (jint)AndroidDocumentPickerStatus::ERROR;
        }
        releaseJavaEnvironment(detachWhenDone);

        if (status < (jint)AndroidDocumentPickerStatus::IDLE ||
            status > (jint)AndroidDocumentPickerStatus::ERROR) {
            return AndroidDocumentPickerStatus::ERROR;
        }
        AndroidDocumentPickerStatus pickerStatus = (AndroidDocumentPickerStatus)status;
        bool statusChanged = false;
        {
            std::lock_guard<std::mutex> lock(androidCompatibilityMutex);
            statusChanged = lastDocumentPickerStatus != status;
            lastDocumentPickerStatus = status;
            androidCompatibility.documentPickerStatus = documentPickerStatusName(pickerStatus);
        }
        if (statusChanged) {
            flog::info("SDRPP_ANDROID15: native document picker status={0}", documentPickerStatusName(pickerStatus));
        }
        return pickerStatus;
    }

    std::uint64_t getDocumentPickerImportedBytes() {
        bool detachWhenDone = false;
        JNIEnv* env = acquireJavaEnvironment(detachWhenDone);
        if (!env) { return 0; }

        jlong importedBytes = 0;
        jclass activityClass = env->GetObjectClass(app->activity->clazz);
        if (activityClass) {
            jmethodID method = env->GetMethodID(activityClass, "getDocumentPickerImportedBytes", "()J");
            if (method) {
                importedBytes = env->CallLongMethod(app->activity->clazz, method);
            }
            env->DeleteLocalRef(activityClass);
        }
        if (clearJavaException(env)) { importedBytes = 0; }
        releaseJavaEnvironment(detachWhenDone);
        return importedBytes > 0 ? (std::uint64_t)importedBytes : 0;
    }

    std::string consumeDocumentPickerResult() {
        bool detachWhenDone = false;
        JNIEnv* env = acquireJavaEnvironment(detachWhenDone);
        if (!env) { return ""; }

        std::string result;
        jclass activityClass = env->GetObjectClass(app->activity->clazz);
        if (activityClass) {
            jmethodID method = env->GetMethodID(activityClass, "consumeDocumentPickerResult", "()Ljava/lang/String;");
            if (method) {
                jstring javaResult = (jstring)env->CallObjectMethod(app->activity->clazz, method);
                if (javaResult && !env->ExceptionCheck()) {
                    const char* chars = env->GetStringUTFChars(javaResult, nullptr);
                    if (chars) {
                        result = chars;
                        env->ReleaseStringUTFChars(javaResult, chars);
                    }
                    env->DeleteLocalRef(javaResult);
                }
            }
            env->DeleteLocalRef(activityClass);
        }

        if (clearJavaException(env)) { result.clear(); }
        releaseJavaEnvironment(detachWhenDone);
        flog::info("SDRPP_ANDROID15: native consumed document picker result path='{0}'", result);
        return result;
    }

    std::string getAppFilesDir() {
        JavaVM* java_vm = app->activity->vm;
        JNIEnv* java_env = NULL;

        jint jni_return = java_vm->GetEnv((void**)&java_env, JNI_VERSION_1_6);
        if (jni_return == JNI_ERR)
            throw std::runtime_error("Could not get JNI environment");

        jni_return = java_vm->AttachCurrentThread(&java_env, NULL);
        if (jni_return != JNI_OK)
            throw std::runtime_error("Could not attach to thread");

        jclass native_activity_clazz = java_env->GetObjectClass(app->activity->clazz);
        if (native_activity_clazz == NULL)
            throw std::runtime_error("Could not get MainActivity class");

        jmethodID method_id = java_env->GetMethodID(native_activity_clazz, "getAppDir", "()Ljava/lang/String;");
        if (method_id == NULL)
            throw std::runtime_error("Could not get method ID");

        jstring jstr = (jstring)java_env->CallObjectMethod(app->activity->clazz, method_id);

        const char* _str = java_env->GetStringUTFChars(jstr, NULL);
        std::string str(_str);
        java_env->ReleaseStringUTFChars(jstr, _str);

        jni_return = java_vm->DetachCurrentThread();
        if (jni_return != JNI_OK)
            throw std::runtime_error("Could not detach from thread");

        
        return str;
    }

    const std::vector<DevVIDPID> AIRSPY_VIDPIDS = {
        { 0x1d50, 0x60a1 }
    };

    const std::vector<DevVIDPID> AIRSPYHF_VIDPIDS = {
        { 0x03EB, 0x800C }
    };

    const std::vector<DevVIDPID> HACKRF_VIDPIDS = {
        { 0x1d50, 0x604b },
        { 0x1d50, 0x6089 },
        { 0x1d50, 0xcc15 }
    };

    const std::vector<DevVIDPID> RTL_SDR_VIDPIDS = {
        { 0x0bda, 0x2832 },
        { 0x0bda, 0x2838 },
        { 0x0413, 0x6680 },
        { 0x0413, 0x6f0f },
        { 0x0458, 0x707f },
        { 0x0ccd, 0x00a9 },
        { 0x0ccd, 0x00b3 },
        { 0x0ccd, 0x00b4 },
        { 0x0ccd, 0x00b5 },
        { 0x0ccd, 0x00b7 },
        { 0x0ccd, 0x00b8 },
        { 0x0ccd, 0x00b9 },
        { 0x0ccd, 0x00c0 },
        { 0x0ccd, 0x00c6 },
        { 0x0ccd, 0x00d3 },
        { 0x0ccd, 0x00d7 },
        { 0x0ccd, 0x00e0 },
        { 0x1554, 0x5020 },
        { 0x15f4, 0x0131 },
        { 0x15f4, 0x0133 },
        { 0x185b, 0x0620 },
        { 0x185b, 0x0650 },
        { 0x185b, 0x0680 },
        { 0x1b80, 0xd393 },
        { 0x1b80, 0xd394 },
        { 0x1b80, 0xd395 },
        { 0x1b80, 0xd397 },
        { 0x1b80, 0xd398 },
        { 0x1b80, 0xd39d },
        { 0x1b80, 0xd3a4 },
        { 0x1b80, 0xd3a8 },
        { 0x1b80, 0xd3af },
        { 0x1b80, 0xd3b0 },
        { 0x1d19, 0x1101 },
        { 0x1d19, 0x1102 },
        { 0x1d19, 0x1103 },
        { 0x1d19, 0x1104 },
        { 0x1f4d, 0xa803 },
        { 0x1f4d, 0xb803 },
        { 0x1f4d, 0xc803 },
        { 0x1f4d, 0xd286 },
        { 0x1f4d, 0xd803 }
    };
}

extern "C" {
    void android_main(struct android_app* app) {
        // Save app instance
        app->onAppCmd = backend::handleAppCmd;
        app->onInputEvent = backend::handleInputEvent;
        backend::app = app;

        // Check if this is the first time we run or not
        if (backend::initialized) {
            flog::warn("android_main called again");
            backend::doPartialInit();
            backend::pauseRendering = false;
            backend::renderLoop();
            return;
        }
        backend::initialized = true;

        // Grab files dir
        std::string appdir = backend::getAppFilesDir();

        // Call main
        char* rootpath = new char[appdir.size() + 1];
        strcpy(rootpath, appdir.c_str());
        char* dummy[] = { "", "-r", rootpath };
        sdrpp_main(3, dummy);
    }
}
