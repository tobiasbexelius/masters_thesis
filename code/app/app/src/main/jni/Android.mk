LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := opencv-prebuilt
LOCAL_SRC_FILES = external/OpenCV-android-sdk/sdk/native/libs/$(TARGET_ARCH_ABI)/libopencv_java.so
LOCAL_EXPORT_C_INCLUDES := External/OpenCV-android-sdk/sdk/native/jni/include
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)

LOCAL_MODULE    := com_apm_tobias_automaticpackagemeasuring_core_PackageMeasurer
LOCAL_SRC_FILES := $(wildcard $(LOCAL_PATH)/apm_core/lib/src/*.cpp)
LOCAL_SRC_FILES += apm_jni.cpp
LOCAL_CFLAGS := -std=gnu++11
LOCAL_C_INCLUDES += $(LOCAL_PATH)
LOCAL_LDLIBS     += -llog -ldl
LOCAL_SHARED_LIBRARIES := opencv-prebuilt gnustl_shared

include $(BUILD_SHARED_LIBRARY)

