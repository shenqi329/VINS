#include <jni.h>
#include <string>
#include <android/log.h>
#include <time.h>
#include <pthread.h>
#include <condition_variable> // std::condition_variable con
#include <android/log.h>
#include "ov2slam.hpp"

#define LOG_TAG "ov2slamlib.cpp"

// debug logging
#define LOGI(...)  __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define LOGE(...)  __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)


#include <android/native_window.h>
#include <android/native_window_jni.h>

class MyStreamBuf : public std::streambuf
{
    enum
    {
        BUFFER_SIZE = 4096,
    };

public:
    MyStreamBuf()
    {
        buffer_[BUFFER_SIZE] = '\0';
        setp(buffer_, buffer_ + BUFFER_SIZE - 1);
    }

    ~MyStreamBuf()
    {
        sync();
    }

protected:
    virtual int_type overflow(int_type c)
    {
        if (c != EOF)
        {
            *pptr() = c;
            pbump(1);
        }
        flush_buffer();
        return c;
    }

    virtual int sync()
    {
        flush_buffer();
        return 0;
    }

private:
    int flush_buffer()
    {
        int len = int(pptr() - pbase());
        if (len <= 0)
            return 0;

        if (len <= BUFFER_SIZE)
            buffer_[len] = '\0';

        android_LogPriority t = ANDROID_LOG_INFO;
        __android_log_write(t, "mylog", buffer_);

        pbump(-len);
        return len;
    }

private:
    char buffer_[BUFFER_SIZE + 1];
};


// global variable to viewController 
// because its much less of a hassle 
// than to pass pointers to Java and back
SlamManager *slamManager = nullptr;
MyStreamBuf g_MyStreamBuf;

void * slam_run(SlamManager *slamManager)
{
    slamManager->run();

    return nullptr;
}

extern "C"
JNIEXPORT void JNICALL
Java_com_example_vins_ov2slamJNI_init(JNIEnv *env, jobject instance) {

    if(slamManager) {
        return;
    }

    //std::cout.rdbuf(&g_MyStreamBuf);

    LOGI("Successfully created ov2slam Object");

    std::shared_ptr<SlamParams> pparams;
    const cv::FileStorage fsSettings("/storage/emulated/0/ov2slam/p40.yaml", cv::FileStorage::READ);
    if(!fsSettings.isOpened()) {
        LOGE("Failed to open settings file...");
        return;
    }

    LOGI("Parameters file loaded...");
    pparams.reset( new SlamParams(fsSettings) );
    slamManager = new SlamManager(pparams);

    // Start the SLAM thread
    pthread_t pthread;
    pthread_create(&pthread, NULL, reinterpret_cast<void *(*)(void *)>(slam_run), (void*)slamManager);
}

float timeStampToSec(long timeStamp) {

    long us = timeStamp / 1000;
    float ms = us / 1000.0;
    return ms / 1000;
}

extern "C"
JNIEXPORT void JNICALL
Java_com_example_vins_ov2slamJNI_onImageAvailable(JNIEnv *env, jclass type,
                                                                           jint width, jint height,
                                                                           jint rowStrideY,
                                                                           jobject bufferY,
                                                                           jint rowStrideUV,
                                                                           jobject bufferU,
                                                                           jobject bufferV,
                                                                           jobject surface,
                                                                           jlong timeStamp,
                                                                           jboolean isScreenRotated,
                                                                           jfloat virtualCamDistance,
                                                                           jbyteArray byteArray,
                                                                           jboolean isGetData) {

//    LOGI("Received image with width: %d height: %d", width, height);

    double timeStampSec = timeStampToSec(timeStamp);
    // IMU Meassurements are momentary meassurements.
    // Camera over an interval. so the mid of the interval is chosen as the timestamp
    // Half the maximum exposure time - half senor time delta
    //const double timeStampOffset = 1.0 / 30.0 / 2.0 - 1.0 / 100.0 / 2.0;
    //timeStampSec += timeStampOffset;

    //camera
    jbyte *srcLumaPtr = env->GetByteArrayElements(byteArray,NULL);

    //camrea2
    //uint8_t *srcLumaPtr = reinterpret_cast<uint8_t *>(env->GetDirectBufferAddress(bufferY));

    if (srcLumaPtr == nullptr) {
        LOGE("blit NULL pointer ERROR");
        return;
    }

    cv::Mat mYuv(height + height / 2, width, CV_8UC1, srcLumaPtr);

    ANativeWindow *win = ANativeWindow_fromSurface(env, surface);

    ANativeWindow_acquire(win);
    ANativeWindow_Buffer buf;

    int rotatedWidth = height; // 480
    int rotatedHeight = width; // 640

    if(isScreenRotated) {
        ANativeWindow_setBuffersGeometry(win, height, width, 0);
    } else {
        ANativeWindow_setBuffersGeometry(win, width, height, 0);
    }

    if (int32_t err = ANativeWindow_lock(win, &buf, NULL)) {
        LOGE("ANativeWindow_lock failed with error code %d\n", err);
        ANativeWindow_release(win);
        return;
    }

//    LOGI("buf.stride: %d", buf.stride);

    uint8_t *dstPtr = reinterpret_cast<uint8_t *>(buf.bits);
    cv::Mat dstRgba;
    if(isScreenRotated) {
        dstRgba = cv::Mat(width, buf.stride, CV_8UC4, dstPtr); // TextureView buffer, use stride as width
    } else {
        dstRgba = cv::Mat(height, buf.stride, CV_8UC4, dstPtr); // TextureView buffer, use stride as width
    }
    cv::Mat srcRgba(height, width, CV_8UC4);
    cv::Mat rotatedRgba(rotatedHeight, rotatedWidth, CV_8UC4);

    //TS(actual_onImageAvailable);

    // convert YUV to RGBA
    cv::cvtColor(mYuv, srcRgba, cv::COLOR_YUV2RGBA_NV21);

    // Rotate 90 degree
    cv::rotate(srcRgba, rotatedRgba, cv::ROTATE_90_CLOCKWISE);

    assert(rotatedRgba.size().width == 480);
    assert(rotatedRgba.size().height == 640);

    cv::Mat rotatedMono(rotatedHeight, rotatedWidth, CV_8UC1);
    cv::cvtColor(rotatedRgba, rotatedMono, cv::COLOR_RGBA2GRAY);
    slamManager->addNewMonoImage(timeStampSec, rotatedMono);

    if (isScreenRotated) {
        srcRgba = rotatedRgba.clone();
    } else {
        cv::rotate(rotatedRgba, srcRgba, cv::ROTATE_90_COUNTERCLOCKWISE);
    }
    // copy to TextureView surface
    uchar *dbuf = dstRgba.data;
    uchar *sbuf = srcRgba.data;
    int i;


    for (i = 0; i < srcRgba.rows; i++) {
        dbuf = dstRgba.data + i * buf.stride * 4;
        memcpy(dbuf, sbuf, srcRgba.cols * 4); //TODO: threw a SIGSEGV SEGV_ACCERR once
        sbuf += srcRgba.cols * 4;
    }
    //TE(actual_onImageAvailable);

    ANativeWindow_unlockAndPost(win);
    ANativeWindow_release(win);

}

extern "C"
JNIEXPORT void JNICALL
Java_com_example_vins_ov2slamJNI_onPause(JNIEnv *env, jclass type) {
//    LOGI("Pause triggered, stopping SensorEvents");
//    viewControllerGlobal->imuStopUpdate();
}

// Constants for ImageView visibility coming from Java
const int VISIBLE = 0x00000000;
const int INVISIBLE = 0x00000004;

extern "C"
JNIEXPORT void JNICALL
Java_com_example_vins_ov2slamJNI_updateViewInfo(JNIEnv *env, jclass type,
                                                                         jobject tvX, jobject tvY,
                                                                         jobject tvZ,
                                                                         jobject tvTotal,
                                                                         jobject tvLoop,
                                                                         jobject tvFeature,
                                                                         jobject tvBuf,
                                                                         jobject ivInit) {

    // Get the method handles
//    jclass tvClass = env->FindClass("android/widget/TextView");
//    jmethodID setTextID = env->GetMethodID(tvClass, "setText", "(Ljava/lang/CharSequence;)V");
//
//    jclass ivClass = env->FindClass("android/widget/ImageView");
//    jmethodID setVisibilityID = env->GetMethodID(ivClass, "setVisibility", "(I)V");
//
//    viewControllerGlobal->viewUpdateMutex.lock();
//    if(viewControllerGlobal->tvXText.empty() == false) {
//        env->CallVoidMethod(tvX, setTextID, env->NewStringUTF(viewControllerGlobal->tvXText.c_str()));
//        env->CallVoidMethod(tvY, setTextID, env->NewStringUTF(viewControllerGlobal->tvYText.c_str()));
//        env->CallVoidMethod(tvZ, setTextID, env->NewStringUTF(viewControllerGlobal->tvZText.c_str()));
//        env->CallVoidMethod(tvTotal, setTextID, env->NewStringUTF(viewControllerGlobal->tvTotalText.c_str()));
//        env->CallVoidMethod(tvLoop, setTextID, env->NewStringUTF(viewControllerGlobal->tvLoopText.c_str()));
//        env->CallVoidMethod(tvFeature, setTextID, env->NewStringUTF(viewControllerGlobal->tvFeatureText.c_str()));
//        env->CallVoidMethod(tvBuf, setTextID, env->NewStringUTF(viewControllerGlobal->tvBufText.c_str()));
//
//        jint visibility = INVISIBLE;//viewControllerGlobal->initImageVisible ? VISIBLE : INVISIBLE;
//        env->CallVoidMethod(ivInit, setVisibilityID, visibility);
//    }
//    viewControllerGlobal->viewUpdateMutex.unlock();
}

extern "C"
JNIEXPORT void JNICALL
Java_com_example_vins_ov2slamJNI_onARSwitch(JNIEnv *env, jclass type,
                                                                     jboolean isChecked) {

//    if(viewControllerGlobal)
//        viewControllerGlobal->switchUI(isChecked);
}

extern "C"
JNIEXPORT void JNICALL
Java_com_example_vins_ov2slamJNI_onLoopSwitch(JNIEnv *env, jclass type,
                                                                       jboolean isChecked) {

//    if(viewControllerGlobal)
//        viewControllerGlobal->loopButtonPressed(isChecked);
}