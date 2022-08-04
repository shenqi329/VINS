#include <jni.h>
#include <string>
#include <android/log.h>
#include <time.h>
#include <pthread.h>
#include <condition_variable> // std::condition_variable con
#include <android/log.h>
#include <streambuf>
#include <opencv2/opencv.hpp>

#include "MagicPenMaLiang.h"

#define LOG_TAG "magicpenlib.cpp"

// debug logging
#define LOGI(...)  __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define LOGE(...)  __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)

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
MyStreamBuf g_MyStreamBuf;

MagicPenMaLiang magicPenMaLiang;

extern "C"
JNIEXPORT void JNICALL
Java_com_example_vins_MagicPenJNI_init(JNIEnv *env, jclass instance) {

    magicPenMaLiang.Init();

}

extern "C"
JNIEXPORT void JNICALL
Java_com_example_vins_MagicPenJNI_setEdgeImageByte(JNIEnv *env, jclass type, jbyteArray bytesArray) {

    uint8_t*  ptr = reinterpret_cast<uint8_t *>(env->GetByteArrayElements(bytesArray,NULL));
    int length = env->GetArrayLength(bytesArray);

    std::vector<uchar> bytes(ptr, ptr + length);

    magicPenMaLiang.setEdgeImageByte(bytes);
}

extern "C"
JNIEXPORT void JNICALL
Java_com_example_vins_MagicPenJNI_onImageAvailable(JNIEnv *env, jclass type,
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

    g_MyStreamBuf.pubsync();

    uint8_t *srcLumaPtr = nullptr;
    if (isGetData) {
        //camera
        srcLumaPtr = reinterpret_cast<uint8_t *>(env->GetByteArrayElements(byteArray,NULL));
    } else {
        //camrea2
        srcLumaPtr = reinterpret_cast<uint8_t *>(env->GetDirectBufferAddress(bufferY));
    }
    if (srcLumaPtr == nullptr) {
        LOGE("blit NULL pointer ERROR");
        return;
    }

    int rotatedWidth = height; // 320
    int rotatedHeight = width; // 360

    cv::Mat mYuv(height + height / 2, width, CV_8UC1, srcLumaPtr);

    cv::Mat srcBRG(height, width, CV_8UC3);
    cv::Mat rotatedBGR(rotatedHeight, rotatedWidth, CV_8UC3);

    //TS(actual_onImageAvailable);

    // convert YUV to RGBA
    cv::cvtColor(mYuv, srcBRG, cv::COLOR_YUV2BGR_NV21);

    // Rotate 90 degree
    cv::rotate(srcBRG, rotatedBGR, cv::ROTATE_90_CLOCKWISE);

    assert(rotatedRgba.size().width == 320);
    assert(rotatedRgba.size().height == 360);

    bool magicPenSuccess = magicPenMaLiang.Magic(rotatedBGR, 105, 464);
    if (magicPenSuccess) {
        LOGI("magicPenSuccess");
    }
}

extern "C"
JNIEXPORT void JNICALL
Java_com_example_vins_MagicPenJNI_draw(JNIEnv *env, jclass type, jdouble timeStamp) {
    magicPenMaLiang.Draw(timeStamp);
}