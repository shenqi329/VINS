package com.example.vins;

import android.view.Surface;
import android.widget.ImageView;
import android.widget.TextView;

import java.io.Serializable;
import java.nio.ByteBuffer;

/**
 * JNI Java Part
 */
public class magicpenJNI implements Serializable {

    // Used to load the 'native-lib' library on application startup.
    static { System.loadLibrary("NativeLib"); }
    
    public native void init();
    
    public native void onImageAvailable(int width, int height, int rowStrideY, ByteBuffer bufferY,
                                               int rowStrideUV, ByteBuffer bufferU, ByteBuffer bufferV, 
                                               Surface surface, long timeStamp, boolean isScreenRotated,
                                               float virtualCamDistance,
                                               byte[] bytes, boolean getData);
}
