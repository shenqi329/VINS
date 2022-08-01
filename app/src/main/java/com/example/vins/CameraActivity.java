package com.example.vins;

import android.app.Activity;
import android.content.pm.PackageInfo;
import android.content.pm.PackageManager;
import android.graphics.SurfaceTexture;
import android.hardware.Camera;
import android.opengl.GLES11Ext;
import android.opengl.GLSurfaceView;
import android.os.Bundle;
import android.os.Environment;
import android.view.Surface;
import android.view.TextureView;
import android.view.View;
import android.widget.CompoundButton;
import android.widget.ImageView;
import android.widget.SeekBar;
import android.widget.Switch;
import android.widget.TextView;
import android.widget.Toast;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.List;

import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;

import static android.opengl.GLSurfaceView.RENDERMODE_WHEN_DIRTY;

import androidx.annotation.NonNull;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

public class CameraActivity extends Activity implements TextureView.SurfaceTextureListener {
    private Camera mCamera;
    private GLSurfaceView mGLSurfaceView;
    private SurfaceTexture mSurfaceTexture;
    private String mPath;

    private final int imageWidth = 640;
    private final int imageHeight = 480;

    // needed for permission request callback
    private static final int PERMISSIONS_REQUEST_CODE = 12345;

    // TextViews
    private TextView tvX;
    private TextView tvY;
    private TextView tvZ;
    private TextView tvTotal;
    private TextView tvLoop;
    private TextView tvFeature;
    private TextView tvBuf;

    // ImageView for initialization instructions
    private ImageView ivInit;
    private ov2slamJNI mVinsJNI;

    private float virtualCamDistance = 4;
    private final float minVirtualCamDistance = 4;
    private final float maxVirtualCamDistance = 40;
    private Surface mSurface;
    private MyRender mRenderer;
    private long mLastExitTime;


    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_camera);

        // first make sure the necessary permissions are given
        checkPermissionsIfNeccessary();

//        mPath = Environment.getExternalStorageDirectory() + File.separator + "VINS" + File.separator + "img_imu_data";
//        File file = new File(mPath);
//        if (!file.exists()) file.mkdirs();
//
//        File file1 = new File(Environment.getExternalStorageDirectory() + File.separator + "VINS" +
//                File.separator + "brief_k10L6.bin");
//        File file2 = new File(Environment.getExternalStorageDirectory() + File.separator + "VINS" +
//                File.separator + "brief_pattern.yml");
//
//        if (!file1.exists()) copyFile(file1);
//        if (!file2.exists()) copyFile(file2);

        initViews();
    }

    private void initViews() {
        tvX = (TextView) findViewById(R.id.x_Label);
        tvY = (TextView) findViewById(R.id.y_Label);
        tvZ = (TextView) findViewById(R.id.z_Label);
        tvTotal = (TextView) findViewById(R.id.total_odom_Label);
        tvLoop = (TextView) findViewById(R.id.loop_Label);
        tvFeature = (TextView) findViewById(R.id.feature_Label);
        tvBuf = (TextView) findViewById(R.id.buf_Label);

        ivInit = (ImageView) findViewById(R.id.init_image_view);
        ivInit.setVisibility(View.VISIBLE);

        mGLSurfaceView = (GLSurfaceView) findViewById(R.id.texture_view);
        mRenderer = new MyRender();
        mGLSurfaceView.setRenderer(mRenderer);
        mGLSurfaceView.setRenderMode(RENDERMODE_WHEN_DIRTY);

        TextureView textureView = (TextureView) findViewById(R.id.surface_view);
        textureView.setSurfaceTextureListener(this);


        // Define the Switch listeners
        Switch arSwitch = (Switch) findViewById(R.id.ar_switch);
        arSwitch.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                mVinsJNI.onARSwitch(isChecked);
            }
        });

        Switch loopSwitch = (Switch) findViewById(R.id.loop_switch);
        loopSwitch.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                mVinsJNI.onLoopSwitch(isChecked);
            }
        });

        SeekBar zoomSlider = (SeekBar) findViewById(R.id.zoom_slider);
        zoomSlider.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                virtualCamDistance = minVirtualCamDistance + ((float) progress / 100) * (maxVirtualCamDistance - minVirtualCamDistance);
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {
            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {
            }
        });
    }

    @Override
    protected void onResume() {
        super.onResume();

        if (mVinsJNI == null) mVinsJNI = new ov2slamJNI();
        mVinsJNI.init();
    }

    @Override
    protected void onPause() {
        super.onPause();
        mVinsJNI.onPause();
    }

    private void copyFile(final File file) {
        try {
            InputStream inputStream = getAssets().open(file.getName());
            FileOutputStream outputStream = new FileOutputStream(file);
            byte[] bytes = new byte[10240];
            int count = 0;
            while ((count = inputStream.read(bytes)) != -1) {
                outputStream.write(bytes, 0, count);
            }

            outputStream.flush();
            inputStream.close();
            outputStream.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }


    @Override
    public void onSurfaceTextureAvailable(SurfaceTexture surface, int width, int height) {
        mSurface = new Surface(surface);
    }

    @Override
    public void onSurfaceTextureSizeChanged(SurfaceTexture surface, int width, int height) {

    }

    @Override
    public boolean onSurfaceTextureDestroyed(SurfaceTexture surface) {
        return false;
    }

    @Override
    public void onSurfaceTextureUpdated(SurfaceTexture surface) {

    }

    @Override
    public void onBackPressed() {
        super.onBackPressed();

        long time = System.currentTimeMillis();
        if (time - mLastExitTime < 2000) {
            android.os.Process.killProcess(android.os.Process.myPid());
            System.exit(0);
        } else {
            Toast.makeText(this, "click again", Toast.LENGTH_SHORT).show();
        }

        mLastExitTime = time;
    }

    public class MyRender implements GLSurfaceView.Renderer, Camera.PreviewCallback, SurfaceTexture.OnFrameAvailableListener {

        @Override
        public void onSurfaceCreated(GL10 gl, EGLConfig config) {
            mSurfaceTexture = new SurfaceTexture(GLES11Ext.GL_SAMPLER_EXTERNAL_OES);

            mCamera = Camera.open();
            if (mCamera != null) {
//            mCamera.setDisplayOrientation(90);
                Camera.Parameters parameters = mCamera.getParameters();
                parameters.setPreviewFrameRate(30);
                parameters.setPreviewSize(imageWidth, imageHeight);
                mCamera.setParameters(parameters);

                try {
                    mCamera.setPreviewTexture(mSurfaceTexture);
                } catch (IOException e) {
                    e.printStackTrace();
                }
                mCamera.setPreviewCallback(this);
                mCamera.startPreview();

            }

            mSurfaceTexture.setOnFrameAvailableListener(this);
        }

        @Override
        public void onFrameAvailable(SurfaceTexture surfaceTexture) {
            mGLSurfaceView.requestRender();
        }

        @Override
        public void onSurfaceChanged(GL10 gl, int width, int height) {
        }


        @Override
        public void onDrawFrame(GL10 gl) {
            mSurfaceTexture.updateTexImage();
        }

        @Override
        public void onPreviewFrame(byte[] data, Camera camera) {

            // pass the current device's screen orientation to the c++ part
            int currentRotation = getWindowManager().getDefaultDisplay().getRotation();
            boolean isScreenRotated = currentRotation != Surface.ROTATION_90;

            if (mSurface != null) mVinsJNI.onImageAvailable(imageWidth, imageHeight,
                    0, null,
                    0, null, null,
                    mSurface, mSurfaceTexture.getTimestamp(), isScreenRotated,
                    virtualCamDistance, data, true);

            // run the updateViewInfo function on the UI Thread so it has permission to modify it
            runOnUiThread(new Runnable() {
                public void run() {
                    mVinsJNI.updateViewInfo(tvX, tvY, tvZ, tvTotal, tvLoop, tvFeature, tvBuf, ivInit);
                }
            });
        }
    }

    /**
     * @return true if permissions where given
     */
    private boolean checkPermissionsIfNeccessary() {
        try {
            PackageInfo info = getPackageManager().getPackageInfo(this.getPackageName(), PackageManager.GET_PERMISSIONS);
            if (info.requestedPermissions != null) {
                List<String> permissionsNotGrantedYet = new ArrayList<>(info.requestedPermissions.length);
                for (String p : info.requestedPermissions) {
                    if (ContextCompat.checkSelfPermission(this, p) != PackageManager.PERMISSION_GRANTED) {
                        permissionsNotGrantedYet.add(p);
                    }
                }
                if(permissionsNotGrantedYet.size() > 0){
                    ActivityCompat.requestPermissions(this, permissionsNotGrantedYet.toArray(new String[permissionsNotGrantedYet.size()]),
                            PERMISSIONS_REQUEST_CODE);
                    return false;
                }
            }
        } catch (PackageManager.NameNotFoundException e) {
            e.printStackTrace();
        }

        return true;
    }

    @Override
    public void onRequestPermissionsResult(int requestCode,
                                           @NonNull String permissions[],
                                           @NonNull int[] grantResults) {

        if(requestCode == PERMISSIONS_REQUEST_CODE) {
            boolean hasAllPermissions = true;
            // If request is cancelled, the result arrays are empty.
            if (grantResults.length == 0)
                hasAllPermissions = false;
            for (int result : grantResults) {
                if (result != PackageManager.PERMISSION_GRANTED)
                    hasAllPermissions = false;
            }

            if(!hasAllPermissions){
                finish();
            }
        }
    }
}
