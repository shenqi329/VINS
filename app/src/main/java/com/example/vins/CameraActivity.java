package com.example.vins;

import android.app.Activity;
import android.content.pm.PackageInfo;
import android.content.pm.PackageManager;
import android.graphics.SurfaceTexture;
import android.hardware.Camera;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.os.Message;
import android.util.Log;
import android.util.Size;
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

import static com.jscheng.scamera.util.LogUtil.TAG;

import androidx.annotation.NonNull;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import com.jscheng.scamera.util.CameraUtil;
import com.jscheng.scamera.util.PermisstionUtil;
import com.jscheng.scamera.widget.CameraGLSurfaceView;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.RealMatrix;

public class CameraActivity extends Activity implements Camera.PreviewCallback, CameraGLSurfaceView.CameraGLSurfaceViewCallback {
    
    private final static int REQUEST_CODE = 1;
    private final static int MSG_START_PREVIEW = 1;

    private CameraGLSurfaceView mCameraView;

    private final int imageWidth = 360;
    private final int imageHeight = 320;

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
    private magicpenJNI mVinsJNI;

    long preImageTimeStamp = -1;

    private float virtualCamDistance = 4;
    private final float minVirtualCamDistance = 4;
    private final float maxVirtualCamDistance = 40;
    private long mLastExitTime;

    private Size mPreviewSize;
    private Handler mCameraHanlder;

    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_camera);

        initCameraHandler();

        double[][] R = new double[3][3];
        double[] T = new double[3];

        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                R[i][j] = 0;
            }
        }
        R[0][0] = 1;
        R[1][1] = 1;
        R[2][2] = 1;

        for (int i = 0; i < 3; i++) {
            T[i] = 0;
        }
        T[2] = 5.0f;
        RealMatrix rotation = new Array2DRowRealMatrix(R);
        RealMatrix translation = new Array2DRowRealMatrix(T);

        MatrixState.set_model_view_matrix(rotation, translation);
        MatrixState.set_projection_matrix(160, 180, 160, 180, 320, 360, 0.01f, 100f);

        initViews();
    }

    public void startPreview() {
        if(mPreviewSize != null && requestPermission()) {
            if (CameraUtil.getCamera() == null) {
                CameraUtil.openCamera();
                Log.e(TAG, "openCamera" );
                CameraUtil.setDisplay(mCameraView.getSurfaceTexture());
                CameraUtil.setPreviewCallback(this);
            }
            CameraUtil.startPreview(this, imageWidth, imageHeight);
        }
    }

    private void initViews() {
        tvX = (TextView) findViewById(R.id.x_Label);
        tvY = (TextView) findViewById(R.id.y_Label);
        tvZ = (TextView) findViewById(R.id.z_Label);

        mCameraView = (CameraGLSurfaceView) findViewById(R.id.gl_texture_view);
        mCameraView.setCallback(this);
    }

    private void initCameraHandler() {
        mCameraHanlder = new Handler(Looper.getMainLooper()) {
            @Override
            public void handleMessage(Message msg) {
                switch (msg.what) {
                    case MSG_START_PREVIEW:
                        startPreview();
                        break;
                    default:
                        break;
                }
            }
        };
    }


    @Override
    protected void onResume() {
        super.onResume();

        startPreview();

        if (mVinsJNI == null) mVinsJNI = new magicpenJNI();
        mVinsJNI.init();
    }

    @Override
    protected void onPause() {
        super.onPause();
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

    @Override
    public void onSurfaceViewCreate(SurfaceTexture texture) {

    }

    @Override
    public void onSurfaceViewChange(int width, int height) {
        Log.e(TAG, "surfaceChanged: ( " + width +" x " + height +" )");
        mPreviewSize = new Size(width, height);
        mCameraHanlder.sendEmptyMessage(MSG_START_PREVIEW);
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
    public void onPreviewFrame(byte[] data, Camera camera) {

        // pass the current device's screen orientation to the c++ part
        int currentRotation = getWindowManager().getDefaultDisplay().getRotation();
        boolean isScreenRotated = true;

        // 避免两张图相同的时间戳
        mCameraView.getSurfaceTexture().getTimestamp();
        long curImageTimeStamp = mCameraView.getSurfaceTexture().getTimestamp();
        if (preImageTimeStamp > 0) {
            long cost = curImageTimeStamp - preImageTimeStamp;
            if (curImageTimeStamp - preImageTimeStamp < 30*1000*1000) {
                curImageTimeStamp = preImageTimeStamp + 30*1000*1000;
            }
        }
        preImageTimeStamp = curImageTimeStamp;

        mVinsJNI.onImageAvailable(imageWidth, imageHeight,
                0, null,
                0, null, null,
                null, curImageTimeStamp, isScreenRotated,
                virtualCamDistance, data, true);
    }

    /**
     * 打印Mat矩阵函数
     **/
    void printMatrix(RealMatrix input) {
        double matrixtoarray[][] = input.getData();
        for (int i = 0; i < matrixtoarray.length; i++) {
            for (int j = 0; j < matrixtoarray[0].length; j++) {
                System.out.print(matrixtoarray[i][j] + "\t");
            }
            System.out.print("\n");
        }
    }

    private boolean requestPermission() {
        return PermisstionUtil.checkPermissionsAndRequest(this, PermisstionUtil.CAMERA, REQUEST_CODE, "请求相机权限被拒绝")
                && PermisstionUtil.checkPermissionsAndRequest(this, PermisstionUtil.STORAGE, REQUEST_CODE, "请求访问SD卡权限被拒绝");
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions, @NonNull int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        if (requestCode == requestCode ) {
            mCameraHanlder.sendEmptyMessage(MSG_START_PREVIEW);
        }
    }
}
