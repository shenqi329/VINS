package com.example.vins;

import android.app.Activity;
import android.content.Context;
import android.content.pm.PackageInfo;
import android.content.pm.PackageManager;
import android.graphics.SurfaceTexture;
import android.hardware.Camera;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.os.Message;
import android.util.Log;
import android.util.Size;
import android.widget.ImageView;
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

public class CameraActivity extends Activity implements Camera.PreviewCallback, CameraGLSurfaceView.CameraGLSurfaceViewCallback, SensorEventListener {
    
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
    //private magicpenJNI mMaginPenJNI;

    long preImageTimeStamp = -1;
    long firstTimeStamp = -1;

    private float virtualCamDistance = 4;
    private final float minVirtualCamDistance = 4;
    private final float maxVirtualCamDistance = 40;
    private long mLastExitTime;

    private Size mPreviewSize;
    private Handler mCameraHanlder;

    private SensorManager sensorManager;
    private Sensor magneticSensor;
    private Sensor accelerometerSensor;
    private float[] gravity;
    private float[] r;
    private float[] geomagnetic;
    private float[] values;

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

        MagicPenJNI.setEdgeImageByte(getFromRaw());


        /**
         * 初始化传感器
         * */
        sensorManager = (SensorManager)getSystemService(Context.SENSOR_SERVICE);
        //获取Sensor
        magneticSensor = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        accelerometerSensor = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        //初始化数组
        gravity = new float[3];//用来保存加速度传感器的值
        r = new float[9];//
        geomagnetic = new float[3];//用来保存地磁传感器的值
        values = new float[3];//用来保存最终的结果

        sensorManager.registerListener(this, magneticSensor, SensorManager.SENSOR_DELAY_GAME);
        sensorManager.registerListener(this, accelerometerSensor, SensorManager.SENSOR_DELAY_GAME);
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
    }

    @Override
    protected void onPause() {
        super.onPause();
        CameraUtil.releaseCamera();
        System.exit(0);
    }

    private byte[] getFromRaw() {
        try {
            InputStream in = getResources().openRawResource(R.raw.edge);
            //获取文件的字节数
            int lenght = in.available();
            //创建byte数组
            byte[] buffer = new byte[lenght];
            //将文件中的数据读到byte数组中
            in.read(buffer);
            return buffer;
        } catch (Exception e) {
            e.printStackTrace();
        }
        return new byte[0];
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
        if (firstTimeStamp < 0) {
            firstTimeStamp = curImageTimeStamp;
        }
        if (preImageTimeStamp > 0) {
            long cost = curImageTimeStamp - preImageTimeStamp;
            if (curImageTimeStamp - preImageTimeStamp < 30*1000*1000) {
                curImageTimeStamp = preImageTimeStamp + 30*1000*1000;
            }
        }
        preImageTimeStamp = curImageTimeStamp;

        if(curImageTimeStamp - firstTimeStamp < 1000000000) {
            return;
        }

        MagicPenJNI.onImageAvailable(imageWidth, imageHeight,
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

    @Override
    public void onSensorChanged(SensorEvent event) {
        if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
            geomagnetic = event.values;
        }
        if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
            gravity = event.values;
            getOritation();
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {

    }

    /**
     * 获取手机旋转角度
     */
    public void getOritation() {
        // r从这里返回
        SensorManager.getRotationMatrix(r, null, gravity, geomagnetic);
        //values从这里返回
        SensorManager.getOrientation(r, values);
        //提取数据
        double degreeX = Math.toDegrees(values[1]);
        double degreeY = Math.toDegrees(values[2]);
        MagicPenJNI.setRotate((float) degreeX, (float)degreeY);
    }
}
