package cn.ict.headingestimation;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.View;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.TextView;

import java.util.ArrayList;

import Jama.*;

import cn.ict.headingestimation.data.Acceleration;
import cn.ict.headingestimation.data.AngularVelocity;
import cn.ict.headingestimation.data.EulerAngle;
import cn.ict.headingestimation.data.MagneticField;

import cn.ict.headingestimation.data.TriaxialData;
import cn.ict.headingestimation.util.MadgwickAHRS;
import cn.ict.headingestimation.util.StepDetectionProvider;
import cn.ict.headingestimation.util.StepDetectionProvider.StepDetectedCallBack;


public class MainActivity extends AppCompatActivity implements View.OnClickListener, SensorEventListener {

    private static final double NS2S = 1.0f / 1000000000.0;
    private final int ACC = 1, GYR = 2, MAG = 3, EULER1 = 4, EULER2 = 5;
    private double viewWidth = 960;
    private double RAD2DEG = 180 / Math.PI;

    private int accCount = 0, gyrCount = 0, magCount = 0, gCount = 0;
    private Acceleration gravity = null;

    double midAccHeight = 120, midGyrHeight = 120, midMagHeight = 120, midEulerHeight = 120;

    private double[] RMArray = new double[9];
    private double[] IMArray = new double[9];
    private double[] orientation = new double[3];
    private double initGyrHeading = 0;
    private int initGyrStepCount = 0;
    private double initGyrMagSum = 0;
    private double lastGyrOrientation = 0;

    private Matrix RM, IM;

    private boolean initAngle = false;
    private MadgwickAHRS madgwickAHRS = new MadgwickAHRS(0.01f, 0.009f);

    private TextView accTxt, gyrTxt, magTxt, infoTxt;
    private SurfaceView accView, gyrView, magView, eulerView1, eulerView2;
    private SurfaceHolder accHolder, gyrHolder, magHolder, eulerHolder1, eulerHolder2;
    private Button startBtn, stopBtn, clearBtn;

    private ArrayList<Acceleration> acc = new ArrayList<Acceleration>();
    private ArrayList<AngularVelocity> gyr = new ArrayList<AngularVelocity>();
    private ArrayList<MagneticField> mag = new ArrayList<MagneticField>();
    private ArrayList<EulerAngle> eul1 = new ArrayList<EulerAngle>();
    private ArrayList<EulerAngle> eul2 = new ArrayList<EulerAngle>();
    private MagneticField lastMag = new MagneticField();

    private final static int DELAY = 500; // 1000000 / 500 = 2000Hz

    private SensorManager sensorManager;
    private Sensor accSensor, gyrSensor, magSensor;
    private Sensor gSensor;

    StepDetectionProvider stepDetectionProvider;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN, WindowManager.LayoutParams.FLAG_FULLSCREEN);
        setContentView(R.layout.activity_main);
        init();
    }

    private void init() {
        bindViews();
        clearLists();
        getSensors();
    }

    private void bindViews() {
        accTxt = (TextView) findViewById(R.id.acc_txt);
        gyrTxt = (TextView) findViewById(R.id.gyr_txt);
        magTxt = (TextView) findViewById(R.id.mag_txt);
        infoTxt = (TextView) findViewById(R.id.info);
        accView = (SurfaceView) findViewById(R.id.acc_view);
        gyrView = (SurfaceView) findViewById(R.id.gyr_view);
        magView = (SurfaceView) findViewById(R.id.mag_view);
        eulerView1 = (SurfaceView) findViewById(R.id.euler_view1);
        eulerView2 = (SurfaceView) findViewById(R.id.euler_view2);
        accHolder = accView.getHolder();
        gyrHolder = gyrView.getHolder();
        magHolder = magView.getHolder();
        eulerHolder1 = eulerView1.getHolder();
        eulerHolder2 = eulerView2.getHolder();
        startBtn = (Button) findViewById(R.id.start_btn);
        stopBtn = (Button) findViewById(R.id.stop_btn);
        clearBtn = (Button) findViewById(R.id.clear_btn);

        startBtn.setOnClickListener(this);
        stopBtn.setOnClickListener(this);
        stopBtn.setEnabled(false);
        clearBtn.setOnClickListener(this);
        clearBtn.setEnabled(false);
    }

    private void getSensors() {
        sensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);

        accSensor = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        gyrSensor = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        magSensor = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        gSensor = sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY);
    }

    private void regListeners() {
        sensorManager.registerListener(this, accSensor, DELAY);
        sensorManager.registerListener(this, gyrSensor, DELAY);
        sensorManager.registerListener(this, magSensor, DELAY);
        sensorManager.registerListener(this, gSensor, DELAY);
    }

    private void unregListeners() {
        sensorManager.unregisterListener(this, accSensor);
        sensorManager.unregisterListener(this, gyrSensor);
        sensorManager.unregisterListener(this, magSensor);
        sensorManager.unregisterListener(this, gSensor);
    }

    private void draw(int id) {
        Paint xPaint = new Paint(), yPaint = new Paint(), zPaint = new Paint();
        xPaint.setStrokeWidth(2); yPaint.setStrokeWidth(2); zPaint.setStrokeWidth(2);
        xPaint.setColor(Color.RED); yPaint.setColor(Color.GREEN); zPaint.setColor(Color.BLUE);
        switch (id) {
            case ACC:
                Canvas accCanvas = accHolder.lockCanvas(null);
                accCanvas.drawColor(Color.WHITE);
                for (int i = 1; i < acc.size(); i++) {
                    Acceleration accTemp1 = acc.get(i - 1);
                    accTemp1 = new Acceleration(midAccHeight - accTemp1.x / 20 * midAccHeight, midAccHeight - accTemp1.y / 20 * midAccHeight, midAccHeight - accTemp1.z / 20 * midAccHeight, accTemp1.timestamp);
                    Acceleration accTemp2 = acc.get(i);
                    accTemp2 = new Acceleration(midAccHeight - accTemp2.x / 20 * midAccHeight, midAccHeight - accTemp2.y / 20 * midAccHeight, midAccHeight - accTemp2.z / 20 * midAccHeight, accTemp2.timestamp);
                    accCanvas.drawLine(i - 1, (float) accTemp1.x, i, (float) accTemp2.x, xPaint);
                    accCanvas.drawLine(i - 1, (float) accTemp1.y, i, (float) accTemp2.y, yPaint);
                    accCanvas.drawLine(i - 1, (float) accTemp1.z, i, (float) accTemp2.z, zPaint);
                }
                accHolder.unlockCanvasAndPost(accCanvas);
                break;
            case GYR:
                Canvas gyrCanvas = gyrHolder.lockCanvas(null);
                gyrCanvas.drawColor(Color.WHITE);
                for (int i = 1; i < gyr.size(); i++) {
                    AngularVelocity gyrTemp1 = gyr.get(i - 1);
                    gyrTemp1 = new AngularVelocity(midGyrHeight - gyrTemp1.x / 15 * midGyrHeight, midGyrHeight - gyrTemp1.y / 15 * midGyrHeight, midGyrHeight - gyrTemp1.z / 15 * midGyrHeight, gyrTemp1.timestamp);
                    AngularVelocity gyrTemp2 = gyr.get(i);
                    gyrTemp2 = new AngularVelocity(midGyrHeight - gyrTemp2.x / 15 * midGyrHeight, midGyrHeight - gyrTemp2.y / 15 * midGyrHeight, midGyrHeight - gyrTemp2.z / 15 * midGyrHeight, gyrTemp2.timestamp);
                    gyrCanvas.drawLine(i - 1, (float) gyrTemp1.x, i, (float) gyrTemp2.x, xPaint);
                    gyrCanvas.drawLine(i - 1, (float) gyrTemp1.y, i, (float) gyrTemp2.y, yPaint);
                    gyrCanvas.drawLine(i - 1, (float) gyrTemp1.z, i, (float) gyrTemp2.z, zPaint);
                }
                gyrHolder.unlockCanvasAndPost(gyrCanvas);
                break;
            case MAG:
                Canvas magCanvas = magHolder.lockCanvas(null);
                magCanvas.drawColor(Color.WHITE);
                for (int i = 1; i < mag.size(); i++) {
                    MagneticField magTemp1 = mag.get(i - 1);
                    magTemp1 = new MagneticField(midMagHeight - magTemp1.x / 100 * midMagHeight, midMagHeight - magTemp1.y / 100 * midMagHeight, midMagHeight - magTemp1.z / 100 * midMagHeight, magTemp1.timestamp);
                    MagneticField magTemp2 = mag.get(i);
                    magTemp2 = new MagneticField(midMagHeight - magTemp2.x / 100 * midMagHeight, midMagHeight - magTemp2.y / 100 * midMagHeight, midMagHeight - magTemp2.z / 100 * midMagHeight, magTemp2.timestamp);
                    magCanvas.drawLine(i - 1, (float) magTemp1.x, i, (float) magTemp2.x, xPaint);
                    magCanvas.drawLine(i - 1, (float) magTemp1.y, i, (float) magTemp2.y, yPaint);
                    magCanvas.drawLine(i - 1, (float) magTemp1.z, i, (float) magTemp2.z, zPaint);
                }
                magHolder.unlockCanvasAndPost(magCanvas);
                break;
            case EULER1:
                Canvas eulerCanvas1 = eulerHolder1.lockCanvas(null);
                eulerCanvas1.drawColor(Color.WHITE);
                for (int i = 1; i < eul1.size(); i++) {
                    EulerAngle eulTemp1 = eul1.get(i - 1);
                    eulTemp1 = new EulerAngle(midEulerHeight - eulTemp1.roll / 200 * midEulerHeight, midEulerHeight - eulTemp1.pitch / 200 * midEulerHeight, midEulerHeight - eulTemp1.yaw / 200 * midEulerHeight);
                    EulerAngle eulTemp2 = eul1.get(i);
                    eulTemp2 = new EulerAngle(midEulerHeight - eulTemp2.roll / 200 * midEulerHeight, midEulerHeight - eulTemp2.pitch / 200 * midEulerHeight, midEulerHeight - eulTemp2.yaw / 200 * midEulerHeight);
                    eulerCanvas1.drawLine(i - 1, (float) eulTemp1.roll, i, (float) eulTemp2.roll, xPaint);
                    eulerCanvas1.drawLine(i - 1, (float) eulTemp1.pitch, i, (float) eulTemp2.pitch, yPaint);
                    eulerCanvas1.drawLine(i - 1, (float) eulTemp1.yaw, i, (float) eulTemp2.yaw, zPaint);
                }
                eulerHolder1.unlockCanvasAndPost(eulerCanvas1);
                break;
            case EULER2:
                Canvas eulerCanvas2 = eulerHolder2.lockCanvas(null);
                eulerCanvas2.drawColor(Color.WHITE);
                for (int i = 1; i < eul2.size(); i++) {
                    EulerAngle eulTemp1 = eul2.get(i - 1);
                    eulTemp1 = new EulerAngle(midEulerHeight - eulTemp1.roll / 200 * midEulerHeight, midEulerHeight - eulTemp1.pitch / 200 * midEulerHeight, midEulerHeight - eulTemp1.yaw / 200 * midEulerHeight);
                    EulerAngle eulTemp2 = eul2.get(i);
                    eulTemp2 = new EulerAngle(midEulerHeight - eulTemp2.roll / 200 * midEulerHeight, midEulerHeight - eulTemp2.pitch / 200 * midEulerHeight, midEulerHeight - eulTemp2.yaw / 200 * midEulerHeight);
                    eulerCanvas2.drawLine(i - 1, (float) eulTemp1.roll, i, (float) eulTemp2.roll, xPaint);
                    eulerCanvas2.drawLine(i - 1, (float) eulTemp1.pitch, i, (float) eulTemp2.pitch, yPaint);
                    eulerCanvas2.drawLine(i - 1, (float) eulTemp1.yaw, i, (float) eulTemp2.yaw, zPaint);
                }
                eulerHolder2.unlockCanvasAndPost(eulerCanvas2);
                break;
            default:
                break;
        }
    }

    private void clearLists() {
        acc.clear(); acc.add(new Acceleration());
        gyr.clear(); gyr.add(new AngularVelocity());
        mag.clear(); mag.add(new MagneticField());
        eul1.clear(); eul1.add(new EulerAngle());
        eul2.clear(); eul2.add(new EulerAngle());
        accCount = 0; gyrCount = 0; magCount = 0; gCount = 0;
        gravity = null;
        madgwickAHRS = new MadgwickAHRS(0.01f, 0.009f);
    }

    private void clearDraw() {
        Canvas accCanvas = accHolder.lockCanvas(null);
        Canvas gyrCanvas = gyrHolder.lockCanvas(null);
        Canvas magCanvas = magHolder.lockCanvas(null);
        accCanvas.drawColor(Color.BLACK);
        gyrCanvas.drawColor(Color.BLACK);
        magCanvas.drawColor(Color.BLACK);
        accHolder.unlockCanvasAndPost(accCanvas);
        gyrHolder.unlockCanvasAndPost(gyrCanvas);
        magHolder.unlockCanvasAndPost(magCanvas);
    }

    private double getMagOrientation() {
//        TriaxialData magSum = new TriaxialData();
//        for (int i = 1; i < mag.size(); i++) {
//            magSum = magSum.add(mag.get(i));
//        }
//        magSum = magSum.times(1.0 / (mag.size() - 1));
//        double magX = RM.times(new Matrix(magSum.toArray(), 3)).get(0, 0);
//        double magY = RM.times(new Matrix(magSum.toArray(), 3)).get(1, 0);
//        double magOrientation;
//        double magNorm = Math.sqrt((float) (magX * magX + magY * magY));
//        if (magY > 0) {
//            magOrientation = Math.asin((float) (magX / magNorm)) * RAD2DEG;
//        } else {
//            magOrientation = (Math.PI - Math.asin((float) (magX / magNorm))) * RAD2DEG;
//        }
//        if (magOrientation > 180) magOrientation -= 360;
//
//        return magOrientation;

        MagneticField magSum = new MagneticField();
        for (int i = 1; i < mag.size(); i++) {
            magSum = magSum.add(mag.get(i));
        }
        magSum = magSum.times(1.0 / (mag.size() - 1));
        double magX = magSum.x;
        double magY = magSum.y;

        double magOrientation;
        double magNorm = Math.sqrt((float) (magX * magX + magY * magY));
        if (magY > 0) {
            magOrientation = Math.asin((float) (magX / magNorm)) * RAD2DEG;
        } else {
            magOrientation = (Math.PI - Math.asin((float) (magX / magNorm))) * RAD2DEG;
        }
        if (magOrientation > 180) magOrientation -= 360;

        return magOrientation;
    }

    /**
     * @return 根据gyr列表中的陀螺仪数据计算出的航向角的变化量，北向转东向为正
     */
    private double getGyrDiff() {
//        TriaxialData avSum = new TriaxialData();
//        for (int i = 2; i < gyr.size(); i++) {
//            AngularVelocity temp1 = gyr.get(i - 1);
//            AngularVelocity temp2 = gyr.get(i);
//            avSum = avSum.add(temp1.add(temp2).times(Math.abs(temp1.timestamp - temp2.timestamp) * NS2S));
//        }
//        return RM.times(new Matrix(avSum.toArray(), 3)).get(2, 0) * RAD2DEG;

        TriaxialData avSum = new TriaxialData();
        for (int i = 2; i < gyr.size(); i++) {
            AngularVelocity temp1 = gyr.get(i - 1);
            AngularVelocity temp2 = gyr.get(i);
            avSum = avSum.add(temp1.add(temp2).times(Math.abs(temp1.timestamp - temp2.timestamp) * NS2S));
        }
        return avSum.z;
    }

    private double[] getGyrAndMagOrientation() {
        double magOrientation = getMagOrientation();
        double gyrOrientationDiff = getGyrDiff();
        double gyrOrientation;
        initGyrMagSum += magOrientation;
        initGyrStepCount++;
        if (!initAngle) {
            if (Math.abs(gyrOrientationDiff) > 30) {
                initGyrStepCount = 0;
                initGyrMagSum = 0;
                infoTxt.setText("clear magsum");
            }
            if (initGyrStepCount > 10) {
                initAngle = true;
                initGyrHeading = initGyrMagSum / initGyrStepCount;
                initGyrStepCount = 0;
                initGyrMagSum = 0;
                lastGyrOrientation = initGyrHeading;
            }
            gyrOrientation = gyrOrientationDiff;
        } else {
            gyrOrientation = lastGyrOrientation + gyrOrientationDiff;
            while (gyrOrientation > 180) gyrOrientation -= 360;
            while (gyrOrientation < -180) gyrOrientation += 360;
            lastGyrOrientation = gyrOrientation;
            infoTxt.setText("GyrOrientation = " + lastGyrOrientation);
        }

        double[] orientations = {gyrOrientation, magOrientation};
        clearLists();
        return orientations;
    }

    @Override
    public void onClick(View v) {
        switch (v.getId()) {
            case R.id.stop_btn:
                stopBtn.setEnabled(false);
                clearBtn.setEnabled(true);
                unregListeners();
                stepDetectionProvider.stop();
                break;
            case R.id.start_btn:
                startBtn.setEnabled(false);
                stopBtn.setEnabled(true);
                regListeners();
                StepDetectedCallBack stepDetectedCallBack = new StepDetectedCallBack() {
                    @Override
                    public void onStepDetected(int stepCount) {
                        double[] ori = getGyrAndMagOrientation();
                        infoTxt.setText("" + ori[0] + " " + ori[1]  );
                    }
                };
                stepDetectionProvider = new StepDetectionProvider(this, stepDetectedCallBack);
                stepDetectionProvider.start();
                break;
            case R.id.clear_btn:
                startBtn.setEnabled(true);
                clearBtn.setEnabled(false);
                initAngle = false;
                lastGyrOrientation = 0;
                clearLists();
                clearDraw();
                break;
            default:
                break;
        }
    }

    private void calcRotationMatrix() {
        Acceleration g = new Acceleration();
        MagneticField m = new MagneticField();
        for (Acceleration accIdx : acc) {
            g = g.add(accIdx);
        }
        for (MagneticField magIdx : mag) {
            m = m.add(magIdx);
        }

        g.times(1 / acc.size());
        m.times(1 / mag.size());

        float[] RMTemp = new float[9];
        float[] IMTemp = new float[9];
        SensorManager.getRotationMatrix(RMTemp, IMTemp, g.toFloatArray(), m.toFloatArray());    // 得到从载体坐标系到导航坐标系的转换矩阵RM和磁偏矩阵IM
        float[] orientationTemp = new float[3];
        SensorManager.getOrientation(RMTemp, orientationTemp);
        orientation = floatArrayToDoubleArray(orientationTemp);

        RM = new Matrix(floatArrayToDoubleArray(RMTemp), 3);
        IM = new Matrix(floatArrayToDoubleArray(IMTemp), 3);
        infoTxt.setText(String.format("Orientation:\nAzimuth:%.5f     Pitch:%.5f     Roll:%.5f", orientation[0] * RAD2DEG, orientation[1] * RAD2DEG, orientation[2] * RAD2DEG));
    }

    private double[] floatArrayToDoubleArray(float[] a) {
        double[] res = new double[a.length];
        for (int i = 0; i < a.length; i++) {
            res[i] = (double) a[i];
        }
        return res;
    }

    private float[] doubleArrayToFloatArray(double[] a) {
        float[] res = new float[a.length];
        for (int i = 0; i < a.length; i++) {
            res[i] = (float) a[i];
        }
        return res;
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        switch (event.sensor.getType()) {
            case Sensor.TYPE_ACCELEROMETER:
//                accTxt.setText(String.format("Acc    x:%.5f    y:%.5f    z:%.5f", event.values[0], event.values[1], event.values[2]));
//                Acceleration accCurrent = new Acceleration(event.values[0], event.values[1], event.values[2], event.timestamp);
//                if (hasGravity) {
//                    accCurrent = accCurrent.substract(gravity);
//                }
//                acc.add(accCurrent);

//                Acceleration a = acc.get(acc.size() - 1);
//                AngularVelocity g = gyr.get(gyr.size() - 1);
//                MagneticField m = lastMag;
//                double ax = a.y, ay = -a.x, az = a.z;
//                double gx = g.y, gy = -g.x, gz = g.z;
//                double mx = m.y, my = -m.x, mz = m.z;
//                madgwickAHRS.update((float) gx, (float) gy, (float) gz, (float) ax, (float) ay, (float) az, (float) mx, (float) my, (float) mz);
//                double[] q = madgwickAHRS.Quaternion;
//                Matrix Rab = new Matrix(3, 3);
//                Rab.set(0, 0, 2 * q[1] * q[1] - 1 + 2 * q[2] * q[2]);
//                Rab.set(0, 1, 2 * (q[2] * q[3] + q[1] * q[4]));
//                Rab.set(0, 2, 2 * (q[2] * q[4] - q[1] * q[3]));
//                Rab.set(1, 0, 2 * (q[2] * q[3] - q[1] * q[4]));
//                Rab.set(1, 1, 2 * q[1] * q[1] - 1 + 2 * q[3] * q[3]);
//                Rab.set(1, 2, 2 * (q[3] * q[4] + q[1] * q[2]));
//                Rab.set(2, 0, 2 * (q[2] * q[4] + q[1] * q[3]));
//                Rab.set(2, 1, 2 * (q[3] * q[4] - q[1] * q[2]));
//                Rab.set(2, 2, 2 * q[1] * q[1] - 1 + 2 * q[4] * q[4]);

//                double roll = Math.atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0] * q[0] + 2 * q[1] * q[1] - 1) * RAD2DEG;
//                double pitch = Math.asin(2 * q[1] * q[3] + 2 * q[0] * q[2]) / 50 * RAD2DEG;
//                double yaw = Math.atan2(2 * q[2] * q[3] - 2 * q[0] * q[1], 2 * q[0] * q[0] + 2 * q[3] * q[3] - 1) * RAD2DEG;

//                double[] euler = madgwickAHRS.getEuler();
//                eul1.add(new EulerAngle(roll, pitch, yaw));
//                eul2.add(new EulerAngle(euler[1], euler[2], euler[0]));

//                infoTxt.setText(String.format("Euler(cal):roll:%.2f     pitch:%.2f     yaw:%.2f\nEuler(get):roll:%.2f     pitch:%.2f     yaw:%.2f", roll, pitch, yaw, euler[1], euler[2], euler[0]));

//                float[] RMTemp = new float[9];
//                float[] IMTemp = new float[9];
//                float[] orientationTemp = new float[3];
//                SensorManager.getRotationMatrix(RMTemp, IMTemp,
//                        doubleArrayToFloatArray(acc.get(acc.size() - 1).toArray()),
//                        doubleArrayToFloatArray(mag.get(mag.size() - 1).toArray()));
//                SensorManager.getOrientation(RMTemp, orientationTemp);
//                orientation = floatArrayToDoubleArray(orientationTemp);
//                RM = new Matrix(floatArrayToDoubleArray(RMTemp), 3);
//                IM = new Matrix(floatArrayToDoubleArray(IMTemp), 3);
//                infoTxt.setText(String.format("Orientation:\nAzimuth:%.5f     Pitch:%.5f     Roll:%.5f", orientation[0], orientation[1], orientation[2]));

//                accCount++;
                if (acc.size() > viewWidth) {
                    acc.remove(0);
////                    eul1.remove(0);
////                    eul2.remove(0);
                }
//                if (accCount == 10) {
//                    draw(ACC);
////                    draw(EULER1);
////                    draw(EULER2);
//                    accCount = 0;
//                }
                break;
            case Sensor.TYPE_GYROSCOPE:
                AngularVelocity gyrTemp = new AngularVelocity(event.values[0], event.values[1], event.values[2], event.timestamp);
                if (null != gravity) {
                    System.out.println(gravity);
                    AngularVelocity gyrZ = gravity.times(gravity.times(gyrTemp) / gravity.normSquare()).toAv();
                    gyr.add(gyrZ);
                    gyrTxt.setText(String.format("Gyr    x:%.5f    y:%.5f    z:%.5f", gyrZ.x, gyrZ.y, gyrZ.z));
                }
//                gyrCount++;
//                if (gyrCount == 10) {
//                    draw(GYR);
//                    gyrCount = 0;
//                }
//                if (gyr.size() > viewWidth) {
//                    gyr.remove(0);
//                }
                break;
            case Sensor.TYPE_MAGNETIC_FIELD:
                lastMag = new MagneticField(event.values[0], event.values[1], event.values[2], event.timestamp);
                // 求地磁的水平分量
                if (null != gravity) {
                    MagneticField magH = lastMag.substract(gravity.times(gravity.times(lastMag) / gravity.normSquare()).toMag());
                    mag.add(magH);
                    magTxt.setText(String.format("Mag    x:%.5f    y:%.5f    z:%.5f", magH.x, magH.y, magH.z));
                }
//                magCount++;
//                if (magCount == 10) {
//                    draw(MAG);
//                    magCount = 0;
//                }
//                if (mag.size() > viewWidth) {
//                    mag.remove(0);
//                }
                break;
            case Sensor.TYPE_GRAVITY:
                Acceleration gTemp = new Acceleration(event.values[0], event.values[1], event.values[2], event.timestamp);
                accTxt.setText(String.format("G:   x:%.5f  y:%.5f  z:%.5f", event.values[0], event.values[1], event.values[2]));
                if (gCount == 10) {
                    gravity = gTemp;
                } else {
                    gCount++;
                }
                break;
            default:
                break;
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {

    }
}
