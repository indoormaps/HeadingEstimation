package cn.ict.headingestimation.util;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

public class TurnDetectionProvider {
	
	private Context mContext;
	private SensorManager sensorManager;
	private Sensor accelerometerSensor;
	private Sensor gyroscopeSensor;
	
	
	// record last record time for two listener
	private long aLastTime, gLastTime;
	public static long TIME_GAP = 100;
	private double[] xAc, yAc, zAc;
	int count = 0;
	double xAccelerometer = 0;
	double yAccelerometer = 0;
	double zAccelerometer = 0;
	double xDirectionCosine = 0, yDirectionCosine = 0, zDirectionCosine = 0;

	private double[] xPit, yRol, zYaw;
	int queue = 0;
	double xPitchAngularRate = 0;
	double yRollAngularRate = 0;
	double zYawAngularRate = 0;

	private double turning = 0.0;
	private double[] angles;
	private int angleCount = 0;
	private long lastTurnTime;
	
	public TurnDetectionProvider(Context ctx, TurnDetectedCallBack cb) {
		mContext = ctx;
		turnDetectionCallback = cb;
		sensorManager = (SensorManager)mContext.getSystemService(Context.SENSOR_SERVICE);
		accelerometerSensor = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
		gyroscopeSensor = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
	}
	
	private void initValue() {
		aLastTime = System.currentTimeMillis();
		gLastTime = System.currentTimeMillis();
		
		xAc = new double[100];
		yAc = new double[100];
		zAc = new double[100];
		xPit = new double[30];
		yRol = new double[30];
		zYaw = new double[30];
		angles = new double[3];
	}
	
	public void start() {
		initValue();
		sensorManager.registerListener(accelerometerListener, accelerometerSensor, SensorManager.SENSOR_DELAY_NORMAL);
		sensorManager.registerListener(gyroscopeListener, gyroscopeSensor, SensorManager.SENSOR_DELAY_NORMAL);
	}
	
	public void stop() {
		sensorManager.unregisterListener(accelerometerListener);
		sensorManager.unregisterListener(gyroscopeListener);
	}
	
	public double sum(double theQueue[], int queueLength) {
		double sum = 0;
		for (int i = 0; i < queueLength; i++) {
			sum += theQueue[i];
		}
		return sum;
	}

	/**
	 * @param angleCount
	 * @param angleThreshold
	 * @return the result of detection
	 */
	private boolean turnDetection(int angleCount, double angleThreshold) {
		if ((Math.abs(angles[angleCount % angles.length]) > angleThreshold)
				&& (Math.abs(angles[angleCount % angles.length]) > Math
						.abs(angles[(angleCount - 1 + angles.length)
								% angles.length]))) {
			return true;
		} else {
			return false;
		}
	}

	private SensorEventListener accelerometerListener = new SensorEventListener() {
		
		@Override
		public void onSensorChanged(SensorEvent event) {
			long aTime = System.currentTimeMillis() - aLastTime;
			if (aTime <= 100) {
				return;
			}
			// TODO Auto-generated method stub
			xAccelerometer = event.values[0];
			yAccelerometer = event.values[1];
			zAccelerometer = event.values[2];

			aLastTime = System.currentTimeMillis();
		}
		
		@Override
		public void onAccuracyChanged(Sensor sensor, int accuracy) {
			
		}
	};
	
	private SensorEventListener gyroscopeListener = new SensorEventListener() {
		
		@Override
		public void onSensorChanged(SensorEvent event) {
			long gTime = System.currentTimeMillis() - gLastTime;
			if (gTime <= 100) {
				return;
			}
			// TODO Auto-generated method stub
			double xAngle = 0, yAngle = 0, zAngle = 0;
			double xx = 0, yy = 0, zz = 0;
			double norm = 0;
			xPitchAngularRate = event.values[0];
			yRollAngularRate = event.values[1];
			zYawAngularRate = event.values[2];

			gLastTime = System.currentTimeMillis();

			if (Math.abs(turning) < 10) {
				
				xAc[count] = xAccelerometer;
				yAc[count] = yAccelerometer;
				zAc[count] = zAccelerometer;

				if ((count + 1) % (xAc.length) != 0) {
					count++;
				} else {
					count = 0;
				}
			}

			xx = sum(xAc, xAc.length);
			yy = sum(yAc, yAc.length);
			zz = sum(zAc, zAc.length);

			norm = Math.sqrt(Math.pow(xx, 2) + Math.pow(yy, 2)
					+ Math.pow(zz, 2));

			if (norm != 0) {
				xDirectionCosine = xx / norm;
				yDirectionCosine = yy / norm;
				zDirectionCosine = zz / norm;
			} else {
				xDirectionCosine = 0;
				yDirectionCosine = 0;
				zDirectionCosine = 0;
			}

			xPit[queue] = xPitchAngularRate;
			yRol[queue] = yRollAngularRate;
			zYaw[queue] = zYawAngularRate;
			if ((queue + 1) % (xPit.length) != 0) {
				queue++;
			} else {
				queue = 0;
			}
			// integrating gyroscope readings along time
			xAngle = sum(xPit, xPit.length);
			yAngle = sum(yRol, yRol.length);
			zAngle = sum(zYaw, zYaw.length);
			// multiple the rate of sensor
			xAngle *= 0.2;
			yAngle *= 0.2;
			zAngle *= 0.2;

			// calculate an incoming turn
			turning = Math.toDegrees(xAngle * xDirectionCosine + yAngle
					* yDirectionCosine + zAngle * zDirectionCosine);
			if (turning != Double.NaN) {
				angles[angleCount % angles.length] = turning
						* 0.7
						+ angles[(angleCount - 1 + angles.length)
								% angles.length]
						* 0.2
						+ angles[(angleCount - 2 + angles.length)
								% angles.length] * 0.1;
				angleCount++;

				if (turnDetection(angleCount - 1, 65.0)) {

					for (int i = 0; i < xPit.length; i++) {
						xPit[i] = 0;
						yRol[i] = 0;
						zYaw[i] = 0;
					}
					// least timeSpan between two turn action is 2s
					if ((event.timestamp - lastTurnTime) / 1000000000 > 2) {
						lastTurnTime = event.timestamp;
						turnDetectionCallback.onTurnDetected(true);
					}
				}
				if (angleCount == angles.length) {
					angleCount = 0;
				}
			}
		}
		
		@Override
		public void onAccuracyChanged(Sensor sensor, int accuracy) {
			
		}
		
	};
	private TurnDetectedCallBack turnDetectionCallback;
	
	public interface TurnDetectedCallBack {
		public void onTurnDetected(boolean flag);
	}
}
