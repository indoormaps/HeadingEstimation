package cn.ict.headingestimation;

/**
 * Created by Archeries on 2017/10/11.
 */
public class Config {

    public static int device = 1;
    public final static int DEVICE442 = 1;
    public final static int DEVICE6 = 2;
    public final static boolean logdata = false;
    public final static boolean calibrateGyr = true;
    public final static boolean useGyr = true;

    public final static double viewWidth = 960;
    public final static double midAccHeight = 120, midGyrHeight = 120, midMagHeight = 120, midPathHeight = 240;

    public final static double STEP_LENGTH = 6;

    public final static double thetaCThreshold = 10;
    public final static double thetaMThreshold = 15;
    public final static double[][] weights = {
    // lastStepOrientation, magOrientation, gyrOrientation
            {0.3, 0.5, 0.2},  // walk straight, when thetaC <= thetaCThreshold && thetaM <= thetaMThreshold
            {  0, 0.25, 0.75},  // turning, when thetaC <= thetaCThreshold && thetaM > thetaMThreshold
            {0.85, 0.15,   0},  // when thetaC > thetaCThreshold && thetaM <= thetaMThreshold
            {0.2,   0, 0.8}   // magnetometer disturbed, when thetaC > thetaCThreshold && thetaM > thetaMThreshold
    };

    public final static int DELAY = 500; // 1000000 / 500 = 2000Hz
}
