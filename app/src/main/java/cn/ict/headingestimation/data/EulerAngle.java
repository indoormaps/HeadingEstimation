package cn.ict.headingestimation.data;

/**
 * Created by Archeries on 2017/9/12.
 */
public class EulerAngle {
    public double roll, pitch, yaw;

    public EulerAngle() {
        this.roll = 0;
        this.pitch = 0;
        this.yaw = 0;
    }

    public EulerAngle(double roll, double pitch, double yaw) {
        this.roll = roll;
        this.pitch = pitch;
        this.yaw = yaw;
    }
}
