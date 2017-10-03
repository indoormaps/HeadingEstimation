package cn.ict.headingestimation.data;

/**
 * Created by Archeries on 2017/9/6.
 */
public class AngularVelocity extends TriaxialData {
    public AngularVelocity() {
        super();
    }

    public AngularVelocity(double x, double y, double z, long timestamp) {
        super(x, y, z, timestamp);
    }

    public AngularVelocity add(TriaxialData data) {
        return new AngularVelocity(x + data.x, y + data.y, z + data.z, timestamp);
    }

    public AngularVelocity add(TriaxialDataDelta delta) {
        return new AngularVelocity(x + delta.x, y + delta.y, z + delta.z, timestamp);
    }

    public AngularVelocity substract(TriaxialData data) {
        return new AngularVelocity(x - data.x, y - data.y, z - data.z, timestamp);
    }

    public AngularVelocity substract(TriaxialDataDelta delta) {
        return new AngularVelocity(x - delta.x, y - delta.y, z - delta.z, timestamp);
    }

    public AngularVelocity times(double time) {
        return new AngularVelocity(x * time, y * time, z * time, timestamp);
    }
}
