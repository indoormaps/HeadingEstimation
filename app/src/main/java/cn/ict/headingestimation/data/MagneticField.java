package cn.ict.headingestimation.data;

/**
 * Created by Archeries on 2017/9/6.
 */
public class MagneticField extends TriaxialData {

    public MagneticField() {
        super();
    }

    public MagneticField(double x, double y, double z, long timestamp) {
        super(x, y, z, timestamp);
    }

    public MagneticField add(TriaxialData data) {
        return new MagneticField(x + data.x, y + data.y, z + data.z, timestamp);
    }

    public MagneticField add(TriaxialDataDelta delta) {
        return new MagneticField(x + delta.x, y + delta.y, z + delta.z, timestamp);
    }

    public MagneticField substract(MagneticField data) {
        return new MagneticField(x - data.x, y - data.y, z - data.z, timestamp);
    }

    public MagneticField times(double time) {
        return new MagneticField(x * time, y * time, z * time, timestamp);
    }
}
