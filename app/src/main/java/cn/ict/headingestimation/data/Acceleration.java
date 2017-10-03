package cn.ict.headingestimation.data;

/**
 * Created by Archeries on 2017/9/6.
 */
public class Acceleration extends TriaxialData {

    public Acceleration() {
        super();
    }

    public Acceleration(double x, double y, double z, long timestamp) {
        super(x, y, z, timestamp);
    }

    public Acceleration add(Acceleration data) {
        return new Acceleration(x + data.x, y + data.y, z + data.z, timestamp);
    }

    public Acceleration add(TriaxialDataDelta delta) {
        return new Acceleration(x + delta.x, y + delta.y, z + delta.z, timestamp);
    }

    public Acceleration substract(Acceleration data) {
        return new Acceleration(x - data.x, y - data.y, z - data.z, timestamp);
    }
}
