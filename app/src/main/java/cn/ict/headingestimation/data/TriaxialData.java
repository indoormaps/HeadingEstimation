package cn.ict.headingestimation.data;

/**
 * Created by Archeries on 2017/9/6.
 */
public class TriaxialData {
    public double x, y, z;
    public long timestamp;

    public TriaxialData() {
        this(0, 0, 0, 0);
    }

    public TriaxialData(double x, double y, double z) {
        this(x, y, z, 0l);
    }

    public TriaxialData(double x, double y, double z, long timestamp) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.timestamp = timestamp;
    }

    public TriaxialData add(TriaxialDataDelta delta) {
        return new TriaxialData(x + delta.x, y + delta.y, z + delta.z, timestamp);
    }

    public TriaxialData add(TriaxialData data){
        return new TriaxialData(x + data.x, y + data.y, z + data.z, timestamp);
    }

    public TriaxialData substract(TriaxialDataDelta delta) {
        return new TriaxialData(x - delta.x, y - delta.y, z - delta.z, timestamp);
    }

    public TriaxialData substract(TriaxialData data) {
        return new TriaxialData(x - data.x, y - data.y, z - data.z, timestamp);
    }

    public TriaxialData times(double time) {
        return new TriaxialData(x * time, y * time, z * time, timestamp);
    }

    public double times(TriaxialData a) {
        return x * a.x + y * a.y + z * a.z;
    }

    public double[] toArray() {
        double[] res = new double[3];
        res[0] = x; res[1] = y; res[2] = z;
        return res;
    }

    public float[] toFloatArray() {
        float[] res = new float[3];
        res[0] = (float) x; res[1] = (float) y; res[2] = (float) z;
        return res;
    }

    public double normSquare() {
        return x * x + y * y + z * z;
    }

    public MagneticField toMag() {
        return new MagneticField(x, y, z, timestamp);
    }

    public AngularVelocity toAv() {
        return new AngularVelocity(x, y, z, timestamp);
    }

    public Acceleration toAcc() {
        return new Acceleration(x, y, z, timestamp);
    }
}
