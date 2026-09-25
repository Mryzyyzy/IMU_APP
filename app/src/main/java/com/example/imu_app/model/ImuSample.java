package com.example.imu_app.model;

public class ImuSample {
    public final long timestampMillis;
    public final double ax;
    public final double ay;
    public final double az;
    public final double gx;
    public final double gy;
    public final double gz;
    public final Double mx;
    public final Double my;
    public final Double mz;
    public final Double temperature;
    public final int packetId;

    public ImuSample(
            long timestampMillis,
            double ax,
            double ay,
            double az,
            double gx,
            double gy,
            double gz,
            Double mx,
            Double my,
            Double mz,
            Double temperature,
            int packetId
    ) {
        this.timestampMillis = timestampMillis;
        this.ax = ax;
        this.ay = ay;
        this.az = az;
        this.gx = gx;
        this.gy = gy;
        this.gz = gz;
        this.mx = mx;
        this.my = my;
        this.mz = mz;
        this.temperature = temperature;
        this.packetId = packetId;
    }
}
