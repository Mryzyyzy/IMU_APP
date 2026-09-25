package com.example.imu_app.model;

public class PositionFix {
    public final long timestampMillis;
    public final Double x;
    public final Double y;
    public final Double z;
    public final Double latitude;
    public final Double longitude;
    public final Double altitude;
    public final Double yaw;
    public final double quality;
    public final Integer packetId;

    public PositionFix(
            long timestampMillis,
            Double x,
            Double y,
            Double z,
            Double latitude,
            Double longitude,
            Double altitude,
            Double yaw,
            double quality,
            Integer packetId
    ) {
        this.timestampMillis = timestampMillis;
        this.x = x;
        this.y = y;
        this.z = z;
        this.latitude = latitude;
        this.longitude = longitude;
        this.altitude = altitude;
        this.yaw = yaw;
        this.quality = quality;
        this.packetId = packetId;
    }
}
