package com.example.imu_app.model;

public class TrackPoint {
    public final long timestampMillis;
    public final Double latitude;
    public final Double longitude;
    public final Double altitude;
    public final double east;
    public final double north;
    public final double up;
    public final double velocityE;
    public final double velocityN;
    public final double velocityU;
    public final double speed;
    public final Double roll;
    public final Double pitch;
    public final Double yaw;
    public final String source;
    public final double quality;

    public TrackPoint(
            long timestampMillis,
            Double latitude,
            Double longitude,
            Double altitude,
            double east,
            double north,
            double up,
            double velocityE,
            double velocityN,
            double velocityU,
            double speed,
            Double roll,
            Double pitch,
            Double yaw,
            String source,
            double quality
    ) {
        this.timestampMillis = timestampMillis;
        this.latitude = latitude;
        this.longitude = longitude;
        this.altitude = altitude;
        this.east = east;
        this.north = north;
        this.up = up;
        this.velocityE = velocityE;
        this.velocityN = velocityN;
        this.velocityU = velocityU;
        this.speed = speed;
        this.roll = roll;
        this.pitch = pitch;
        this.yaw = yaw;
        this.source = source;
        this.quality = quality;
    }
}
