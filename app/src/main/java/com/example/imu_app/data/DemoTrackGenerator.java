package com.example.imu_app.data;

import com.example.imu_app.model.TrackPoint;

public class DemoTrackGenerator {
    private int index = 0;
    private double lastEast = 0.0;
    private double lastNorth = 0.0;
    private double totalDistance = 0.0;

    public TrackPoint next() {
        double t = index * 0.12;
        double east = 22.0 * Math.sin(t * 0.55) + 0.55 * index;
        double north = 16.0 * Math.sin(t * 0.32) + 9.0 * Math.cos(t * 0.18);
        double up = 0.18 * Math.sin(t * 0.4);

        double dEast = east - lastEast;
        double dNorth = north - lastNorth;
        double distance = Math.hypot(dEast, dNorth);
        if (index > 0) {
            totalDistance += distance;
        }
        double speed = Math.min(2.8, distance / 0.1);
        double yaw = Math.toDegrees(Math.atan2(dNorth, dEast));
        if (yaw < 0) {
            yaw += 360.0;
        }

        double roll = 2.5 * Math.sin(t * 0.8);
        double pitch = 1.8 * Math.cos(t * 0.7);
        double quality = 0.92 + 0.05 * Math.sin(t * 0.25);

        lastEast = east;
        lastNorth = north;
        index++;

        return new TrackPoint(
                System.currentTimeMillis(),
                null,
                null,
                null,
                east,
                north,
                up,
                dEast / 0.1,
                dNorth / 0.1,
                0.0,
                speed,
                roll,
                pitch,
                yaw,
                "demo",
                quality
        );
    }

    public double getTotalDistance() {
        return totalDistance;
    }

    public void reset() {
        index = 0;
        lastEast = 0.0;
        lastNorth = 0.0;
        totalDistance = 0.0;
    }
}
