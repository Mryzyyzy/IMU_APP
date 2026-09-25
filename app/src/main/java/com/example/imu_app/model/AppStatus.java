package com.example.imu_app.model;

public class AppStatus {
    public String connectionState;
    public String mode;
    public double dataRateHz;
    public int packetCount;
    public String message;
    public boolean running;

    public AppStatus() {
        connectionState = "未连接";
        mode = "IMU 解算模式";
        dataRateHz = 0.0;
        packetCount = 0;
        message = "等待启动";
        running = false;
    }
}
