package com.example.imu_app.protocol;

import com.example.imu_app.model.ImuSample;

public class ImuCsvParser {
    private static final double STANDARD_GRAVITY = 9.80665;

    public ImuSample parse(String line, long timestampMillis) throws ProtocolParseException {
        double[] values = parseNumbers(line);
        if (values.length != 10) {
            throw new ProtocolParseException("IMU CSV 需要 10 个字段，当前为 " + values.length + " 个。");
        }
        int packetId = (int) Math.round(values[0]);
        return new ImuSample(
                timestampMillis,
                values[1] * STANDARD_GRAVITY,
                values[2] * STANDARD_GRAVITY,
                values[3] * STANDARD_GRAVITY,
                Math.toRadians(values[4]),
                Math.toRadians(values[5]),
                Math.toRadians(values[6]),
                values[7],
                values[8],
                values[9],
                null,
                packetId
        );
    }

    private double[] parseNumbers(String line) throws ProtocolParseException {
        String[] parts = line.trim().split(",");
        double[] values = new double[parts.length];
        for (int i = 0; i < parts.length; i++) {
            String part = parts[i].trim();
            if (part.isEmpty()) {
                throw new ProtocolParseException("CSV 字段为空。");
            }
            try {
                values[i] = Double.parseDouble(part);
            } catch (NumberFormatException error) {
                throw new ProtocolParseException("CSV 字段包含非数字内容。");
            }
            if (!Double.isFinite(values[i])) {
                throw new ProtocolParseException("CSV 包含 NaN 或 Inf。");
            }
        }
        return values;
    }
}
