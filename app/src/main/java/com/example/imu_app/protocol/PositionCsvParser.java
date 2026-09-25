package com.example.imu_app.protocol;

import com.example.imu_app.model.PositionFix;

public class PositionCsvParser {
    public PositionFix parse(String line, long timestampMillis) throws ProtocolParseException {
        double[] values = parseNumbers(line);
        if (values.length != 3 && values.length != 5 && values.length != 6) {
            throw new ProtocolParseException("Position CSV 支持 3、5 或 6 个字段。");
        }

        Integer packetId = null;
        int offset = 0;
        if (values.length == 6 && looksLikePacketId(values[0])) {
            packetId = (int) Math.round(values[0]);
            offset = 1;
        }

        double a = values[offset];
        double b = values[offset + 1];
        double c = values[offset + 2];
        Double yaw = values.length - offset >= 4 ? values[offset + 3] : null;
        double quality = values.length - offset >= 5 ? values[offset + 4] : 1.0;
        quality = Math.max(0.0, Math.min(1.0, quality));

        if (looksLikeLatLon(a, b)) {
            return new PositionFix(timestampMillis, null, null, null, a, b, c, yaw, quality, packetId);
        }
        return new PositionFix(timestampMillis, a, b, c, null, null, null, yaw, quality, packetId);
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

    private boolean looksLikePacketId(double value) {
        return value >= 0.0 && Math.abs(value - Math.round(value)) < 1e-6;
    }

    private boolean looksLikeLatLon(double latitude, double longitude) {
        boolean inRange = latitude >= -90.0 && latitude <= 90.0 && longitude >= -180.0 && longitude <= 180.0;
        boolean likelyGeographic = Math.abs(latitude) > 10.0 || Math.abs(longitude) > 20.0;
        return inRange && likelyGeographic;
    }
}
