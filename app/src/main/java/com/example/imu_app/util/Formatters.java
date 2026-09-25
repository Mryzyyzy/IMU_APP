package com.example.imu_app.util;

import java.util.Locale;

public final class Formatters {
    private Formatters() {
    }

    public static String oneDecimal(double value) {
        return String.format(Locale.US, "%.1f", value);
    }

    public static String twoDecimals(double value) {
        return String.format(Locale.US, "%.2f", value);
    }

    public static String optional(Double value, int digits) {
        if (value == null || value.isNaN() || value.isInfinite()) {
            return "--";
        }
        return String.format(Locale.US, "%." + digits + "f", value);
    }

    public static String meters(double value) {
        return twoDecimals(value) + " m";
    }

    public static String speed(double value) {
        return twoDecimals(value) + " m/s";
    }

    public static String degrees(Double value) {
        if (value == null || value.isNaN() || value.isInfinite()) {
            return "--";
        }
        return oneDecimal(value) + "°";
    }

    public static String percent(double value) {
        return String.format(Locale.US, "%.0f%%", value * 100.0);
    }
}
