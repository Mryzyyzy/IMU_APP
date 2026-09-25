package com.example.imu_app.coordinate;

public final class CoordinateConverter {
    private static final double WGS84_A = 6378137.0;
    private static final double WGS84_F = 1.0 / 298.257223563;
    private static final double WGS84_E2 = WGS84_F * (2.0 - WGS84_F);

    private CoordinateConverter() {
    }

    public static double[] wgs84ToEnu(
            double latitude,
            double longitude,
            double altitude,
            double referenceLatitude,
            double referenceLongitude,
            double referenceAltitude
    ) {
        double[] radii = curvatureRadii(referenceLatitude);
        double lat0Rad = Math.toRadians(referenceLatitude);
        double east = Math.toRadians(longitude - referenceLongitude) * radii[1] * Math.cos(lat0Rad);
        double north = Math.toRadians(latitude - referenceLatitude) * radii[0];
        double up = altitude - referenceAltitude;
        return new double[]{east, north, up};
    }

    public static double[] enuToWgs84(
            double east,
            double north,
            double up,
            double referenceLatitude,
            double referenceLongitude,
            double referenceAltitude
    ) {
        double[] radii = curvatureRadii(referenceLatitude);
        double lat0Rad = Math.toRadians(referenceLatitude);
        double latitude = referenceLatitude + Math.toDegrees(north / radii[0]);
        double longitude = referenceLongitude + Math.toDegrees(east / (radii[1] * Math.cos(lat0Rad)));
        double altitude = referenceAltitude + up;
        return new double[]{latitude, longitude, altitude};
    }

    private static double[] curvatureRadii(double latitude) {
        double latRad = Math.toRadians(latitude);
        double sinLat = Math.sin(latRad);
        double denominator = 1.0 - WGS84_E2 * sinLat * sinLat;
        double primeVertical = WGS84_A / Math.sqrt(denominator);
        double meridianRadius = WGS84_A * (1.0 - WGS84_E2) / Math.pow(denominator, 1.5);
        return new double[]{meridianRadius, primeVertical};
    }
}
