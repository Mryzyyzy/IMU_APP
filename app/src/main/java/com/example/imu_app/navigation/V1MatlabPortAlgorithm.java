package com.example.imu_app.navigation;

import com.example.imu_app.coordinate.CoordinateConverter;
import com.example.imu_app.model.ImuSample;
import com.example.imu_app.model.TrackPoint;

import java.util.ArrayList;
import java.util.List;

final class V1MatlabPortAlgorithm {
    private static final double GRAVITY_MPS2 = 9.80665;
    private static final double SAMPLE_RATE_HZ = 100.0;
    private static final int ALIGNMENT_SAMPLE_COUNT = 300;
    private static final double MAX_SPEED_MPS = 20.0;
    private static final double ZUPT_ACCEL_THRESHOLD_MPS2 = 0.18;
    private static final double ZUPT_GYRO_THRESHOLD_RADPS = 0.035;
    private static final double INITIAL_YAW_DEG = 180.0;

    private static final double[][] INSTALLATION_MATRIX = {
            {0.0, -1.0, 0.0},
            {1.0, 0.0, 0.0},
            {0.0, 0.0, 1.0}
    };
    private static final double[] GYRO_BIAS_RADPS = {
            -0.0006981317007977318,
            -0.0005235987755982988,
            0.0
    };

    private double referenceLatitude;
    private double referenceLongitude;
    private double referenceAltitude;
    private final List<CalibratedSample> alignmentSamples = new ArrayList<>();
    private final State state = new State();
    private Quaternion attitude = Quaternion.fromEuler(0.0, 0.0, Math.toRadians(INITIAL_YAW_DEG));

    void reset(double referenceLatitude, double referenceLongitude, double referenceAltitude) {
        this.referenceLatitude = referenceLatitude;
        this.referenceLongitude = referenceLongitude;
        this.referenceAltitude = referenceAltitude;
        alignmentSamples.clear();
        state.reset();
        state.yawDeg = INITIAL_YAW_DEG;
        attitude = Quaternion.fromEuler(0.0, 0.0, Math.toRadians(INITIAL_YAW_DEG));
    }

    TrackPoint process(ImuSample rawSample) {
        CalibratedSample sample = calibrate(rawSample);
        state.sampleCount++;
        boolean stationary = isStationary(sample);
        state.stationary = stationary;

        if (!state.aligned) {
            double[] alignment = addAlignmentSample(sample);
            state.timestampMillis = sample.timestampMillis;
            if (alignment == null) {
                return null;
            }
            attitude = Quaternion.fromEuler(alignment[0], alignment[1], alignment[2]);
            updateEuler();
            state.aligned = true;
            return pointFromState(sample.timestampMillis, state, "ImuNavigationEngine/v1_matlab_port", 0.85);
        }

        double dt = sampleDt(sample.timestampMillis, state.timestampMillis);
        attitude = attitude.integrate(sample.gx, sample.gy, sample.gz, dt);
        updateEuler();
        double[] accelerationEnu = rotate(attitude.toMatrix(), new double[]{sample.ax, sample.ay, sample.az});
        accelerationEnu[2] -= GRAVITY_MPS2;
        integrate(accelerationEnu, dt);
        applyConstraints();
        boolean limited = limitSpeed();
        if (stationary) {
            state.velocityE = 0.0;
            state.velocityN = 0.0;
            state.velocityU = 0.0;
        }
        state.timestampMillis = sample.timestampMillis;
        return pointFromState(sample.timestampMillis, state, "ImuNavigationEngine/v1_matlab_port", limited ? 0.72 : 0.92);
    }

    List<TrackPoint> processBatch(List<ImuSample> samples) {
        List<TrackPoint> points = new ArrayList<>();
        for (ImuSample sample : samples) {
            TrackPoint point = process(sample);
            if (point != null) {
                points.add(point);
            }
        }
        return points;
    }

    private CalibratedSample calibrate(ImuSample sample) {
        double[] acc = matVec(INSTALLATION_MATRIX, new double[]{sample.ax, sample.ay, sample.az});
        double[] gyro = matVec(INSTALLATION_MATRIX, new double[]{
                sample.gx - GYRO_BIAS_RADPS[0],
                sample.gy - GYRO_BIAS_RADPS[1],
                sample.gz - GYRO_BIAS_RADPS[2]
        });
        return new CalibratedSample(
                sample.timestampMillis,
                acc[0],
                acc[1],
                acc[2],
                gyro[0],
                gyro[1],
                gyro[2],
                sample.mx,
                sample.my,
                sample.mz
        );
    }

    private TrackPoint pointFromState(long timestampMillis, State state, String source, double quality) {
        double[] wgs84 = CoordinateConverter.enuToWgs84(
                state.east,
                state.north,
                state.up,
                referenceLatitude,
                referenceLongitude,
                referenceAltitude
        );
        double speed = norm(state.velocityE, state.velocityN, state.velocityU);
        return new TrackPoint(
                timestampMillis,
                wgs84[0],
                wgs84[1],
                wgs84[2],
                state.east,
                state.north,
                state.up,
                state.velocityE,
                state.velocityN,
                state.velocityU,
                speed,
                state.rollDeg,
                state.pitchDeg,
                state.yawDeg,
                source,
                quality
        );
    }

    private double[] addAlignmentSample(CalibratedSample sample) {
        alignmentSamples.add(sample);
        if (alignmentSamples.size() < ALIGNMENT_SAMPLE_COUNT) {
            return null;
        }
        double ax = 0.0;
        double ay = 0.0;
        double az = 0.0;
        for (CalibratedSample item : alignmentSamples) {
            ax += item.ax;
            ay += item.ay;
            az += item.az;
        }
        ax /= alignmentSamples.size();
        ay /= alignmentSamples.size();
        az /= alignmentSamples.size();
        double roll = Math.atan2(ay, az);
        double pitch = Math.atan2(-ax, Math.sqrt(ay * ay + az * az));
        double yaw = Math.toRadians(INITIAL_YAW_DEG);
        return new double[]{roll, pitch, yaw};
    }

    private boolean isStationary(CalibratedSample sample) {
        double accNorm = norm(sample.ax, sample.ay, sample.az);
        double gyroNorm = norm(sample.gx, sample.gy, sample.gz);
        return Math.abs(accNorm - GRAVITY_MPS2) <= ZUPT_ACCEL_THRESHOLD_MPS2
                && gyroNorm <= ZUPT_GYRO_THRESHOLD_RADPS;
    }

    private void integrate(double[] accelerationEnu, double dt) {
        if (dt <= 0.0) {
            return;
        }
        state.east += state.velocityE * dt + 0.5 * accelerationEnu[0] * dt * dt;
        state.north += state.velocityN * dt + 0.5 * accelerationEnu[1] * dt * dt;
        state.up += state.velocityU * dt + 0.5 * accelerationEnu[2] * dt * dt;
        state.velocityE += accelerationEnu[0] * dt;
        state.velocityN += accelerationEnu[1] * dt;
        state.velocityU += accelerationEnu[2] * dt;
    }

    private void applyConstraints() {
        state.velocityU = 0.0;
        double yaw = Math.toRadians(state.yawDeg == null ? 0.0 : state.yawDeg);
        double forwardE = Math.cos(yaw);
        double forwardN = Math.sin(yaw);
        double forwardSpeed = state.velocityE * forwardE + state.velocityN * forwardN;
        state.velocityE = forwardSpeed * forwardE;
        state.velocityN = forwardSpeed * forwardN;
    }

    private boolean limitSpeed() {
        double speed = norm(state.velocityE, state.velocityN, state.velocityU);
        if (speed <= MAX_SPEED_MPS || speed <= 0.0) {
            return false;
        }
        double scale = MAX_SPEED_MPS / speed;
        state.velocityE *= scale;
        state.velocityN *= scale;
        state.velocityU *= scale;
        return true;
    }

    private void updateEuler() {
        double[] euler = attitude.eulerRad();
        state.rollDeg = Math.toDegrees(euler[0]);
        state.pitchDeg = Math.toDegrees(euler[1]);
        state.yawDeg = normalizeDegrees(Math.toDegrees(euler[2]));
    }

    private static double sampleDt(long timestampMillis, long previousTimestampMillis) {
        if (previousTimestampMillis <= 0L) {
            return 1.0 / SAMPLE_RATE_HZ;
        }
        double dt = (timestampMillis - previousTimestampMillis) / 1000.0;
        if (dt <= 0.0 || !Double.isFinite(dt)) {
            return 1.0 / SAMPLE_RATE_HZ;
        }
        return Math.min(dt, 1.0);
    }

    private static double[] matVec(double[][] matrix, double[] vector) {
        return new double[]{
                matrix[0][0] * vector[0] + matrix[0][1] * vector[1] + matrix[0][2] * vector[2],
                matrix[1][0] * vector[0] + matrix[1][1] * vector[1] + matrix[1][2] * vector[2],
                matrix[2][0] * vector[0] + matrix[2][1] * vector[1] + matrix[2][2] * vector[2]
        };
    }

    private static double[] rotate(double[][] matrix, double[] vector) {
        return matVec(matrix, vector);
    }

    private static double norm(double x, double y, double z) {
        return Math.sqrt(x * x + y * y + z * z);
    }

    private static double normalizeDegrees(double degrees) {
        double normalized = degrees % 360.0;
        return normalized < 0.0 ? normalized + 360.0 : normalized;
    }

    private static final class State {
        long timestampMillis = 0L;
        double east = 0.0;
        double north = 0.0;
        double up = 0.0;
        double velocityE = 0.0;
        double velocityN = 0.0;
        double velocityU = 0.0;
        Double rollDeg = 0.0;
        Double pitchDeg = 0.0;
        Double yawDeg = INITIAL_YAW_DEG;
        boolean aligned = false;
        boolean stationary = false;
        int sampleCount = 0;

        void reset() {
            timestampMillis = 0L;
            east = 0.0;
            north = 0.0;
            up = 0.0;
            velocityE = 0.0;
            velocityN = 0.0;
            velocityU = 0.0;
            rollDeg = 0.0;
            pitchDeg = 0.0;
            yawDeg = INITIAL_YAW_DEG;
            aligned = false;
            stationary = false;
            sampleCount = 0;
        }
    }

    private static final class CalibratedSample {
        final long timestampMillis;
        final double ax;
        final double ay;
        final double az;
        final double gx;
        final double gy;
        final double gz;
        final Double mx;
        final Double my;
        final Double mz;

        CalibratedSample(long timestampMillis, double ax, double ay, double az, double gx, double gy, double gz, Double mx, Double my, Double mz) {
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
        }
    }

    private static final class Quaternion {
        final double w;
        final double x;
        final double y;
        final double z;

        Quaternion(double w, double x, double y, double z) {
            this.w = w;
            this.x = x;
            this.y = y;
            this.z = z;
        }

        static Quaternion fromEuler(double roll, double pitch, double yaw) {
            double cr = Math.cos(roll * 0.5);
            double sr = Math.sin(roll * 0.5);
            double cp = Math.cos(pitch * 0.5);
            double sp = Math.sin(pitch * 0.5);
            double cy = Math.cos(yaw * 0.5);
            double sy = Math.sin(yaw * 0.5);
            return new Quaternion(
                    cy * cp * cr + sy * sp * sr,
                    cy * cp * sr - sy * sp * cr,
                    sy * cp * sr + cy * sp * cr,
                    sy * cp * cr - cy * sp * sr
            ).normalized();
        }

        Quaternion integrate(double gx, double gy, double gz, double dt) {
            if (dt <= 0.0) {
                return this;
            }
            Quaternion derivative = new Quaternion(
                    -x * gx - y * gy - z * gz,
                    w * gx + y * gz - z * gy,
                    w * gy - x * gz + z * gx,
                    w * gz + x * gy - y * gx
            );
            double halfDt = 0.5 * dt;
            return new Quaternion(
                    w + derivative.w * halfDt,
                    x + derivative.x * halfDt,
                    y + derivative.y * halfDt,
                    z + derivative.z * halfDt
            ).normalized();
        }

        Quaternion normalized() {
            double norm = Math.sqrt(w * w + x * x + y * y + z * z);
            if (norm <= 0.0 || !Double.isFinite(norm)) {
                return new Quaternion(1.0, 0.0, 0.0, 0.0);
            }
            return new Quaternion(w / norm, x / norm, y / norm, z / norm);
        }

        double[][] toMatrix() {
            Quaternion q = normalized();
            double ww = q.w * q.w;
            double xx = q.x * q.x;
            double yy = q.y * q.y;
            double zz = q.z * q.z;
            double wx = q.w * q.x;
            double wy = q.w * q.y;
            double wz = q.w * q.z;
            double xy = q.x * q.y;
            double xz = q.x * q.z;
            double yz = q.y * q.z;
            return new double[][]{
                    {ww + xx - yy - zz, 2.0 * (xy - wz), 2.0 * (xz + wy)},
                    {2.0 * (xy + wz), ww - xx + yy - zz, 2.0 * (yz - wx)},
                    {2.0 * (xz - wy), 2.0 * (yz + wx), ww - xx - yy + zz}
            };
        }

        double[] eulerRad() {
            Quaternion q = normalized();
            double roll = Math.atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y));
            double sinPitch = 2.0 * (q.w * q.y - q.z * q.x);
            double pitch = Math.asin(Math.max(-1.0, Math.min(1.0, sinPitch)));
            double yaw = Math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
            return new double[]{roll, pitch, yaw};
        }
    }
}
