package com.example.imu_app.navigation;

import com.example.imu_app.coordinate.CoordinateConverter;
import com.example.imu_app.model.ImuSample;
import com.example.imu_app.model.TrackPoint;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class ImuNavigationProcessor {
    public static final String V1_MATLAB_PORT = "v1_matlab_port";
    public static final String V2_PDR_TURN_SNAP = "v2_pdr_turn_snap";

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

    private String algorithmVersion = V1_MATLAB_PORT;
    private double referenceLatitude = 30.659462;
    private double referenceLongitude = 104.065735;
    private double referenceAltitude = 482.0;
    private final V1MatlabPort v1 = new V1MatlabPort();
    private final V2PdrTurnSnap v2 = new V2PdrTurnSnap();

    public void reset(double referenceLatitude, double referenceLongitude, double referenceAltitude, String algorithmVersion) {
        this.referenceLatitude = referenceLatitude;
        this.referenceLongitude = referenceLongitude;
        this.referenceAltitude = referenceAltitude;
        this.algorithmVersion = normalizeAlgorithmVersion(algorithmVersion);
        v1.reset();
        v2.reset();
    }

    public TrackPoint process(ImuSample sample) {
        if (V2_PDR_TURN_SNAP.equals(algorithmVersion)) {
            return v2.process(sample);
        }
        return v1.process(sample);
    }

    public List<TrackPoint> processBatch(List<ImuSample> samples) {
        if (samples.isEmpty()) {
            return Collections.emptyList();
        }
        reset(referenceLatitude, referenceLongitude, referenceAltitude, algorithmVersion);
        if (V2_PDR_TURN_SNAP.equals(algorithmVersion)) {
            return v2.processBatch(samples);
        }
        List<TrackPoint> points = new ArrayList<>();
        for (ImuSample sample : samples) {
            TrackPoint point = v1.process(sample);
            if (point != null) {
                points.add(point);
            }
        }
        return points;
    }

    public static String normalizeAlgorithmVersion(String value) {
        if (V2_PDR_TURN_SNAP.equals(value)) {
            return V2_PDR_TURN_SNAP;
        }
        return V1_MATLAB_PORT;
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

    private final class V1MatlabPort {
        private final List<CalibratedSample> alignmentSamples = new ArrayList<>();
        private final State state = new State();
        private Quaternion attitude = Quaternion.fromEuler(0.0, 0.0, Math.toRadians(INITIAL_YAW_DEG));

        void reset() {
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
    }

    private final class V2PdrTurnSnap {
        private final List<ImuSample> samples = new ArrayList<>();
        private int lastBatchPointCount = 0;

        void reset() {
            samples.clear();
            lastBatchPointCount = 0;
        }

        TrackPoint process(ImuSample sample) {
            samples.add(sample);
            if (samples.size() < ALIGNMENT_SAMPLE_COUNT) {
                return null;
            }
            if (samples.size() % Math.max(20, (int) SAMPLE_RATE_HZ) != 0) {
                return null;
            }
            List<TrackPoint> points = processBatch(samples);
            if (points.size() <= lastBatchPointCount) {
                return null;
            }
            TrackPoint point = points.get(points.size() - 1);
            lastBatchPointCount = points.size();
            return point;
        }

        private List<TrackPoint> processBatch(List<ImuSample> batch) {
            if (batch.isEmpty()) {
                return Collections.emptyList();
            }
            double dt = 1.0 / SAMPLE_RATE_HZ;
            List<Double> accNorm = new ArrayList<>();
            List<Double> gzDps = new ArrayList<>();
            for (ImuSample sample : batch) {
                accNorm.add(norm(sample.ax, sample.ay, sample.az));
                gzDps.add(Math.toDegrees(sample.gz));
            }

            List<Double> accNormSmoothed = movingAverage(accNorm, 5);
            double gyroBias = estimateYawBias(gzDps);
            List<Double> gzCorrected = new ArrayList<>();
            for (double value : gzDps) {
                gzCorrected.add(value - gyroBias);
            }

            List<Double> yawRaw = integrateYaw(gzCorrected, dt);
            List<Integer> stepIndexes = findPeaks(accNormSmoothed, GRAVITY_MPS2 + 0.5, 30);
            List<Double> stepLengths = stepLengths(accNormSmoothed, stepIndexes);
            List<TurnSegment> turns = detectTurns(gzCorrected, dt);
            List<Double> headingUsed = headingProfile(yawRaw, turns, batch.size());
            return buildTrackPoints(batch, stepIndexes, stepLengths, headingUsed);
        }

        private double estimateYawBias(List<Double> gzDps) {
            int window = Math.min(Math.max(1, (int) Math.round(3.0 * SAMPLE_RATE_HZ)), gzDps.size());
            if (window == gzDps.size()) {
                return mean(gzDps);
            }
            int step = Math.max(1, (int) Math.round(0.5 * SAMPLE_RATE_HZ));
            double bestStd = Double.POSITIVE_INFINITY;
            int bestStart = 0;
            for (int start = 0; start <= gzDps.size() - window; start += step) {
                List<Double> segment = gzDps.subList(start, start + window);
                double std = std(segment);
                if (std < bestStd) {
                    bestStd = std;
                    bestStart = start;
                }
            }
            return mean(gzDps.subList(bestStart, bestStart + window));
        }

        private List<Double> integrateYaw(List<Double> gzCorrected, double dt) {
            List<Double> yaw = new ArrayList<>();
            yaw.add(INITIAL_YAW_DEG);
            double turnSign = -1.0;
            for (int i = 1; i < gzCorrected.size(); i++) {
                yaw.add(yaw.get(i - 1) + turnSign * 0.5 * (gzCorrected.get(i - 1) + gzCorrected.get(i)) * dt);
            }
            return yaw;
        }

        private List<Double> stepLengths(List<Double> accNormSmoothed, List<Integer> stepIndexes) {
            if (stepIndexes.isEmpty()) {
                return Collections.emptyList();
            }
            List<Integer> bounds = new ArrayList<>();
            bounds.add(0);
            bounds.addAll(stepIndexes);
            List<Double> rawAmp = new ArrayList<>();
            for (int index = 0; index < stepIndexes.size(); index++) {
                int start = bounds.get(index);
                int end = bounds.get(index + 1);
                double min = Double.POSITIVE_INFINITY;
                double max = Double.NEGATIVE_INFINITY;
                for (int i = start; i <= end; i++) {
                    double value = accNormSmoothed.get(i);
                    min = Math.min(min, value);
                    max = Math.max(max, value);
                }
                rawAmp.add(Double.isInfinite(min) ? 0.0 : max - min);
            }

            List<Double> positives = new ArrayList<>();
            for (double value : rawAmp) {
                if (value > 0.0) {
                    positives.add(value);
                }
            }
            double replacement = positives.isEmpty() ? 1.0 : median(positives);
            List<Double> rawShape = new ArrayList<>();
            double shapeSum = 0.0;
            for (double value : rawAmp) {
                double shape = Math.pow(Math.max(value, replacement), 0.25);
                rawShape.add(shape);
                shapeSum += shape;
            }
            double scale = 67.2 / Math.max(shapeSum, 1e-9);
            List<Double> lengths = new ArrayList<>();
            for (double value : rawShape) {
                lengths.add(scale * value);
            }
            return lengths;
        }

        private List<TurnSegment> detectTurns(List<Double> gzCorrected, double dt) {
            List<Double> gzSmoothed = movingAverage(gzCorrected, 8);
            List<TurnSegment> turns = new ArrayList<>();
            boolean inSegment = false;
            int start = 0;
            for (int index = 0; index < gzSmoothed.size(); index++) {
                boolean active = Math.abs(gzSmoothed.get(index)) > 15.0;
                if (active && !inSegment) {
                    start = index;
                    inSegment = true;
                }
                if (inSegment && ((!active) || index == gzSmoothed.size() - 1)) {
                    int end = active ? index : index - 1;
                    double duration = (end - start) / SAMPLE_RATE_HZ;
                    double rawAngle = 0.0;
                    for (int i = start; i <= end; i++) {
                        rawAngle += gzCorrected.get(i) * dt;
                    }
                    if (duration >= 0.5 && Math.abs(rawAngle) >= 45.0) {
                        double snapAngle = Math.round(rawAngle / 90.0) * 90.0;
                        turns.add(new TurnSegment(start, end, rawAngle, snapAngle));
                    }
                    inSegment = false;
                }
            }
            return turns;
        }

        private List<Double> headingProfile(List<Double> yawRaw, List<TurnSegment> turns, int sampleCount) {
            if (yawRaw.isEmpty()) {
                return Collections.emptyList();
            }
            List<Integer> boundaries = new ArrayList<>();
            boundaries.add(0);
            for (TurnSegment turn : turns) {
                boundaries.add((int) Math.round((turn.start + turn.end) / 2.0));
            }
            boundaries.add(sampleCount);

            List<Double> anchors = new ArrayList<>();
            anchors.add(INITIAL_YAW_DEG);
            double heading = INITIAL_YAW_DEG;
            double turnSign = -1.0;
            for (TurnSegment turn : turns) {
                heading += turnSign * turn.snapAngle;
                anchors.add(heading);
            }

            List<Double> headingUsed = new ArrayList<>();
            for (int i = 0; i < sampleCount; i++) {
                headingUsed.add(anchors.get(0));
            }
            for (int segmentIndex = 0; segmentIndex < boundaries.size() - 1; segmentIndex++) {
                int start = Math.max(0, Math.min(sampleCount, boundaries.get(segmentIndex)));
                int end = Math.max(start + 1, Math.min(sampleCount, boundaries.get(segmentIndex + 1)));
                double rawMean = mean(yawRaw.subList(start, end));
                double anchor = anchors.get(Math.min(segmentIndex, anchors.size() - 1));
                for (int index = start; index < end; index++) {
                    headingUsed.set(index, anchor + (yawRaw.get(index) - rawMean));
                }
            }
            return headingUsed;
        }

        private List<TrackPoint> buildTrackPoints(
                List<ImuSample> batch,
                List<Integer> stepIndexes,
                List<Double> stepLengths,
                List<Double> headingUsed
        ) {
            double north = 0.0;
            double east = 0.0;
            long previousTime = batch.get(0).timestampMillis;
            List<TrackPoint> points = new ArrayList<>();
            for (int stepNumber = 0; stepNumber < stepIndexes.size(); stepNumber++) {
                int sampleIndex = stepIndexes.get(stepNumber);
                double headingDeg = headingUsed.get(sampleIndex);
                double headingRad = Math.toRadians(headingDeg);
                double stepLength = stepLengths.get(stepNumber);
                north += stepLength * Math.cos(headingRad);
                east += stepLength * Math.sin(headingRad);
                ImuSample sample = batch.get(sampleIndex);
                double stepDt = Math.max((sample.timestampMillis - previousTime) / 1000.0, 1.0 / SAMPLE_RATE_HZ);
                double speed = stepLength / stepDt;
                double velocityN = stepLength * Math.cos(headingRad) / stepDt;
                double velocityE = stepLength * Math.sin(headingRad) / stepDt;
                double[] wgs84 = CoordinateConverter.enuToWgs84(
                        east,
                        north,
                        0.0,
                        referenceLatitude,
                        referenceLongitude,
                        referenceAltitude
                );
                points.add(new TrackPoint(
                        sample.timestampMillis,
                        wgs84[0],
                        wgs84[1],
                        wgs84[2],
                        east,
                        north,
                        0.0,
                        velocityE,
                        velocityN,
                        0.0,
                        speed,
                        null,
                        null,
                        headingDeg,
                        "ImuNavigationEngine/v2_pdr_turn_snap",
                        0.96
                ));
                previousTime = sample.timestampMillis;
            }
            return points;
        }
    }

    private static List<Double> movingAverage(List<Double> values, int window) {
        window = Math.max(1, window);
        if (values.isEmpty()) {
            return Collections.emptyList();
        }
        List<Double> padded = new ArrayList<>();
        int half = window / 2;
        for (int i = 0; i < half; i++) {
            padded.add(values.get(0));
        }
        padded.addAll(values);
        for (int i = 0; i < window - half - 1; i++) {
            padded.add(values.get(values.size() - 1));
        }

        List<Double> result = new ArrayList<>();
        List<Double> queue = new ArrayList<>();
        double runningSum = 0.0;
        for (double value : padded) {
            queue.add(value);
            runningSum += value;
            if (queue.size() > window) {
                runningSum -= queue.remove(0);
            }
            if (queue.size() == window) {
                result.add(runningSum / window);
            }
        }
        return result.subList(0, Math.min(result.size(), values.size()));
    }

    private static List<Integer> findPeaks(List<Double> values, double minHeight, int minDist) {
        List<Integer> peaks = new ArrayList<>();
        for (int index = 1; index < values.size() - 1; index++) {
            if (values.get(index) > values.get(index - 1)
                    && values.get(index) >= values.get(index + 1)
                    && values.get(index) >= minHeight) {
                if (peaks.isEmpty() || index - peaks.get(peaks.size() - 1) >= minDist) {
                    peaks.add(index);
                } else if (values.get(index) > values.get(peaks.get(peaks.size() - 1))) {
                    peaks.set(peaks.size() - 1, index);
                }
            }
        }
        return peaks;
    }

    private static double mean(List<Double> values) {
        if (values.isEmpty()) {
            return 0.0;
        }
        double sum = 0.0;
        for (double value : values) {
            sum += value;
        }
        return sum / values.size();
    }

    private static double std(List<Double> values) {
        if (values.isEmpty()) {
            return 0.0;
        }
        double mean = mean(values);
        double sum = 0.0;
        for (double value : values) {
            double delta = value - mean;
            sum += delta * delta;
        }
        return Math.sqrt(sum / values.size());
    }

    private static double median(List<Double> values) {
        if (values.isEmpty()) {
            return 0.0;
        }
        List<Double> ordered = new ArrayList<>(values);
        Collections.sort(ordered);
        int mid = ordered.size() / 2;
        if (ordered.size() % 2 == 1) {
            return ordered.get(mid);
        }
        return 0.5 * (ordered.get(mid - 1) + ordered.get(mid));
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

    private static final class TurnSegment {
        final int start;
        final int end;
        final double rawAngle;
        final double snapAngle;

        TurnSegment(int start, int end, double rawAngle, double snapAngle) {
            this.start = start;
            this.end = end;
            this.rawAngle = rawAngle;
            this.snapAngle = snapAngle;
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
