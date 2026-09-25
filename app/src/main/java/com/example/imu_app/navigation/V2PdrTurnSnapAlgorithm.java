package com.example.imu_app.navigation;

import com.example.imu_app.coordinate.CoordinateConverter;
import com.example.imu_app.model.ImuSample;
import com.example.imu_app.model.TrackPoint;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

final class V2PdrTurnSnapAlgorithm {
    private static final double GRAVITY_MPS2 = 9.80665;
    private static final double SAMPLE_RATE_HZ = 100.0;
    private static final int ALIGNMENT_SAMPLE_COUNT = 300;
    private static final double INITIAL_YAW_DEG = 180.0;
    private static final double TOTAL_DISTANCE_M = 67.2;
    private static final double TURN_SIGN = -1.0;
    private static final int SMOOTH_WIN_ACC = 5;
    private static final double STEP_MIN_PROM = 0.5;
    private static final int STEP_MIN_DIST = 30;
    private static final int SMOOTH_WIN_GZ = 8;
    private static final double TURN_RATE_THRESHOLD_DPS = 15.0;
    private static final double TURN_MIN_DURATION_SEC = 0.5;
    private static final double TURN_MIN_ANGLE_DEG = 45.0;

    private double referenceLatitude;
    private double referenceLongitude;
    private double referenceAltitude;
    private final List<ImuSample> samples = new ArrayList<>();
    private int lastBatchPointCount = 0;

    void reset(double referenceLatitude, double referenceLongitude, double referenceAltitude) {
        this.referenceLatitude = referenceLatitude;
        this.referenceLongitude = referenceLongitude;
        this.referenceAltitude = referenceAltitude;
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

    List<TrackPoint> processBatch(List<ImuSample> batch) {
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

        List<Double> accNormSmoothed = movingAverage(accNorm, SMOOTH_WIN_ACC);
        double gyroBias = estimateYawBias(gzDps);
        List<Double> gzCorrected = new ArrayList<>();
        for (double value : gzDps) {
            gzCorrected.add(value - gyroBias);
        }

        List<Double> yawRaw = integrateYaw(gzCorrected, dt);
        List<Integer> stepIndexes = findPeaks(accNormSmoothed, GRAVITY_MPS2 + STEP_MIN_PROM, STEP_MIN_DIST);
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
        for (int i = 1; i < gzCorrected.size(); i++) {
            yaw.add(yaw.get(i - 1) + TURN_SIGN * 0.5 * (gzCorrected.get(i - 1) + gzCorrected.get(i)) * dt);
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
        double scale = TOTAL_DISTANCE_M / Math.max(shapeSum, 1e-9);
        List<Double> lengths = new ArrayList<>();
        for (double value : rawShape) {
            lengths.add(scale * value);
        }
        return lengths;
    }

    private List<TurnSegment> detectTurns(List<Double> gzCorrected, double dt) {
        List<Double> gzSmoothed = movingAverage(gzCorrected, SMOOTH_WIN_GZ);
        List<TurnSegment> turns = new ArrayList<>();
        boolean inSegment = false;
        int start = 0;
        for (int index = 0; index < gzSmoothed.size(); index++) {
            boolean active = Math.abs(gzSmoothed.get(index)) > TURN_RATE_THRESHOLD_DPS;
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
                if (duration >= TURN_MIN_DURATION_SEC && Math.abs(rawAngle) >= TURN_MIN_ANGLE_DEG) {
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
        for (TurnSegment turn : turns) {
            heading += TURN_SIGN * turn.snapAngle;
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

    private static double norm(double x, double y, double z) {
        return Math.sqrt(x * x + y * y + z * z);
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
}
