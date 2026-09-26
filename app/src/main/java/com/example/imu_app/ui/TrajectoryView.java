package com.example.imu_app.ui;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Path;
import android.graphics.RectF;
import android.util.AttributeSet;
import android.view.MotionEvent;
import android.view.ScaleGestureDetector;
import android.view.View;

import com.example.imu_app.model.TrackPoint;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

public class TrajectoryView extends View {
    private final Paint paint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final RectF plotRect = new RectF();
    private final List<TrackPoint> points = new ArrayList<>();
    private ScaleGestureDetector scaleDetector;
    private double zoomFactor = 1.0;

    public TrajectoryView(Context context) {
        super(context);
        init();
    }

    public TrajectoryView(Context context, AttributeSet attrs) {
        super(context, attrs);
        init();
    }

    private void init() {
        setMinimumHeight(dp(300));
        scaleDetector = new ScaleGestureDetector(getContext(), new ScaleListener());
    }

    public void setTrack(List<TrackPoint> newPoints) {
        points.clear();
        points.addAll(newPoints);
        invalidate();
    }

    public void clear() {
        points.clear();
        invalidate();
    }

    private void setZoomFactor(double value) {
        zoomFactor = Math.max(0.25, Math.min(8.0, value));
        invalidate();
    }

    @Override
    public boolean onTouchEvent(MotionEvent event) {
        scaleDetector.onTouchEvent(event);
        return true;
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);
        drawBackground(canvas);
        drawGrid(canvas);
        if (points.isEmpty()) {
            drawEmptyState(canvas);
            return;
        }
        Bounds bounds = bounds();
        drawScale(canvas, bounds);
        drawAxes(canvas, bounds);
        drawTrack(canvas, bounds);
        drawCurrentMarker(canvas, bounds);
    }

    private void drawBackground(Canvas canvas) {
        canvas.drawColor(Color.WHITE);
        plotRect.set(dp(48), dp(20), getWidth() - dp(16), getHeight() - dp(48));
        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeWidth(dp(1));
        paint.setColor(Color.rgb(217, 225, 236));
        canvas.drawRoundRect(plotRect, dp(8), dp(8), paint);
    }

    private void drawGrid(Canvas canvas) {
        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeWidth(dp(1));
        paint.setColor(Color.rgb(215, 224, 235));
        for (int i = 1; i < 4; i++) {
            float x = plotRect.left + plotRect.width() * i / 4f;
            canvas.drawLine(x, plotRect.top, x, plotRect.bottom, paint);
            float y = plotRect.top + plotRect.height() * i / 4f;
            canvas.drawLine(plotRect.left, y, plotRect.right, y, paint);
        }
    }

    private void drawEmptyState(Canvas canvas) {
        paint.setStyle(Paint.Style.FILL);
        paint.setTextAlign(Paint.Align.CENTER);
        paint.setTextSize(sp(15));
        paint.setColor(Color.rgb(118, 131, 152));
        canvas.drawText("等待 IMU 数据", plotRect.centerX(), plotRect.centerY(), paint);
    }

    private void drawTrack(Canvas canvas, Bounds bounds) {
        if (points.size() < 2) {
            return;
        }
        Path path = new Path();
        float[] start = map(points.get(0), bounds);
        path.moveTo(start[0], start[1]);
        for (int i = 1; i < points.size(); i++) {
            float[] point = map(points.get(i), bounds);
            path.lineTo(point[0], point[1]);
        }

        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeCap(Paint.Cap.ROUND);
        paint.setStrokeJoin(Paint.Join.ROUND);
        paint.setStrokeWidth(dp(8));
        paint.setColor(Color.argb(90, 29, 111, 233));
        canvas.drawPath(path, paint);

        paint.setStrokeWidth(dp(3));
        paint.setColor(Color.rgb(29, 111, 233));
        canvas.drawPath(path, paint);
    }

    private void drawCurrentMarker(Canvas canvas, Bounds bounds) {
        TrackPoint current = points.get(points.size() - 1);
        float[] mapped = map(current, bounds);
        float x = mapped[0];
        float y = mapped[1];
        double yawRadians = Math.toRadians(current.yaw == null ? 0.0 : current.yaw);
        float dx = (float) Math.cos(yawRadians);
        float dy = (float) -Math.sin(yawRadians);

        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeWidth(dp(2));
        paint.setColor(Color.rgb(100, 116, 139));
        canvas.drawLine(x, y, x + dx * dp(32), y + dy * dp(32), paint);

        paint.setStyle(Paint.Style.FILL);
        paint.setColor(Color.rgb(10, 159, 77));
        canvas.drawCircle(x, y, dp(7), paint);
        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeWidth(dp(2));
        paint.setColor(Color.WHITE);
        canvas.drawCircle(x, y, dp(7), paint);

        Path arrow = new Path();
        arrow.moveTo(x + dx * dp(22), y + dy * dp(22));
        arrow.lineTo(x - dy * dp(7), y + dx * dp(7));
        arrow.lineTo(x + dy * dp(7), y - dx * dp(7));
        arrow.close();
        paint.setStyle(Paint.Style.FILL);
        paint.setColor(Color.rgb(29, 111, 233));
        canvas.drawPath(arrow, paint);
    }

    private void drawScale(Canvas canvas, Bounds bounds) {
        double span = Math.max(bounds.maxEast - bounds.minEast, 1.0);
        double scale = niceScale(span);
        float length = (float) Math.min(scale / span * plotRect.width(), plotRect.width() * 0.32f);
        float x0 = plotRect.right - length - dp(18);
        float y = plotRect.bottom - dp(18);

        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeWidth(dp(2));
        paint.setColor(Color.rgb(23, 32, 51));
        canvas.drawLine(x0, y, x0 + length, y, paint);
        canvas.drawLine(x0, y - dp(5), x0, y + dp(5), paint);
        canvas.drawLine(x0 + length, y - dp(5), x0 + length, y + dp(5), paint);

        paint.setStyle(Paint.Style.FILL);
        paint.setTextAlign(Paint.Align.CENTER);
        paint.setTextSize(sp(11));
        paint.setColor(Color.rgb(71, 85, 105));
        canvas.drawText(String.format(Locale.US, "%.0f m", scale), x0 + length / 2f, y - dp(8), paint);
    }

    private void drawAxes(Canvas canvas, Bounds bounds) {
        paint.setStyle(Paint.Style.FILL);
        paint.setTextSize(sp(10));
        paint.setColor(Color.rgb(71, 85, 105));

        double minEast = bounds.minEast;
        double midEast = (bounds.minEast + bounds.maxEast) * 0.5;
        double maxEast = bounds.maxEast;
        double minNorth = bounds.minNorth;
        double midNorth = (bounds.minNorth + bounds.maxNorth) * 0.5;
        double maxNorth = bounds.maxNorth;

        float yLabel = plotRect.bottom + dp(16);
        paint.setTextAlign(Paint.Align.LEFT);
        canvas.drawText(formatAxis(minEast), plotRect.left, yLabel, paint);
        paint.setTextAlign(Paint.Align.CENTER);
        canvas.drawText(formatAxis(midEast), plotRect.centerX(), yLabel, paint);
        paint.setTextAlign(Paint.Align.RIGHT);
        canvas.drawText(formatAxis(maxEast), plotRect.right, yLabel, paint);

        paint.setTextAlign(Paint.Align.RIGHT);
        canvas.drawText(formatAxis(maxNorth), plotRect.left - dp(6), plotRect.top + dp(4), paint);
        canvas.drawText(formatAxis(midNorth), plotRect.left - dp(6), plotRect.centerY() + dp(4), paint);
        canvas.drawText(formatAxis(minNorth), plotRect.left - dp(6), plotRect.bottom, paint);

        paint.setTextAlign(Paint.Align.RIGHT);
        paint.setTextSize(sp(11));
        paint.setColor(Color.rgb(23, 32, 51));
        canvas.drawText("E / m", plotRect.right, plotRect.bottom + dp(34), paint);
        paint.setTextAlign(Paint.Align.LEFT);
        canvas.drawText("N / m", plotRect.left, plotRect.top - dp(6), paint);
    }

    private Bounds bounds() {
        double minEast = Double.POSITIVE_INFINITY;
        double maxEast = Double.NEGATIVE_INFINITY;
        double minNorth = Double.POSITIVE_INFINITY;
        double maxNorth = Double.NEGATIVE_INFINITY;
        for (TrackPoint point : points) {
            minEast = Math.min(minEast, point.east);
            maxEast = Math.max(maxEast, point.east);
            minNorth = Math.min(minNorth, point.north);
            maxNorth = Math.max(maxNorth, point.north);
        }
        double centerEast = (minEast + maxEast) / 2.0;
        double centerNorth = (minNorth + maxNorth) / 2.0;
        double span = Math.max(Math.max(maxEast - minEast, maxNorth - minNorth), 20.0) / zoomFactor;
        double pad = span * 0.24;
        double aspect = Math.max(plotRect.width(), 1f) / Math.max(plotRect.height(), 1f);
        double halfNorth = span / 2.0 + pad;
        double halfEast = halfNorth * aspect;
        return new Bounds(
                centerEast - halfEast,
                centerEast + halfEast,
                centerNorth - halfNorth,
                centerNorth + halfNorth
        );
    }

    private float[] map(TrackPoint point, Bounds bounds) {
        double eastSpan = Math.max(bounds.maxEast - bounds.minEast, 1e-9);
        double northSpan = Math.max(bounds.maxNorth - bounds.minNorth, 1e-9);
        float x = (float) (plotRect.left + (point.east - bounds.minEast) / eastSpan * plotRect.width());
        float y = (float) (plotRect.bottom - (point.north - bounds.minNorth) / northSpan * plotRect.height());
        return new float[]{x, y};
    }

    private double niceScale(double span) {
        double[] candidates = {5, 10, 20, 50, 100, 200, 500, 1000};
        double target = span / 5.0;
        double best = candidates[0];
        for (double candidate : candidates) {
            if (Math.abs(candidate - target) < Math.abs(best - target)) {
                best = candidate;
            }
        }
        return best;
    }

    private String formatAxis(double value) {
        double abs = Math.abs(value);
        if (abs >= 1000.0) {
            return String.format(Locale.US, "%.1fk", value / 1000.0);
        }
        if (abs >= 100.0) {
            return String.format(Locale.US, "%.0f", value);
        }
        if (abs >= 10.0) {
            return String.format(Locale.US, "%.1f", value);
        }
        return String.format(Locale.US, "%.2f", value);
    }

    private int dp(float value) {
        return (int) (value * getResources().getDisplayMetrics().density + 0.5f);
    }

    private float sp(float value) {
        return value * getResources().getDisplayMetrics().scaledDensity;
    }

    private static class Bounds {
        final double minEast;
        final double maxEast;
        final double minNorth;
        final double maxNorth;

        Bounds(double minEast, double maxEast, double minNorth, double maxNorth) {
            this.minEast = minEast;
            this.maxEast = maxEast;
            this.minNorth = minNorth;
            this.maxNorth = maxNorth;
        }
    }

    private final class ScaleListener extends ScaleGestureDetector.SimpleOnScaleGestureListener {
        @Override
        public boolean onScale(ScaleGestureDetector detector) {
            setZoomFactor(zoomFactor * detector.getScaleFactor());
            return true;
        }
    }
}
