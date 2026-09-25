package com.example.imu_app.recording;

import android.content.Context;
import android.os.Environment;

import com.example.imu_app.model.TrackPoint;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.Locale;

public class TrackRecorder {
    private BufferedWriter rawWriter;
    private BufferedWriter trackWriter;
    private File rawFile;
    private File trackFile;
    private boolean recording;

    public void start(Context context) throws IOException {
        stop();
        File directory = context.getExternalFilesDir(Environment.DIRECTORY_DOCUMENTS);
        if (directory == null) {
            directory = new File(context.getFilesDir(), "records");
        }
        if (!directory.exists() && !directory.mkdirs()) {
            throw new IOException("无法创建记录目录: " + directory.getAbsolutePath());
        }
        String stamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
        rawFile = new File(directory, "imu_raw_" + stamp + ".txt");
        trackFile = new File(directory, "track_points_" + stamp + ".csv");
        rawWriter = new BufferedWriter(new FileWriter(rawFile));
        trackWriter = new BufferedWriter(new FileWriter(trackFile));
        trackWriter.write("time_ms,source,east,north,up,velocity_e,velocity_n,velocity_u,speed,roll,pitch,yaw,latitude,longitude,altitude,quality,total_distance,protocol");
        trackWriter.newLine();
        recording = true;
    }

    public void recordRaw(String line) throws IOException {
        if (!recording || rawWriter == null) {
            return;
        }
        rawWriter.write(line);
        rawWriter.newLine();
    }

    public void recordTrack(TrackPoint point, double totalDistance, String protocol) throws IOException {
        if (!recording || trackWriter == null) {
            return;
        }
        trackWriter.write(String.format(
                Locale.US,
                "%d,%s,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%s,%s,%s,%s,%s,%s,%.4f,%.6f,%s",
                point.timestampMillis,
                csv(point.source),
                point.east,
                point.north,
                point.up,
                point.velocityE,
                point.velocityN,
                point.velocityU,
                point.speed,
                optional(point.roll),
                optional(point.pitch),
                optional(point.yaw),
                optional(point.latitude),
                optional(point.longitude),
                optional(point.altitude),
                point.quality,
                totalDistance,
                csv(protocol)
        ));
        trackWriter.newLine();
    }

    public void stop() throws IOException {
        IOException firstError = null;
        if (rawWriter != null) {
            try {
                rawWriter.flush();
                rawWriter.close();
            } catch (IOException error) {
                firstError = error;
            }
            rawWriter = null;
        }
        if (trackWriter != null) {
            try {
                trackWriter.flush();
                trackWriter.close();
            } catch (IOException error) {
                if (firstError == null) {
                    firstError = error;
                }
            }
            trackWriter = null;
        }
        recording = false;
        if (firstError != null) {
            throw firstError;
        }
    }

    public boolean isRecording() {
        return recording;
    }

    public List<File> latestFiles() {
        List<File> files = new ArrayList<>();
        if (rawFile != null && rawFile.exists()) {
            files.add(rawFile);
        }
        if (trackFile != null && trackFile.exists()) {
            files.add(trackFile);
        }
        return files;
    }

    public String latestSummary() {
        if (rawFile == null && trackFile == null) {
            return "无记录文件";
        }
        return "原始: " + nameOrNone(rawFile) + " / 轨迹: " + nameOrNone(trackFile);
    }

    private static String nameOrNone(File file) {
        return file == null ? "--" : file.getName();
    }

    private static String optional(Double value) {
        if (value == null || value.isNaN() || value.isInfinite()) {
            return "";
        }
        return String.format(Locale.US, "%.8f", value);
    }

    private static String csv(String value) {
        if (value == null) {
            return "";
        }
        return "\"" + value.replace("\"", "\"\"") + "\"";
    }
}
