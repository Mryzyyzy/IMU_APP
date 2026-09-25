package com.example.imu_app;

import androidx.appcompat.app.AppCompatActivity;

import android.content.Intent;
import android.database.Cursor;
import android.graphics.Color;
import android.graphics.Typeface;
import android.graphics.drawable.GradientDrawable;
import android.net.Uri;
import android.os.Bundle;
import android.provider.OpenableColumns;
import android.os.Handler;
import android.os.Looper;
import android.view.Gravity;
import android.view.View;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.EditText;
import android.widget.FrameLayout;
import android.widget.LinearLayout;
import android.widget.ScrollView;
import android.widget.Spinner;
import android.widget.TextView;

import com.example.imu_app.communication.TcpClient;
import com.example.imu_app.model.AppStatus;
import com.example.imu_app.model.ImuSample;
import com.example.imu_app.model.PositionFix;
import com.example.imu_app.model.TrackPoint;
import com.example.imu_app.protocol.ImuCsvParser;
import com.example.imu_app.protocol.PositionCsvParser;
import com.example.imu_app.protocol.ProtocolParseException;
import com.example.imu_app.ui.TrajectoryView;
import com.example.imu_app.util.Formatters;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.Locale;

public class MainActivity extends AppCompatActivity {
    private static final int COLOR_PAGE = Color.rgb(238, 243, 248);
    private static final int COLOR_CARD = Color.WHITE;
    private static final int COLOR_BORDER = Color.rgb(217, 225, 236);
    private static final int COLOR_BLUE = Color.rgb(29, 111, 233);
    private static final int COLOR_GREEN = Color.rgb(10, 159, 77);
    private static final int COLOR_RED = Color.rgb(217, 37, 52);
    private static final int COLOR_TEXT = Color.rgb(23, 32, 51);
    private static final int COLOR_MUTED = Color.rgb(118, 131, 152);
    private static final int MAX_TRACK_POINTS = 5000;
    private static final int REQUEST_OPEN_DATA_FILE = 3101;
    private static final int FILE_REPLAY_DELAY_MS = 10;

    private final Handler handler = new Handler(Looper.getMainLooper());
    private final AppStatus status = new AppStatus();
    private final List<TrackPoint> trackPoints = new ArrayList<>();
    private final List<String> logs = new ArrayList<>();
    private final ImuCsvParser imuParser = new ImuCsvParser();
    private final PositionCsvParser positionParser = new PositionCsvParser();

    private TcpClient tcpClient;
    private FrameLayout pageContainer;
    private Button trajectoryTab;
    private Button deviceTab;
    private LinearLayout trajectoryPage;
    private LinearLayout devicePage;
    private TrajectoryView trajectoryView;

    private TextView headerConnection;
    private TextView headerSubtitle;
    private TextView speedValue;
    private TextView distanceValue;
    private TextView yawValue;
    private TextView qualityValue;
    private TextView eastValue;
    private TextView northValue;
    private TextView upValue;
    private TextView rollValue;
    private TextView pitchValue;
    private TextView yawDetailValue;
    private TextView latValue;
    private TextView lonValue;
    private TextView altitudeValue;
    private Button startButton;
    private Button pauseButton;
    private Button recordButton;

    private TextView deviceConnection;
    private TextView deviceEndpoint;
    private TextView deviceRuntime;
    private TextView logText;
    private Spinner transportSpinner;
    private EditText hostInput;
    private EditText portInput;
    private Spinner protocolSpinner;
    private Spinner modeSpinner;
    private Button connectButton;
    private LinearLayout hostRow;
    private LinearLayout portRow;
    private LinearLayout fileRow;
    private TextView filePathView;
    private Uri selectedFileUri;
    private Thread fileReaderThread;

    private boolean connected = false;
    private volatile boolean fileReading = false;
    private boolean receivingPaused = false;
    private boolean recording = false;
    private double totalDistance = 0.0;
    private TrackPoint previousPoint;

    private int framesSinceRateUpdate = 0;
    private long lastRateUpdateMillis = 0L;
    private int suppressedParseErrors = 0;

    private double imuEast = 0.0;
    private double imuNorth = 0.0;
    private double imuUp = 0.0;
    private double imuSpeed = 0.0;
    private double imuYawDeg = 0.0;
    private long lastImuTimestampMillis = 0L;

    private Double referenceLatitude;
    private Double referenceLongitude;
    private Double referenceAltitude;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        if (getSupportActionBar() != null) {
            getSupportActionBar().hide();
        }
        getWindow().setStatusBarColor(COLOR_BLUE);
        tcpClient = new TcpClient(new TcpEvents());
        status.connectionState = "未连接";
        status.message = "等待 TCP 连接";
        buildUi();
        appendLog("V2 TCP 接入与 CSV 解析已就绪");
        appendLog("请选择协议，输入 Host/Port 后点击连接");
        updateAllViews();
    }

    @Override
    protected void onDestroy() {
        if (tcpClient != null) {
            tcpClient.disconnect();
        }
        stopFileRead("关闭页面，停止文件读取", false);
        handler.removeCallbacksAndMessages(null);
        super.onDestroy();
    }

    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        super.onActivityResult(requestCode, resultCode, data);
        if (requestCode != REQUEST_OPEN_DATA_FILE || resultCode != RESULT_OK || data == null || data.getData() == null) {
            return;
        }
        selectedFileUri = data.getData();
        int flags = data.getFlags() & (Intent.FLAG_GRANT_READ_URI_PERMISSION | Intent.FLAG_GRANT_PERSISTABLE_URI_PERMISSION);
        try {
            getContentResolver().takePersistableUriPermission(selectedFileUri, flags & Intent.FLAG_GRANT_READ_URI_PERMISSION);
        } catch (SecurityException ignored) {
            // Some providers grant one-shot read access only; that is still enough for immediate replay.
        }
        filePathView.setText(displayNameForUri(selectedFileUri));
        appendLog("已选择文件: " + displayNameForUri(selectedFileUri));
        updateAllViews();
    }

    private void buildUi() {
        LinearLayout root = new LinearLayout(this);
        root.setOrientation(LinearLayout.VERTICAL);
        root.setBackgroundColor(COLOR_PAGE);
        setContentView(root);

        pageContainer = new FrameLayout(this);
        root.addView(pageContainer, new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT,
                0,
                1f
        ));

        trajectoryPage = buildTrajectoryPage();
        devicePage = buildDevicePage();
        pageContainer.addView(trajectoryPage);
        pageContainer.addView(devicePage);

        root.addView(buildBottomNav(), new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT,
                dp(62)
        ));

        showTrajectoryPage();
    }

    private LinearLayout buildTrajectoryPage() {
        LinearLayout page = new LinearLayout(this);
        page.setOrientation(LinearLayout.VERTICAL);
        page.setPadding(dp(14), dp(12), dp(14), dp(10));
        page.setBackgroundColor(COLOR_PAGE);

        page.addView(buildHeaderCard(), matchWrap());

        trajectoryView = new TrajectoryView(this);
        LinearLayout.LayoutParams trajectoryParams = new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT,
                0,
                1f
        );
        trajectoryParams.setMargins(0, dp(10), 0, dp(10));
        trajectoryView.setBackground(cardDrawable());
        page.addView(trajectoryView, trajectoryParams);

        LinearLayout metricGrid = card();
        page.addView(metricGrid, matchWrap());

        LinearLayout row1 = horizontalRow();
        speedValue = addMetric(row1, "速度", "--");
        distanceValue = addMetric(row1, "累计距离", "0.0 m");
        metricGrid.addView(row1, matchWrap());

        LinearLayout row2 = horizontalRow();
        yawValue = addMetric(row2, "航向", "--");
        qualityValue = addMetric(row2, "质量", "--");
        metricGrid.addView(row2, matchWrap());

        LinearLayout detailGrid = card();
        LinearLayout.LayoutParams detailParams = matchWrap();
        detailParams.setMargins(0, dp(10), 0, dp(10));
        page.addView(detailGrid, detailParams);

        LinearLayout row3 = horizontalRow();
        eastValue = addMetric(row3, "East", "--");
        northValue = addMetric(row3, "North", "--");
        upValue = addMetric(row3, "Up", "--");
        detailGrid.addView(row3, matchWrap());

        LinearLayout row4 = horizontalRow();
        rollValue = addMetric(row4, "Roll", "--");
        pitchValue = addMetric(row4, "Pitch", "--");
        yawDetailValue = addMetric(row4, "Yaw", "--");
        detailGrid.addView(row4, matchWrap());

        LinearLayout row5 = horizontalRow();
        latValue = addMetric(row5, "纬度", "--");
        lonValue = addMetric(row5, "经度", "--");
        altitudeValue = addMetric(row5, "高度", "--");
        detailGrid.addView(row5, matchWrap());

        page.addView(buildControlBar(), matchWrap());
        return page;
    }

    private View buildHeaderCard() {
        LinearLayout header = card();

        LinearLayout top = new LinearLayout(this);
        top.setOrientation(LinearLayout.HORIZONTAL);
        top.setGravity(Gravity.CENTER_VERTICAL);
        TextView title = text("IMU Monitor", 20, COLOR_TEXT, Typeface.BOLD);
        headerConnection = text("● 未连接", 14, COLOR_RED, Typeface.BOLD);
        top.addView(title, new LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f));
        top.addView(headerConnection);
        header.addView(top, matchWrap());

        headerSubtitle = text("IMU 解算模式 | 0 Hz | 包: 0", 13, COLOR_MUTED, Typeface.NORMAL);
        header.addView(headerSubtitle, topMargin(6));
        return header;
    }

    private View buildControlBar() {
        LinearLayout controls = new LinearLayout(this);
        controls.setOrientation(LinearLayout.HORIZONTAL);
        controls.setGravity(Gravity.CENTER);
        controls.setBackground(cardDrawable());
        controls.setPadding(dp(8), dp(8), dp(8), dp(8));

        startButton = controlButton("继续", COLOR_GREEN);
        pauseButton = controlButton("暂停", Color.rgb(100, 116, 139));
        recordButton = controlButton("记录", COLOR_RED);
        Button clearButton = controlButton("清空", COLOR_BLUE);

        startButton.setOnClickListener(v -> resumeReceiving());
        pauseButton.setOnClickListener(v -> pauseReceiving());
        recordButton.setOnClickListener(v -> toggleRecording());
        clearButton.setOnClickListener(v -> clearTrack());

        controls.addView(startButton, buttonWeightParams());
        controls.addView(pauseButton, buttonWeightParams());
        controls.addView(recordButton, buttonWeightParams());
        controls.addView(clearButton, buttonWeightParams());
        return controls;
    }

    private LinearLayout buildDevicePage() {
        ScrollView scrollView = new ScrollView(this);
        scrollView.setFillViewport(true);

        LinearLayout page = new LinearLayout(this);
        page.setOrientation(LinearLayout.VERTICAL);
        page.setPadding(dp(14), dp(12), dp(14), dp(12));
        page.setBackgroundColor(COLOR_PAGE);
        scrollView.addView(page);

        page.addView(buildDeviceSummary(), matchWrap());
        page.addView(buildConnectionSettings(), sectionParams());
        page.addView(buildModeSettings(), sectionParams());
        page.addView(buildLogPanel(), sectionParams());

        LinearLayout wrapper = new LinearLayout(this);
        wrapper.setOrientation(LinearLayout.VERTICAL);
        wrapper.addView(scrollView, new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT,
                LinearLayout.LayoutParams.MATCH_PARENT
        ));
        return wrapper;
    }

    private View buildDeviceSummary() {
        LinearLayout card = card();
        TextView title = text("设备状态", 16, COLOR_TEXT, Typeface.BOLD);
        deviceConnection = text("● 未连接", 15, COLOR_RED, Typeface.BOLD);
        deviceEndpoint = text("192.168.16.254:8000", 13, COLOR_MUTED, Typeface.NORMAL);
        deviceRuntime = text("等待 TCP 连接 | 0 Hz", 13, COLOR_MUTED, Typeface.NORMAL);
        card.addView(title, matchWrap());
        card.addView(deviceConnection, topMargin(8));
        card.addView(deviceEndpoint, topMargin(4));
        card.addView(deviceRuntime, topMargin(4));
        return card;
    }

    private View buildConnectionSettings() {
        LinearLayout card = card();
        card.addView(text("连接设置", 16, COLOR_TEXT, Typeface.BOLD), matchWrap());

        transportSpinner = spinner(new String[]{"TCP Client", "文件读取 File"});
        hostInput = editText("192.168.16.254");
        portInput = editText("8000");
        protocolSpinner = spinner(new String[]{"IMU CSV", "Position CSV"});
        connectButton = primaryButton("连接");
        connectButton.setOnClickListener(v -> toggleTransportConnection());

        card.addView(formRow("通信方式", transportSpinner), topMargin(10));
        hostRow = formRow("Host", hostInput);
        portRow = formRow("Port", portInput);
        fileRow = formRow("文件", buildFilePickerControl());
        card.addView(hostRow, topMargin(8));
        card.addView(portRow, topMargin(8));
        card.addView(fileRow, topMargin(8));
        card.addView(formRow("协议", protocolSpinner), topMargin(8));
        card.addView(connectButton, topMargin(12));
        transportSpinner.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(AdapterView<?> parent, View view, int position, long id) {
                updateTransportUi();
            }

            @Override
            public void onNothingSelected(AdapterView<?> parent) {
                updateTransportUi();
            }
        });
        updateTransportUi();
        return card;
    }

    private View buildFilePickerControl() {
        LinearLayout row = new LinearLayout(this);
        row.setOrientation(LinearLayout.HORIZONTAL);
        row.setGravity(Gravity.CENTER_VERTICAL);
        filePathView = text("未选择文件", 13, COLOR_MUTED, Typeface.NORMAL);
        filePathView.setSingleLine(true);
        filePathView.setPadding(dp(10), 0, dp(10), 0);
        filePathView.setBackground(plainDrawable(Color.rgb(248, 250, 252), COLOR_BORDER, dp(6)));
        Button chooseButton = smallButton("选择");
        chooseButton.setOnClickListener(v -> openFilePicker());
        row.addView(filePathView, new LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.MATCH_PARENT, 1f));
        LinearLayout.LayoutParams buttonParams = new LinearLayout.LayoutParams(dp(64), LinearLayout.LayoutParams.MATCH_PARENT);
        buttonParams.setMargins(dp(8), 0, 0, 0);
        row.addView(chooseButton, buttonParams);
        return row;
    }

    private View buildModeSettings() {
        LinearLayout card = card();
        card.addView(text("工作模式", 16, COLOR_TEXT, Typeface.BOLD), matchWrap());
        modeSpinner = spinner(new String[]{"IMU 解算模式", "位置直显模式"});
        Spinner algorithmSpinner = spinner(new String[]{"v2_tcp_preview", "v1_matlab_port", "v2_pdr_turn_snap"});
        card.addView(formRow("模式", modeSpinner), topMargin(10));
        card.addView(formRow("算法版本", algorithmSpinner), topMargin(8));
        return card;
    }

    private View buildLogPanel() {
        LinearLayout card = card();
        LinearLayout titleRow = new LinearLayout(this);
        titleRow.setOrientation(LinearLayout.HORIZONTAL);
        titleRow.setGravity(Gravity.CENTER_VERTICAL);
        titleRow.addView(text("运行日志", 16, COLOR_TEXT, Typeface.BOLD), new LinearLayout.LayoutParams(
                0,
                LinearLayout.LayoutParams.WRAP_CONTENT,
                1f
        ));
        Button clearLog = smallButton("清空");
        clearLog.setOnClickListener(v -> {
            logs.clear();
            updateLogText();
        });
        titleRow.addView(clearLog);
        card.addView(titleRow, matchWrap());

        logText = text("", 12, COLOR_TEXT, Typeface.NORMAL);
        logText.setTypeface(Typeface.MONOSPACE);
        ScrollView logScroll = new ScrollView(this);
        logScroll.setBackground(plainDrawable(Color.rgb(248, 250, 252), COLOR_BORDER, dp(6)));
        logScroll.setPadding(dp(10), dp(8), dp(10), dp(8));
        logScroll.addView(logText);
        LinearLayout.LayoutParams logParams = topMargin(10);
        logParams.height = dp(210);
        card.addView(logScroll, logParams);
        return card;
    }

    private View buildBottomNav() {
        LinearLayout nav = new LinearLayout(this);
        nav.setOrientation(LinearLayout.HORIZONTAL);
        nav.setPadding(dp(12), dp(8), dp(12), dp(8));
        nav.setBackgroundColor(Color.WHITE);

        trajectoryTab = navButton("轨迹");
        deviceTab = navButton("设备");
        trajectoryTab.setOnClickListener(v -> showTrajectoryPage());
        deviceTab.setOnClickListener(v -> showDevicePage());
        nav.addView(trajectoryTab, buttonWeightParams());
        nav.addView(deviceTab, buttonWeightParams());
        return nav;
    }

    private void toggleTransportConnection() {
        if ("文件读取 File".equals(currentTransport())) {
            if (fileReading) {
                stopFileRead("手动停止文件读取", true);
            } else {
                startFileRead();
            }
            return;
        }
        toggleTcpConnection();
    }

    private void openFilePicker() {
        Intent intent = new Intent(Intent.ACTION_OPEN_DOCUMENT);
        intent.addCategory(Intent.CATEGORY_OPENABLE);
        intent.setType("*/*");
        intent.addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION | Intent.FLAG_GRANT_PERSISTABLE_URI_PERMISSION);
        String[] mimeTypes = new String[]{"text/*", "application/octet-stream", "application/vnd.ms-excel", "text/comma-separated-values"};
        intent.putExtra(Intent.EXTRA_MIME_TYPES, mimeTypes);
        startActivityForResult(intent, REQUEST_OPEN_DATA_FILE);
    }

    private void startFileRead() {
        if (selectedFileUri == null) {
            appendLog("WARN 请先选择微信/QQ保存出来的数据文件");
            openFilePicker();
            return;
        }
        if (tcpClient != null && tcpClient.isRunning()) {
            tcpClient.disconnect();
        }
        stopFileRead("切换文件读取", false);
        resetRuntimeState(false);
        connected = true;
        fileReading = true;
        receivingPaused = false;
        status.connectionState = "读取中";
        status.running = true;
        status.message = "正在读取文件";
        appendLog("开始读取文件: " + displayNameForUri(selectedFileUri) + " / " + currentProtocol());
        updateAllViews();

        fileReaderThread = new Thread(() -> readSelectedFile(selectedFileUri), "imu-file-reader");
        fileReaderThread.start();
    }

    private void stopFileRead(String reason, boolean updateUi) {
        fileReading = false;
        if (fileReaderThread != null) {
            fileReaderThread.interrupt();
            fileReaderThread = null;
        }
        if (updateUi) {
            connected = false;
            status.running = false;
            status.connectionState = "未连接";
            status.message = reason;
            status.dataRateHz = 0.0;
            appendLog(reason);
            updateAllViews();
        }
    }

    private void readSelectedFile(Uri uri) {
        int lineCount = 0;
        try (InputStream stream = getContentResolver().openInputStream(uri);
             BufferedReader reader = stream == null ? null : new BufferedReader(new InputStreamReader(stream))) {
            if (reader == null) {
                throw new IOException("无法打开文件输入流");
            }
            String line;
            while (fileReading && !Thread.currentThread().isInterrupted() && (line = reader.readLine()) != null) {
                String frame = line.trim();
                if (frame.isEmpty()) {
                    continue;
                }
                lineCount++;
                String finalFrame = frame;
                handler.post(() -> handleIncomingLine(finalFrame));
                Thread.sleep(FILE_REPLAY_DELAY_MS);
            }
            if (!fileReading) {
                return;
            }
            int finalLineCount = lineCount;
            handler.post(() -> {
                fileReading = false;
                connected = false;
                status.running = false;
                status.connectionState = "已完成";
                status.message = "文件读取完成";
                status.dataRateHz = 0.0;
                appendLog("文件读取完成，共读取 " + finalLineCount + " 行");
                updateAllViews();
            });
        } catch (Exception error) {
            if (!fileReading) {
                return;
            }
            handler.post(() -> {
                fileReading = false;
                connected = false;
                status.running = false;
                status.connectionState = "错误";
                status.message = error.getMessage() == null ? "文件读取失败" : error.getMessage();
                status.dataRateHz = 0.0;
                appendLog("ERROR 文件读取失败: " + status.message);
                updateAllViews();
            });
        }
    }

    private void toggleTcpConnection() {
        if (connected || (tcpClient != null && tcpClient.isRunning())) {
            disconnectTcp("手动断开");
            return;
        }
        if (fileReading) {
            stopFileRead("切换到 TCP", false);
        }
        String host = hostInput.getText().toString().trim();
        int port;
        try {
            port = Integer.parseInt(portInput.getText().toString().trim());
        } catch (NumberFormatException error) {
            appendLog("ERROR 端口不是有效数字");
            return;
        }
        if (host.isEmpty()) {
            appendLog("ERROR Host 不能为空");
            return;
        }
        resetRuntimeState(false);
        receivingPaused = false;
        status.connectionState = "连接中";
        status.message = "正在连接 " + host + ":" + port;
        appendLog("连接 TCP: " + host + ":" + port + " / " + currentProtocol());
        updateAllViews();
        tcpClient.connect(host, port);
    }

    private void disconnectTcp(String reason) {
        if (tcpClient != null) {
            tcpClient.disconnect();
        }
        connected = false;
        receivingPaused = false;
        status.running = false;
        status.connectionState = "未连接";
        status.message = reason;
        status.dataRateHz = 0.0;
        appendLog(reason);
        updateAllViews();
    }

    private void resumeReceiving() {
        if (!connected) {
            appendLog("WARN 尚未连接 TCP");
            return;
        }
        receivingPaused = false;
        status.running = true;
        status.message = "接收中";
        appendLog("继续接收数据");
        updateAllViews();
    }

    private void pauseReceiving() {
        if (!connected) {
            return;
        }
        receivingPaused = true;
        status.running = false;
        status.message = "已暂停显示";
        appendLog("已暂停显示，TCP 连接保持");
        updateAllViews();
    }

    private void toggleRecording() {
        recording = !recording;
        appendLog(recording ? "开始记录标记" : "停止记录标记");
        updateAllViews();
    }

    private void clearTrack() {
        resetRuntimeState(true);
        trajectoryView.clear();
        appendLog("轨迹已清空");
        updateAllViews();
    }

    private void resetRuntimeState(boolean keepConnectionState) {
        trackPoints.clear();
        totalDistance = 0.0;
        previousPoint = null;
        status.packetCount = 0;
        status.dataRateHz = 0.0;
        framesSinceRateUpdate = 0;
        lastRateUpdateMillis = 0L;
        suppressedParseErrors = 0;
        imuEast = 0.0;
        imuNorth = 0.0;
        imuUp = 0.0;
        imuSpeed = 0.0;
        imuYawDeg = 0.0;
        lastImuTimestampMillis = 0L;
        referenceLatitude = null;
        referenceLongitude = null;
        referenceAltitude = null;
        if (!keepConnectionState) {
            connected = false;
            status.connectionState = "未连接";
            status.message = "等待 TCP 连接";
        }
    }

    private void handleIncomingLine(String line) {
        if (receivingPaused) {
            return;
        }
        long now = System.currentTimeMillis();
        try {
            TrackPoint point;
            if ("Position CSV".equals(currentProtocol())) {
                PositionFix fix = positionParser.parse(line, now);
                point = trackPointFromPosition(fix);
            } else {
                ImuSample sample = imuParser.parse(line, now);
                point = trackPointFromImu(sample);
            }
            appendTrackPoint(point);
            suppressedParseErrors = 0;
        } catch (ProtocolParseException error) {
            suppressedParseErrors++;
            if (suppressedParseErrors <= 5 || suppressedParseErrors % 25 == 0) {
                appendLog("ERROR 解析失败: " + error.getMessage());
            }
        }
    }

    private TrackPoint trackPointFromPosition(PositionFix fix) {
        double east;
        double north;
        double up;
        if (fix.latitude != null && fix.longitude != null && fix.altitude != null) {
            if (referenceLatitude == null) {
                referenceLatitude = fix.latitude;
                referenceLongitude = fix.longitude;
                referenceAltitude = fix.altitude;
                appendLog("设置经纬度参考点: " + Formatters.optional(referenceLatitude, 6) + ", " + Formatters.optional(referenceLongitude, 6));
            }
            double latScale = 110540.0;
            double lonScale = 111320.0 * Math.cos(Math.toRadians(referenceLatitude));
            east = (fix.longitude - referenceLongitude) * lonScale;
            north = (fix.latitude - referenceLatitude) * latScale;
            up = fix.altitude - referenceAltitude;
        } else {
            east = valueOrZero(fix.x);
            north = valueOrZero(fix.y);
            up = valueOrZero(fix.z);
        }

        double velocityE = 0.0;
        double velocityN = 0.0;
        double velocityU = 0.0;
        double speed = 0.0;
        Double yaw = fix.yaw;
        if (previousPoint != null) {
            double dt = Math.max((fix.timestampMillis - previousPoint.timestampMillis) / 1000.0, 1e-3);
            velocityE = (east - previousPoint.east) / dt;
            velocityN = (north - previousPoint.north) / dt;
            velocityU = (up - previousPoint.up) / dt;
            speed = Math.sqrt(velocityE * velocityE + velocityN * velocityN + velocityU * velocityU);
            if (yaw == null && speed > 1e-3) {
                yaw = normalizeDegrees(Math.toDegrees(Math.atan2(velocityN, velocityE)));
            }
        }

        return new TrackPoint(
                fix.timestampMillis,
                fix.latitude,
                fix.longitude,
                fix.altitude,
                east,
                north,
                up,
                velocityE,
                velocityN,
                velocityU,
                speed,
                null,
                null,
                yaw,
                "Position CSV",
                fix.quality
        );
    }

    private TrackPoint trackPointFromImu(ImuSample sample) {
        double dt = 0.02;
        if (lastImuTimestampMillis > 0L) {
            dt = Math.max((sample.timestampMillis - lastImuTimestampMillis) / 1000.0, 1e-3);
            dt = Math.min(dt, 0.25);
        }
        lastImuTimestampMillis = sample.timestampMillis;

        imuYawDeg = normalizeDegrees(imuYawDeg + Math.toDegrees(sample.gz) * dt);
        double forwardAccel = clamp(sample.ax, -3.0, 3.0);
        imuSpeed = clamp((imuSpeed + forwardAccel * dt) * 0.992, 0.0, 4.0);
        double yawRad = Math.toRadians(imuYawDeg);
        double velocityE = Math.cos(yawRad) * imuSpeed;
        double velocityN = Math.sin(yawRad) * imuSpeed;
        imuEast += velocityE * dt;
        imuNorth += velocityN * dt;
        imuUp += clamp(sample.az - 9.80665, -1.0, 1.0) * dt * 0.02;

        double roll = Math.toDegrees(Math.atan2(sample.ay, sample.az));
        double pitch = Math.toDegrees(Math.atan2(-sample.ax, Math.sqrt(sample.ay * sample.ay + sample.az * sample.az)));

        return new TrackPoint(
                sample.timestampMillis,
                null,
                null,
                null,
                imuEast,
                imuNorth,
                imuUp,
                velocityE,
                velocityN,
                0.0,
                imuSpeed,
                roll,
                pitch,
                imuYawDeg,
                "IMU CSV / v2 preview",
                1.0
        );
    }

    private void appendTrackPoint(TrackPoint point) {
        if (previousPoint != null) {
            double step = Math.sqrt(
                    Math.pow(point.east - previousPoint.east, 2)
                            + Math.pow(point.north - previousPoint.north, 2)
                            + Math.pow(point.up - previousPoint.up, 2)
            );
            if (Double.isFinite(step) && step < 500.0) {
                totalDistance += step;
            }
        }
        previousPoint = point;
        trackPoints.add(point);
        if (trackPoints.size() > MAX_TRACK_POINTS) {
            trackPoints.remove(0);
        }
        status.packetCount++;
        updateRate();
        status.connectionState = "已连接";
        status.running = true;
        status.message = recording ? "接收中 / 记录标记" : "接收中";
        trajectoryView.setTrack(trackPoints);
        updateAllViews();
    }

    private void updateRate() {
        long now = System.currentTimeMillis();
        if (lastRateUpdateMillis == 0L) {
            lastRateUpdateMillis = now;
            framesSinceRateUpdate = 0;
            return;
        }
        framesSinceRateUpdate++;
        long elapsed = now - lastRateUpdateMillis;
        if (elapsed >= 1000L) {
            status.dataRateHz = framesSinceRateUpdate * 1000.0 / elapsed;
            framesSinceRateUpdate = 0;
            lastRateUpdateMillis = now;
        }
    }

    private void updateAllViews() {
        TrackPoint point = trackPoints.isEmpty() ? null : trackPoints.get(trackPoints.size() - 1);
        int connectionColor = connected ? COLOR_GREEN : COLOR_RED;
        if ("连接中".equals(status.connectionState) || "已完成".equals(status.connectionState)) {
            connectionColor = COLOR_BLUE;
        } else if ("读取中".equals(status.connectionState)) {
            connectionColor = COLOR_GREEN;
        }
        headerConnection.setText("● " + status.connectionState);
        headerConnection.setTextColor(connectionColor);
        headerSubtitle.setText(currentMode() + " | " + Formatters.oneDecimal(status.dataRateHz) + " Hz | 包: " + status.packetCount);

        if (point == null) {
            speedValue.setText("--");
            distanceValue.setText("0.0 m");
            yawValue.setText("--");
            qualityValue.setText("--");
            eastValue.setText("--");
            northValue.setText("--");
            upValue.setText("--");
            rollValue.setText("--");
            pitchValue.setText("--");
            yawDetailValue.setText("--");
            latValue.setText("--");
            lonValue.setText("--");
            altitudeValue.setText("--");
        } else {
            speedValue.setText(Formatters.speed(point.speed));
            distanceValue.setText(Formatters.oneDecimal(totalDistance) + " m");
            yawValue.setText(Formatters.degrees(point.yaw));
            qualityValue.setText(Formatters.percent(point.quality));
            eastValue.setText(Formatters.meters(point.east));
            northValue.setText(Formatters.meters(point.north));
            upValue.setText(Formatters.meters(point.up));
            rollValue.setText(Formatters.degrees(point.roll));
            pitchValue.setText(Formatters.degrees(point.pitch));
            yawDetailValue.setText(Formatters.degrees(point.yaw));
            latValue.setText(Formatters.optional(point.latitude, 6));
            lonValue.setText(Formatters.optional(point.longitude, 6));
            altitudeValue.setText(point.altitude == null ? "--" : Formatters.meters(point.altitude));
        }

        startButton.setEnabled(connected && receivingPaused && !fileReading);
        pauseButton.setEnabled(connected && !receivingPaused && !fileReading);
        recordButton.setText(recording ? "停止记录" : "记录");
        recordButton.setBackground(plainDrawable(recording ? COLOR_RED : Color.WHITE, COLOR_RED, dp(6)));
        recordButton.setTextColor(recording ? Color.WHITE : COLOR_RED);

        deviceConnection.setText("● " + status.connectionState);
        deviceConnection.setTextColor(connectionColor);
        if ("文件读取 File".equals(currentTransport())) {
            deviceEndpoint.setText(displayNameForUri(selectedFileUri) + " / " + currentProtocol());
        } else {
            deviceEndpoint.setText(hostInput.getText().toString() + ":" + portInput.getText().toString() + " / " + currentProtocol());
        }
        deviceRuntime.setText(status.message + " | " + Formatters.oneDecimal(status.dataRateHz) + " Hz");
        if ("文件读取 File".equals(currentTransport())) {
            connectButton.setText(fileReading ? "停止读取" : "读取文件");
        } else {
            connectButton.setText(connected || (tcpClient != null && tcpClient.isRunning()) ? "断开" : "连接");
        }
    }

    private String currentTransport() {
        return transportSpinner == null ? "TCP Client" : String.valueOf(transportSpinner.getSelectedItem());
    }

    private String currentProtocol() {
        return protocolSpinner == null ? "IMU CSV" : String.valueOf(protocolSpinner.getSelectedItem());
    }

    private String currentMode() {
        return modeSpinner == null ? status.mode : String.valueOf(modeSpinner.getSelectedItem());
    }

    private void updateTransportUi() {
        boolean fileMode = "文件读取 File".equals(currentTransport());
        if (hostRow != null) {
            hostRow.setVisibility(fileMode ? View.GONE : View.VISIBLE);
        }
        if (portRow != null) {
            portRow.setVisibility(fileMode ? View.GONE : View.VISIBLE);
        }
        if (fileRow != null) {
            fileRow.setVisibility(fileMode ? View.VISIBLE : View.GONE);
        }
        if (connected || fileReading || (tcpClient != null && tcpClient.isRunning())) {
            updateAllViews();
            return;
        }
        status.connectionState = "未连接";
        status.message = fileMode ? "等待选择文件" : "等待 TCP 连接";
        updateAllViews();
    }

    private String displayNameForUri(Uri uri) {
        if (uri == null) {
            return "未选择文件";
        }
        try (Cursor cursor = getContentResolver().query(uri, null, null, null, null)) {
            if (cursor != null && cursor.moveToFirst()) {
                int index = cursor.getColumnIndex(OpenableColumns.DISPLAY_NAME);
                if (index >= 0) {
                    String name = cursor.getString(index);
                    if (name != null && !name.trim().isEmpty()) {
                        return name;
                    }
                }
            }
        } catch (Exception ignored) {
            // Fall back to URI text below.
        }
        String lastPath = uri.getLastPathSegment();
        return lastPath == null ? uri.toString() : lastPath;
    }

    private void appendLog(String message) {
        String time = new SimpleDateFormat("HH:mm:ss", Locale.CHINA).format(new Date());
        logs.add(time + "  " + message);
        while (logs.size() > 100) {
            logs.remove(0);
        }
        updateLogText();
    }

    private void updateLogText() {
        if (logText == null) {
            return;
        }
        StringBuilder builder = new StringBuilder();
        for (String log : logs) {
            builder.append(log).append('\n');
        }
        logText.setText(builder.toString());
    }

    private void showTrajectoryPage() {
        trajectoryPage.setVisibility(View.VISIBLE);
        devicePage.setVisibility(View.GONE);
        trajectoryTab.setTextColor(Color.WHITE);
        trajectoryTab.setBackground(plainDrawable(COLOR_BLUE, COLOR_BLUE, dp(8)));
        deviceTab.setTextColor(COLOR_MUTED);
        deviceTab.setBackground(plainDrawable(Color.WHITE, COLOR_BORDER, dp(8)));
    }

    private void showDevicePage() {
        trajectoryPage.setVisibility(View.GONE);
        devicePage.setVisibility(View.VISIBLE);
        trajectoryTab.setTextColor(COLOR_MUTED);
        trajectoryTab.setBackground(plainDrawable(Color.WHITE, COLOR_BORDER, dp(8)));
        deviceTab.setTextColor(Color.WHITE);
        deviceTab.setBackground(plainDrawable(COLOR_BLUE, COLOR_BLUE, dp(8)));
    }

    private TextView addMetric(LinearLayout row, String label, String value) {
        LinearLayout item = new LinearLayout(this);
        item.setOrientation(LinearLayout.VERTICAL);
        item.setPadding(dp(4), dp(2), dp(4), dp(2));

        TextView labelView = text(label, 12, COLOR_MUTED, Typeface.NORMAL);
        TextView valueView = text(value, 15, COLOR_TEXT, Typeface.BOLD);
        item.addView(labelView, matchWrap());
        item.addView(valueView, topMargin(3));
        row.addView(item, new LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f));
        return valueView;
    }

    private LinearLayout formRow(String label, View input) {
        LinearLayout row = new LinearLayout(this);
        row.setOrientation(LinearLayout.HORIZONTAL);
        row.setGravity(Gravity.CENTER_VERTICAL);
        TextView labelView = text(label, 13, COLOR_MUTED, Typeface.NORMAL);
        row.addView(labelView, new LinearLayout.LayoutParams(dp(76), LinearLayout.LayoutParams.WRAP_CONTENT));
        row.addView(input, new LinearLayout.LayoutParams(0, dp(42), 1f));
        return row;
    }

    private LinearLayout card() {
        LinearLayout card = new LinearLayout(this);
        card.setOrientation(LinearLayout.VERTICAL);
        card.setPadding(dp(14), dp(12), dp(14), dp(12));
        card.setBackground(cardDrawable());
        return card;
    }

    private LinearLayout horizontalRow() {
        LinearLayout row = new LinearLayout(this);
        row.setOrientation(LinearLayout.HORIZONTAL);
        row.setGravity(Gravity.CENTER_VERTICAL);
        return row;
    }

    private TextView text(String value, int sp, int color, int style) {
        TextView textView = new TextView(this);
        textView.setText(value);
        textView.setTextSize(sp);
        textView.setTextColor(color);
        textView.setTypeface(Typeface.DEFAULT, style);
        textView.setIncludeFontPadding(true);
        return textView;
    }

    private EditText editText(String value) {
        EditText editText = new EditText(this);
        editText.setSingleLine(true);
        editText.setText(value);
        editText.setTextSize(14);
        editText.setTextColor(COLOR_TEXT);
        editText.setPadding(dp(10), 0, dp(10), 0);
        editText.setBackground(plainDrawable(Color.rgb(248, 250, 252), COLOR_BORDER, dp(6)));
        return editText;
    }

    private Spinner spinner(String[] values) {
        Spinner spinner = new Spinner(this);
        ArrayAdapter<String> adapter = new ArrayAdapter<>(
                this,
                android.R.layout.simple_spinner_dropdown_item,
                values
        );
        spinner.setAdapter(adapter);
        spinner.setBackground(plainDrawable(Color.rgb(248, 250, 252), COLOR_BORDER, dp(6)));
        return spinner;
    }

    private Button controlButton(String text, int color) {
        Button button = new Button(this);
        button.setAllCaps(false);
        button.setText(text);
        button.setTextSize(13);
        button.setTypeface(Typeface.DEFAULT, Typeface.BOLD);
        button.setTextColor(color);
        button.setBackground(plainDrawable(Color.WHITE, color, dp(6)));
        return button;
    }

    private Button primaryButton(String text) {
        Button button = new Button(this);
        button.setAllCaps(false);
        button.setText(text);
        button.setTextSize(14);
        button.setTypeface(Typeface.DEFAULT, Typeface.BOLD);
        button.setTextColor(Color.WHITE);
        button.setBackground(plainDrawable(COLOR_BLUE, COLOR_BLUE, dp(6)));
        return button;
    }

    private Button smallButton(String text) {
        Button button = new Button(this);
        button.setAllCaps(false);
        button.setText(text);
        button.setTextSize(12);
        button.setTextColor(COLOR_BLUE);
        button.setBackground(plainDrawable(Color.WHITE, COLOR_BORDER, dp(6)));
        return button;
    }

    private Button navButton(String text) {
        Button button = new Button(this);
        button.setAllCaps(false);
        button.setText(text);
        button.setTextSize(15);
        button.setTypeface(Typeface.DEFAULT, Typeface.BOLD);
        return button;
    }

    private GradientDrawable cardDrawable() {
        return plainDrawable(COLOR_CARD, COLOR_BORDER, dp(8));
    }

    private GradientDrawable plainDrawable(int fill, int stroke, int radius) {
        GradientDrawable drawable = new GradientDrawable();
        drawable.setColor(fill);
        drawable.setStroke(dp(1), stroke);
        drawable.setCornerRadius(radius);
        return drawable;
    }

    private LinearLayout.LayoutParams matchWrap() {
        return new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT,
                LinearLayout.LayoutParams.WRAP_CONTENT
        );
    }

    private LinearLayout.LayoutParams topMargin(int marginDp) {
        LinearLayout.LayoutParams params = matchWrap();
        params.setMargins(0, dp(marginDp), 0, 0);
        return params;
    }

    private LinearLayout.LayoutParams sectionParams() {
        LinearLayout.LayoutParams params = matchWrap();
        params.setMargins(0, dp(10), 0, 0);
        return params;
    }

    private LinearLayout.LayoutParams buttonWeightParams() {
        LinearLayout.LayoutParams params = new LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.MATCH_PARENT, 1f);
        params.setMargins(dp(4), 0, dp(4), 0);
        return params;
    }

    private int dp(float value) {
        return (int) (value * getResources().getDisplayMetrics().density + 0.5f);
    }

    private double valueOrZero(Double value) {
        return value == null ? 0.0 : value;
    }

    private double normalizeDegrees(double value) {
        double normalized = value % 360.0;
        return normalized < 0.0 ? normalized + 360.0 : normalized;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private class TcpEvents implements TcpClient.Listener {
        @Override
        public void onConnected() {
            handler.post(() -> {
                connected = true;
                receivingPaused = false;
                status.connectionState = "已连接";
                status.running = true;
                status.message = "接收中";
                appendLog("TCP 已连接");
                updateAllViews();
            });
        }

        @Override
        public void onLine(String line) {
            handler.post(() -> handleIncomingLine(line));
        }

        @Override
        public void onDisconnected(String reason) {
            handler.post(() -> {
                connected = false;
                status.running = false;
                status.connectionState = "未连接";
                status.message = reason;
                status.dataRateHz = 0.0;
                appendLog("WARN " + reason);
                updateAllViews();
            });
        }

        @Override
        public void onError(String message) {
            handler.post(() -> {
                connected = false;
                status.running = false;
                status.connectionState = "错误";
                status.message = message == null ? "TCP 错误" : message;
                status.dataRateHz = 0.0;
                appendLog("ERROR TCP: " + status.message);
                updateAllViews();
            });
        }
    }
}
