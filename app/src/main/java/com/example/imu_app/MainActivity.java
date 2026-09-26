package com.example.imu_app;

import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.core.content.FileProvider;
import androidx.core.content.ContextCompat;

import android.Manifest;
import android.content.ClipData;
import android.content.Context;
import android.content.Intent;
import android.content.SharedPreferences;
import android.content.pm.PackageManager;
import android.content.pm.ResolveInfo;
import android.database.Cursor;
import android.graphics.Color;
import android.graphics.Typeface;
import android.graphics.drawable.GradientDrawable;
import android.graphics.drawable.StateListDrawable;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.net.Uri;
import android.os.Bundle;
import android.provider.OpenableColumns;
import android.os.Handler;
import android.os.Looper;
import android.view.Gravity;
import android.view.MotionEvent;
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

import com.amap.api.maps.AMap;
import com.amap.api.maps.CameraUpdateFactory;
import com.amap.api.maps.MapView;
import com.amap.api.maps.MapsInitializer;
import com.amap.api.maps.model.BitmapDescriptorFactory;
import com.amap.api.maps.model.LatLng;
import com.amap.api.maps.model.LatLngBounds;
import com.amap.api.maps.model.Marker;
import com.amap.api.maps.model.MarkerOptions;
import com.amap.api.maps.model.Polyline;
import com.amap.api.maps.model.PolylineOptions;
import com.example.imu_app.communication.TcpClient;
import com.example.imu_app.coordinate.CoordinateConverter;
import com.example.imu_app.model.AppStatus;
import com.example.imu_app.model.ImuSample;
import com.example.imu_app.model.PositionFix;
import com.example.imu_app.model.TrackPoint;
import com.example.imu_app.navigation.ImuNavigationProcessor;
import com.example.imu_app.protocol.ImuCsvParser;
import com.example.imu_app.protocol.PositionCsvParser;
import com.example.imu_app.protocol.ProtocolParseException;
import com.example.imu_app.recording.TrackRecorder;
import com.example.imu_app.ui.TrajectoryView;
import com.example.imu_app.util.Formatters;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.Locale;
import java.util.zip.ZipEntry;
import java.util.zip.ZipOutputStream;

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
    private static final int REQUEST_LOCATION_PERMISSION = 3102;
    private static final int FILE_REPLAY_DELAY_MS = 35;
    private static final long UI_UPDATE_INTERVAL_MS = 50L;
    private static final long TRAJECTORY_UPDATE_INTERVAL_MS = 50L;
    private static final long MAP_UPDATE_INTERVAL_MS = 200L;
    private static final long MAP_CAMERA_INTERVAL_MS = 1000L;
    private static final long RAW_LINE_UPDATE_INTERVAL_MS = 100L;
    private static final String PREFS_NAME = "imu_mobile_settings";
    private static final String AMAP_KEY = "0e8026cdfbf451fb8988af4207a0a509";
    private static final double DEFAULT_REFERENCE_LATITUDE = 30.659462;
    private static final double DEFAULT_REFERENCE_LONGITUDE = 104.065735;
    private static final double DEFAULT_REFERENCE_ALTITUDE = 482.0;

    private final Handler handler = new Handler(Looper.getMainLooper());
    private final AppStatus status = new AppStatus();
    private final List<TrackPoint> trackPoints = new ArrayList<>();
    private final List<String> logs = new ArrayList<>();
    private final ImuCsvParser imuParser = new ImuCsvParser();
    private final PositionCsvParser positionParser = new PositionCsvParser();
    private final TrackRecorder trackRecorder = new TrackRecorder();
    private final ImuNavigationProcessor imuNavigationProcessor = new ImuNavigationProcessor();

    private TcpClient tcpClient;
    private FrameLayout pageContainer;
    private Button trajectoryTab;
    private Button mapTab;
    private Button deviceTab;
    private LinearLayout trajectoryPage;
    private LinearLayout mapPage;
    private LinearLayout devicePage;
    private TrajectoryView trajectoryView;
    private LinearLayout.LayoutParams trajectoryViewParams;
    private float trajectoryResizeStartY = 0f;
    private int trajectoryResizeStartHeight = 0;
    private Bundle mapSavedInstanceState;
    private MapView amapMapView;
    private AMap aMap;
    private Polyline amapPolyline;
    private Marker amapMarker;
    private TextView mapStatusText;
    private TextView mapInfoText;
    private TextView mapEmptyText;
    private boolean amapLoaded = false;
    private boolean mapHasTrack = false;
    private boolean mapUserZoomLocked = false;

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
    private TextView recentRawText;
    private TextView recentTrackText;
    private TextView latestRecordText;
    private Button exportRecordButton;
    private Spinner transportSpinner;
    private EditText hostInput;
    private EditText portInput;
    private EditText referenceLatInput;
    private EditText referenceLonInput;
    private EditText referenceAltInput;
    private Spinner protocolSpinner;
    private Spinner modeSpinner;
    private Spinner algorithmSpinner;
    private Button connectButton;
    private LinearLayout endpointRow;
    private LinearLayout hostRow;
    private LinearLayout portRow;
    private LinearLayout fileRow;
    private TextView filePathView;
    private Uri selectedFileUri;
    private Thread fileReaderThread;
    private final List<TrackPoint> fileReplayPoints = new ArrayList<>();
    private final List<String> fileReplayRawLines = new ArrayList<>();
    private Runnable fileReplayRunnable;
    private int fileReplayIndex = 0;
    private boolean fileReplayRawAlreadyRecorded = false;

    private boolean connected = false;
    private volatile boolean fileReading = false;
    private volatile boolean receivingPaused = false;
    private boolean recording = false;
    private double totalDistance = 0.0;
    private TrackPoint previousPoint;

    private int framesSinceRateUpdate = 0;
    private long lastRateUpdateMillis = 0L;
    private int suppressedParseErrors = 0;
    private long lastUiUpdateMillis = 0L;
    private long lastTrajectoryUpdateMillis = 0L;
    private long lastMapUpdateMillis = 0L;
    private long lastMapCameraMoveMillis = 0L;
    private long lastRawLineUpdateMillis = 0L;

    private double referenceLatitude = DEFAULT_REFERENCE_LATITUDE;
    private double referenceLongitude = DEFAULT_REFERENCE_LONGITUDE;
    private double referenceAltitude = DEFAULT_REFERENCE_ALTITUDE;
    private boolean referenceInitializedFromInput = false;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        if (getSupportActionBar() != null) {
            getSupportActionBar().hide();
        }
        getWindow().setStatusBarColor(COLOR_BLUE);
        MapsInitializer.updatePrivacyShow(this, true, true);
        MapsInitializer.updatePrivacyAgree(this, true);
        MapsInitializer.setApiKey(AMAP_KEY);
        tcpClient = new TcpClient(new TcpEvents());
        mapSavedInstanceState = savedInstanceState;
        status.connectionState = "未连接";
        status.message = "等待 TCP 连接";
        buildUi();
        loadSettings();
        resetImuNavigationProcessor();
        appendLog("INFO V3 监控闭环已就绪");
        appendLog("INFO 可 TCP 接收、文件读取、微信/QQ导入和记录导出");
        handleImportIntent(getIntent());
        updateAllViews();
    }

    @Override
    protected void onNewIntent(Intent intent) {
        super.onNewIntent(intent);
        setIntent(intent);
        handleImportIntent(intent);
    }

    @Override
    protected void onDestroy() {
        if (tcpClient != null) {
            tcpClient.disconnect();
        }
        stopFileRead("关闭页面，停止文件读取", false);
        stopRecordingIfNeeded(false);
        handler.removeCallbacksAndMessages(null);
        if (amapMapView != null) {
            amapMapView.onDestroy();
        }
        super.onDestroy();
    }

    @Override
    protected void onResume() {
        super.onResume();
        if (amapMapView != null) {
            amapMapView.onResume();
        }
    }

    @Override
    protected void onPause() {
        saveSettings();
        if (amapMapView != null) {
            amapMapView.onPause();
        }
        super.onPause();
    }

    @Override
    protected void onSaveInstanceState(Bundle outState) {
        super.onSaveInstanceState(outState);
        if (amapMapView != null) {
            amapMapView.onSaveInstanceState(outState);
        }
    }

    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        super.onActivityResult(requestCode, resultCode, data);
        if (requestCode != REQUEST_OPEN_DATA_FILE || resultCode != RESULT_OK || data == null || data.getData() == null) {
            return;
        }
        setSelectedFileUri(data.getData(), data.getFlags());
        updateAllViews();
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, String[] permissions, int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        if (requestCode != REQUEST_LOCATION_PERMISSION) {
            return;
        }
        if (grantResults.length > 0 && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
            requestReferenceLocation();
        } else {
            appendLog("WARN 未授予定位权限，无法使用 GPS 设置参考原点");
        }
    }

    private void handleImportIntent(Intent intent) {
        if (intent == null) {
            return;
        }
        Uri uri = null;
        if (Intent.ACTION_SEND.equals(intent.getAction())) {
            Object stream = intent.getParcelableExtra(Intent.EXTRA_STREAM);
            if (stream instanceof Uri) {
                uri = (Uri) stream;
            }
        } else if (Intent.ACTION_VIEW.equals(intent.getAction())) {
            uri = intent.getData();
        }
        if (uri == null) {
            return;
        }
        setSelectedFileUri(uri, intent.getFlags());
        if (transportSpinner != null) {
            transportSpinner.setSelection(1);
        }
        appendLog("INFO 已从外部应用导入文件，可点击读取文件");
        updateAllViews();
    }

    private void setSelectedFileUri(Uri uri, int intentFlags) {
        selectedFileUri = uri;
        int flags = intentFlags & (Intent.FLAG_GRANT_READ_URI_PERMISSION | Intent.FLAG_GRANT_PERSISTABLE_URI_PERMISSION);
        try {
            getContentResolver().takePersistableUriPermission(selectedFileUri, flags & Intent.FLAG_GRANT_READ_URI_PERMISSION);
        } catch (SecurityException ignored) {
            // Some providers grant one-shot read access only; that is still enough for immediate replay.
        }
        if (filePathView != null) {
            filePathView.setText(displayNameForUri(selectedFileUri));
        }
        appendLog("INFO 已选择文件: " + displayNameForUri(selectedFileUri));
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
        mapPage = buildMapPage();
        devicePage = buildDevicePage();
        pageContainer.addView(trajectoryPage);
        pageContainer.addView(mapPage);
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

        trajectoryViewParams = new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT,
                dp(330)
        );
        trajectoryViewParams.setMargins(0, dp(10), 0, dp(5));

        trajectoryView = new TrajectoryView(this);
        trajectoryView.setBackground(cardDrawable());
        page.addView(trajectoryView, trajectoryViewParams);
        page.addView(buildTrajectoryResizeHandle(page), new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT,
                dp(18)
        ));

        ScrollView metricsScroll = new ScrollView(this);
        metricsScroll.setFillViewport(false);
        LinearLayout metricsContent = new LinearLayout(this);
        metricsContent.setOrientation(LinearLayout.VERTICAL);
        metricsScroll.addView(metricsContent);
        LinearLayout.LayoutParams metricsScrollParams = new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT,
                0,
                1f
        );
        metricsScrollParams.setMargins(0, dp(5), 0, dp(10));

        LinearLayout metricGrid = card();
        metricsContent.addView(metricGrid, matchWrap());

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
        detailParams.setMargins(0, dp(10), 0, 0);
        metricsContent.addView(detailGrid, detailParams);

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

        page.addView(metricsScroll, metricsScrollParams);
        page.addView(buildControlBar(), matchWrap());
        return page;
    }

    private View buildTrajectoryResizeHandle(LinearLayout page) {
        FrameLayout handle = new FrameLayout(this);
        handle.setPadding(0, dp(5), 0, dp(5));
        View grip = new View(this);
        grip.setBackground(plainDrawable(Color.rgb(148, 163, 184), Color.TRANSPARENT, dp(3)));
        FrameLayout.LayoutParams gripParams = new FrameLayout.LayoutParams(
                dp(54),
                dp(4),
                Gravity.CENTER
        );
        handle.addView(grip, gripParams);
        handle.setOnTouchListener((view, event) -> {
            if (trajectoryViewParams == null) {
                return false;
            }
            switch (event.getActionMasked()) {
                case MotionEvent.ACTION_DOWN:
                    trajectoryResizeStartY = event.getRawY();
                    trajectoryResizeStartHeight = trajectoryViewParams.height;
                    view.setPressed(true);
                    grip.setAlpha(0.65f);
                    return true;
                case MotionEvent.ACTION_MOVE:
                    int delta = Math.round(event.getRawY() - trajectoryResizeStartY);
                    int minHeight = dp(180);
                    int maxHeight = Math.max(minHeight, page.getHeight() - dp(250));
                    int nextHeight = Math.max(minHeight, Math.min(maxHeight, trajectoryResizeStartHeight + delta));
                    if (trajectoryViewParams.height != nextHeight) {
                        trajectoryViewParams.height = nextHeight;
                        trajectoryView.setLayoutParams(trajectoryViewParams);
                    }
                    return true;
                case MotionEvent.ACTION_UP:
                case MotionEvent.ACTION_CANCEL:
                    view.setPressed(false);
                    grip.setAlpha(1f);
                    return true;
                default:
                    return false;
            }
        });
        return handle;
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

    private LinearLayout buildMapPage() {
        LinearLayout page = new LinearLayout(this);
        page.setOrientation(LinearLayout.VERTICAL);
        page.setPadding(dp(14), dp(12), dp(14), dp(10));
        page.setBackgroundColor(COLOR_PAGE);

        LinearLayout statusCard = card();
        mapStatusText = text("● 未连接 | 0 Hz | 包: 0", 14, COLOR_MUTED, Typeface.BOLD);
        statusCard.addView(mapStatusText, matchWrap());
        page.addView(statusCard, matchWrap());

        FrameLayout mapFrame = new FrameLayout(this);
        mapFrame.setBackground(cardDrawable());
        mapFrame.setPadding(dp(1), dp(1), dp(1), dp(1));
        LinearLayout.LayoutParams mapParams = new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT,
                0,
                1f
        );
        mapParams.setMargins(0, dp(10), 0, dp(10));

        amapMapView = new MapView(this);
        amapMapView.onCreate(mapSavedInstanceState);
        aMap = amapMapView.getMap();
        aMap.getUiSettings().setZoomControlsEnabled(false);
        aMap.getUiSettings().setZoomGesturesEnabled(true);
        aMap.getUiSettings().setCompassEnabled(true);
        aMap.getUiSettings().setScaleControlsEnabled(true);
        aMap.setOnMapTouchListener(event -> mapUserZoomLocked = true);
        aMap.setOnMapLoadedListener(() -> {
            amapLoaded = true;
            if (mapHasTrack && mapEmptyText != null) {
                mapEmptyText.setVisibility(View.GONE);
            }
            appendLog("INFO 高德地图加载完成");
        });
        aMap.moveCamera(CameraUpdateFactory.newLatLngZoom(new LatLng(referenceLatitude, referenceLongitude), 17f));
        handler.postDelayed(() -> {
            if (!amapLoaded && mapEmptyText != null) {
                mapEmptyText.setVisibility(View.VISIBLE);
                mapEmptyText.setText("高德地图未加载：请检查 Android Key、SHA1、包名和网络");
                appendLog("WARN 高德地图未加载，优先检查 Key/SHA1/包名是否匹配");
            }
        }, 6000);
        mapFrame.addView(amapMapView, new FrameLayout.LayoutParams(
                FrameLayout.LayoutParams.MATCH_PARENT,
                FrameLayout.LayoutParams.MATCH_PARENT
        ));

        mapEmptyText = text("等待轨迹数据，IMU/ENU 将按参考原点映射到地图", 15, COLOR_MUTED, Typeface.BOLD);
        mapEmptyText.setGravity(Gravity.CENTER);
        mapEmptyText.setBackgroundColor(Color.argb(210, 255, 255, 255));
        mapFrame.addView(mapEmptyText, new FrameLayout.LayoutParams(
                FrameLayout.LayoutParams.MATCH_PARENT,
                FrameLayout.LayoutParams.MATCH_PARENT
        ));
        page.addView(mapFrame, mapParams);

        LinearLayout infoCard = card();
        mapInfoText = text("速度 --   距离 0.0 m   航向 --", 14, COLOR_TEXT, Typeface.BOLD);
        mapInfoText.setGravity(Gravity.CENTER);
        infoCard.addView(mapInfoText, matchWrap());
        page.addView(infoCard, matchWrap());
        return page;
    }

    private LinearLayout buildDevicePage() {
        ScrollView scrollView = new ScrollView(this);
        scrollView.setFillViewport(true);

        LinearLayout page = new LinearLayout(this);
        page.setOrientation(LinearLayout.VERTICAL);
        page.setPadding(dp(10), dp(8), dp(10), dp(8));
        page.setBackgroundColor(COLOR_PAGE);
        scrollView.addView(page);

        page.addView(buildDeviceSummary(), matchWrap());
        page.addView(buildConnectionSettings(), compactSectionParams());
        page.addView(buildReferenceSettings(), compactSectionParams());
        page.addView(buildRuntimePanel(), compactSectionParams());

        LinearLayout wrapper = new LinearLayout(this);
        wrapper.setOrientation(LinearLayout.VERTICAL);
        wrapper.addView(scrollView, new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT,
                LinearLayout.LayoutParams.MATCH_PARENT
        ));
        return wrapper;
    }

    private View buildDeviceSummary() {
        LinearLayout card = compactCard();
        LinearLayout row = horizontalRow();
        TextView title = text("设备状态", 14, COLOR_TEXT, Typeface.BOLD);
        deviceConnection = text("● 未连接", 13, COLOR_RED, Typeface.BOLD);
        row.addView(title, new LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f));
        row.addView(deviceConnection);
        card.addView(row, matchWrap());

        deviceEndpoint = text("192.168.16.254:8000", 12, COLOR_MUTED, Typeface.NORMAL);
        deviceRuntime = text("等待 TCP 连接 | 0 Hz", 12, COLOR_MUTED, Typeface.NORMAL);
        card.addView(deviceEndpoint, topMargin(3));
        card.addView(deviceRuntime, topMargin(2));
        return card;
    }

    private View buildConnectionSettings() {
        LinearLayout card = compactCard();
        card.addView(text("连接与算法", 14, COLOR_TEXT, Typeface.BOLD), matchWrap());

        transportSpinner = spinner(new String[]{"TCP Client", "文件读取 File"});
        hostInput = editText("192.168.16.254");
        portInput = editText("8000");
        protocolSpinner = spinner(new String[]{"IMU CSV", "Position CSV"});
        modeSpinner = spinner(new String[]{"IMU 解算模式", "位置直显模式"});
        algorithmSpinner = spinner(new String[]{"v1_matlab_port", "v2_pdr_turn_snap"});
        connectButton = primaryButton("连接");
        connectButton.setOnClickListener(v -> toggleTransportConnection());

        card.addView(compactFormRow("通信", transportSpinner), topMargin(6));
        endpointRow = buildEndpointRow();
        fileRow = compactFormRow("文件", buildFilePickerControl());
        card.addView(endpointRow, topMargin(5));
        card.addView(fileRow, topMargin(5));
        card.addView(compactFormRow("协议", protocolSpinner), topMargin(5));
        card.addView(compactFormRow("模式", modeSpinner), topMargin(5));
        card.addView(compactFormRow("算法", algorithmSpinner), topMargin(5));
        LinearLayout.LayoutParams buttonParams = topMargin(7);
        buttonParams.height = dp(38);
        card.addView(connectButton, buttonParams);
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
        filePathView = text("未选择文件", 12, COLOR_MUTED, Typeface.NORMAL);
        filePathView.setSingleLine(true);
        filePathView.setPadding(dp(8), 0, dp(8), 0);
        filePathView.setBackground(plainDrawable(Color.rgb(248, 250, 252), COLOR_BORDER, dp(6)));
        Button chooseButton = smallButton("选择");
        chooseButton.setOnClickListener(v -> openFilePicker());
        row.addView(filePathView, new LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.MATCH_PARENT, 1f));
        LinearLayout.LayoutParams buttonParams = new LinearLayout.LayoutParams(dp(56), LinearLayout.LayoutParams.MATCH_PARENT);
        buttonParams.setMargins(dp(6), 0, 0, 0);
        row.addView(chooseButton, buttonParams);
        return row;
    }

    private LinearLayout buildEndpointRow() {
        LinearLayout row = new LinearLayout(this);
        row.setOrientation(LinearLayout.HORIZONTAL);
        row.setGravity(Gravity.CENTER_VERTICAL);
        TextView labelView = text("地址", 12, COLOR_MUTED, Typeface.NORMAL);
        row.addView(labelView, new LinearLayout.LayoutParams(dp(48), LinearLayout.LayoutParams.WRAP_CONTENT));

        LinearLayout inputs = new LinearLayout(this);
        inputs.setOrientation(LinearLayout.HORIZONTAL);
        inputs.addView(hostInput, new LinearLayout.LayoutParams(0, dp(36), 1f));
        LinearLayout.LayoutParams portParams = new LinearLayout.LayoutParams(dp(78), dp(36));
        portParams.setMargins(dp(6), 0, 0, 0);
        inputs.addView(portInput, portParams);
        row.addView(inputs, new LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f));
        return row;
    }

    private View buildModeSettings() {
        LinearLayout card = card();
        card.addView(text("工作模式", 16, COLOR_TEXT, Typeface.BOLD), matchWrap());
        modeSpinner = spinner(new String[]{"IMU 解算模式", "位置直显模式"});
        algorithmSpinner = spinner(new String[]{"v1_matlab_port", "v2_pdr_turn_snap"});
        card.addView(formRow("模式", modeSpinner), topMargin(10));
        card.addView(formRow("算法版本", algorithmSpinner), topMargin(8));
        return card;
    }

    private View buildReferenceSettings() {
        LinearLayout card = compactCard();
        LinearLayout titleRow = horizontalRow();
        titleRow.addView(text("地图参考原点", 14, COLOR_TEXT, Typeface.BOLD), new LinearLayout.LayoutParams(
                0,
                LinearLayout.LayoutParams.WRAP_CONTENT,
                1f
        ));
        Button gpsButton = smallButton("GPS定位");
        gpsButton.setOnClickListener(v -> locateReferenceWithGps());
        Button defaultButton = smallButton("默认定位");
        defaultButton.setOnClickListener(v -> applyDefaultReferenceLocation());
        titleRow.addView(gpsButton, new LinearLayout.LayoutParams(dp(76), dp(32)));
        LinearLayout.LayoutParams defaultButtonParams = new LinearLayout.LayoutParams(dp(76), dp(32));
        defaultButtonParams.setMargins(dp(6), 0, 0, 0);
        titleRow.addView(defaultButton, defaultButtonParams);
        card.addView(titleRow, matchWrap());
        referenceLatInput = editText(String.valueOf(DEFAULT_REFERENCE_LATITUDE));
        referenceLonInput = editText(String.valueOf(DEFAULT_REFERENCE_LONGITUDE));
        referenceAltInput = editText(String.valueOf(DEFAULT_REFERENCE_ALTITUDE));
        LinearLayout row = new LinearLayout(this);
        row.setOrientation(LinearLayout.HORIZONTAL);
        row.addView(compactStackField("纬度", referenceLatInput), compactFieldParams(0));
        row.addView(compactStackField("经度", referenceLonInput), compactFieldParams(dp(6)));
        row.addView(compactStackField("高度", referenceAltInput), compactFieldParams(dp(6)));
        card.addView(row, topMargin(6));
        return card;
    }

    private View buildRuntimePanel() {
        LinearLayout card = compactCard();
        LinearLayout titleRow = horizontalRow();
        titleRow.addView(text("运行状态", 14, COLOR_TEXT, Typeface.BOLD), new LinearLayout.LayoutParams(
                0,
                LinearLayout.LayoutParams.WRAP_CONTENT,
                1f
        ));
        Button clearLog = smallButton("清空日志");
        clearLog.setOnClickListener(v -> {
            logs.clear();
            updateLogText();
        });
        titleRow.addView(clearLog, new LinearLayout.LayoutParams(dp(76), dp(32)));
        card.addView(titleRow, matchWrap());

        recentRawText = text("原始行: --", 11, COLOR_MUTED, Typeface.NORMAL);
        recentRawText.setSingleLine(false);
        recentRawText.setMaxLines(2);
        recentRawText.setTypeface(Typeface.MONOSPACE);
        recentTrackText = text("轨迹点: --", 11, COLOR_MUTED, Typeface.NORMAL);
        recentTrackText.setSingleLine(true);
        recentTrackText.setTypeface(Typeface.MONOSPACE);
        latestRecordText = text("记录文件: 无", 11, COLOR_MUTED, Typeface.NORMAL);
        latestRecordText.setSingleLine(true);
        Button exportButton = smallButton("导出");
        exportRecordButton = exportButton;
        exportButton.setOnClickListener(v -> exportLatestRecording());

        card.addView(recentRawText, topMargin(5));
        card.addView(recentTrackText, topMargin(3));
        LinearLayout recordRow = horizontalRow();
        recordRow.addView(latestRecordText, new LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f));
        recordRow.addView(exportButton, new LinearLayout.LayoutParams(dp(56), dp(32)));
        card.addView(recordRow, topMargin(3));

        logText = text("", 11, COLOR_TEXT, Typeface.NORMAL);
        logText.setTypeface(Typeface.MONOSPACE);
        ScrollView logScroll = new ScrollView(this);
        logScroll.setBackground(plainDrawable(Color.rgb(248, 250, 252), COLOR_BORDER, dp(6)));
        logScroll.setPadding(dp(8), dp(5), dp(8), dp(5));
        logScroll.addView(logText);
        LinearLayout.LayoutParams logParams = topMargin(5);
        logParams.height = dp(86);
        card.addView(logScroll, logParams);
        return card;
    }

    private View buildRecentDataPanel() {
        LinearLayout card = card();
        card.addView(text("最近数据", 16, COLOR_TEXT, Typeface.BOLD), matchWrap());
        recentRawText = text("原始行: --", 12, COLOR_MUTED, Typeface.NORMAL);
        recentRawText.setTypeface(Typeface.MONOSPACE);
        recentTrackText = text("轨迹点: --", 12, COLOR_MUTED, Typeface.NORMAL);
        recentTrackText.setTypeface(Typeface.MONOSPACE);
        latestRecordText = text("记录文件: 无", 12, COLOR_MUTED, Typeface.NORMAL);
        Button exportButton = smallButton("导出最新记录");
        exportRecordButton = exportButton;
        exportButton.setOnClickListener(v -> exportLatestRecording());
        card.addView(recentRawText, topMargin(10));
        card.addView(recentTrackText, topMargin(6));
        card.addView(latestRecordText, topMargin(6));
        card.addView(exportButton, topMargin(10));
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
        mapTab = navButton("地图");
        deviceTab = navButton("设备");
        trajectoryTab.setOnClickListener(v -> showTrajectoryPage());
        mapTab.setOnClickListener(v -> showMapPage());
        deviceTab.setOnClickListener(v -> showDevicePage());
        nav.addView(trajectoryTab, buttonWeightParams());
        nav.addView(mapTab, buttonWeightParams());
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
        if (!applyReferenceFromInputs()) {
            return;
        }
        saveSettings();
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

        if ("IMU CSV".equals(currentProtocol())) {
            fileReaderThread = new Thread(() -> processImuFileBatch(selectedFileUri), "imu-file-batch-reader");
        } else {
            fileReaderThread = new Thread(() -> readSelectedFile(selectedFileUri), "imu-file-reader");
        }
        fileReaderThread.start();
    }

    private void stopFileRead(String reason, boolean updateUi) {
        fileReading = false;
        stopFileReplayTimer();
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
            while (fileReading && !Thread.currentThread().isInterrupted()) {
                if (receivingPaused) {
                    Thread.sleep(FILE_REPLAY_DELAY_MS);
                    continue;
                }
                line = reader.readLine();
                if (line == null) {
                    break;
                }
                String frame = line.trim();
                if (frame.isEmpty()) {
                    continue;
                }
                lineCount++;
                String finalFrame = frame;
                handler.post(() -> updateRecentRawLine(finalFrame));
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
                stopRecordingIfNeeded(false);
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
                stopRecordingIfNeeded(false);
                appendLog("ERROR 文件读取失败: " + status.message);
                updateAllViews();
            });
        }
    }

    private void processImuFileBatch(Uri uri) {
        List<ImuSample> samples = new ArrayList<>();
        List<String> rawFrames = new ArrayList<>();
        int lineCount = 0;
        int parseErrors = 0;
        Integer firstPacketId = null;
        boolean rawRecordedDuringBatch = false;
        try (InputStream stream = getContentResolver().openInputStream(uri);
             BufferedReader reader = stream == null ? null : new BufferedReader(new InputStreamReader(stream))) {
            if (reader == null) {
                throw new IOException("无法打开文件输入流");
            }
            String line;
            while (fileReading && !Thread.currentThread().isInterrupted()) {
                if (receivingPaused) {
                    Thread.sleep(FILE_REPLAY_DELAY_MS);
                    continue;
                }
                line = reader.readLine();
                if (line == null) {
                    break;
                }
                String frame = line.trim();
                if (frame.isEmpty()) {
                    continue;
                }
                lineCount++;
                if (lineCount == 1 || lineCount % 50 == 0) {
                    String finalFrame = frame;
                    handler.post(() -> updateRecentRawLine(finalFrame));
                }
                try {
                    int packetId = packetIdFromLine(frame);
                    if (firstPacketId == null) {
                        firstPacketId = packetId;
                    }
                    long timestampMillis = Math.round((packetId - firstPacketId) * 1000.0 / 100.0);
                    ImuSample sample = imuParser.parse(frame, timestampMillis);
                    samples.add(sample);
                    rawFrames.add(frame);
                    if (recording) {
                        trackRecorder.recordRaw(frame);
                        rawRecordedDuringBatch = true;
                    }
                } catch (ProtocolParseException | IOException error) {
                    parseErrors++;
                }
            }
            if (!fileReading) {
                return;
            }
            List<TrackPoint> points = imuNavigationProcessor.processBatch(samples);
            List<String> replayRawLines = buildReplayRawLines(points, samples, rawFrames);
            int finalLineCount = lineCount;
            int finalParseErrors = parseErrors;
            boolean finalRawRecordedDuringBatch = rawRecordedDuringBatch;
            handler.post(() -> {
                if (!fileReading) {
                    return;
                }
                appendLog("FILE IMU BATCH: algorithm=" + currentAlgorithmVersion()
                        + ", lines=" + finalLineCount
                        + ", samples=" + samples.size()
                        + ", track_points=" + points.size()
                        + ", parse_errors=" + finalParseErrors);
                startFileTrackReplay(points, replayRawLines, finalRawRecordedDuringBatch);
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
                status.message = error.getMessage() == null ? "IMU 文件批处理失败" : error.getMessage();
                status.dataRateHz = 0.0;
                stopRecordingIfNeeded(false);
                appendLog("ERROR IMU 文件批处理失败: " + status.message);
                updateAllViews();
            });
        }
    }

    private void toggleTcpConnection() {
        if (connected || (tcpClient != null && tcpClient.isRunning())) {
            disconnectTcp("手动断开");
            return;
        }
        if (!applyReferenceFromInputs()) {
            return;
        }
        saveSettings();
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
        stopRecordingIfNeeded(false);
        appendLog(reason);
        updateAllViews();
    }

    private void resumeReceiving() {
        if (!connected) {
            appendLog("WARN 当前没有可继续的数据源");
            return;
        }
        receivingPaused = false;
        status.running = true;
        if (fileReading) {
            status.connectionState = fileReplayRunnable == null ? "读取中" : "文件回放";
            status.message = fileReplayRunnable == null
                    ? "文件读取继续"
                    : "文件回放运行：" + fileReplayIndex + "/" + fileReplayPoints.size() + " 个轨迹点";
            if (fileReplayRunnable != null) {
                handler.removeCallbacks(fileReplayRunnable);
                handler.post(fileReplayRunnable);
            }
        } else {
            status.connectionState = "已连接";
            status.message = "接收中";
        }
        appendLog("继续接收数据");
        updateAllViews();
    }

    private void pauseReceiving() {
        if (!connected) {
            return;
        }
        receivingPaused = true;
        status.running = false;
        if (fileReading && fileReplayRunnable != null) {
            handler.removeCallbacks(fileReplayRunnable);
        }
        status.message = fileReading ? "已暂停文件读取" : "已暂停接收";
        appendLog(fileReading ? "已暂停文件读取/回放" : "已暂停接收，TCP 连接保持");
        updateAllViews();
    }

    private void toggleRecording() {
        if (recording) {
            stopRecordingIfNeeded(true);
            return;
        }
        try {
            trackRecorder.start(this);
            recording = true;
            appendLog("INFO 开始记录: " + trackRecorder.latestSummary());
        } catch (IOException error) {
            recording = false;
            appendLog("ERROR 无法开始记录: " + error.getMessage());
        }
        updateAllViews();
    }

    private void stopRecordingIfNeeded(boolean updateUi) {
        if (!recording && !trackRecorder.isRecording()) {
            return;
        }
        try {
            trackRecorder.stop();
            appendLog("INFO 停止记录: " + trackRecorder.latestSummary());
        } catch (IOException error) {
            appendLog("ERROR 停止记录失败: " + error.getMessage());
        }
        recording = false;
        if (updateUi) {
            updateAllViews();
        }
    }

    private void clearTrack() {
        resetRuntimeState(true);
        trajectoryView.clear();
        clearMapTrack();
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
        lastUiUpdateMillis = 0L;
        lastTrajectoryUpdateMillis = 0L;
        lastMapUpdateMillis = 0L;
        lastMapCameraMoveMillis = 0L;
        lastRawLineUpdateMillis = 0L;
        resetImuNavigationProcessor();
        referenceInitializedFromInput = false;
        if (recentRawText != null) {
            recentRawText.setText("原始行: --");
        }
        if (recentTrackText != null) {
            recentTrackText.setText("轨迹点: --");
        }
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
        updateRecentRawLine(line);
        if (recording) {
            try {
                trackRecorder.recordRaw(line);
            } catch (IOException error) {
                appendLog("ERROR 写入原始记录失败: " + error.getMessage());
                stopRecordingIfNeeded(true);
            }
        }
        long now = System.currentTimeMillis();
        try {
            TrackPoint point;
            markFrameReceived();
            if ("Position CSV".equals(currentProtocol())) {
                PositionFix fix = positionParser.parse(line, now);
                point = trackPointFromPosition(fix);
            } else {
                ImuSample sample = imuParser.parse(line, now);
                point = trackPointFromImu(sample);
            }
            if (point == null) {
                return;
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

    private void updateRecentRawLine(String line) {
        if (recentRawText == null) {
            return;
        }
        long now = System.currentTimeMillis();
        if (lastRawLineUpdateMillis != 0L && now - lastRawLineUpdateMillis < RAW_LINE_UPDATE_INTERVAL_MS) {
            return;
        }
        lastRawLineUpdateMillis = now;
        recentRawText.setText("原始行: " + trimForDisplay(line, 140));
    }

    private TrackPoint trackPointFromPosition(PositionFix fix) {
        double east;
        double north;
        double up;
        Double latitude = fix.latitude;
        Double longitude = fix.longitude;
        Double altitude = fix.altitude;
        if (fix.latitude != null && fix.longitude != null && fix.altitude != null) {
            double[] enu = CoordinateConverter.wgs84ToEnu(
                    fix.latitude,
                    fix.longitude,
                    fix.altitude,
                    referenceLatitude,
                    referenceLongitude,
                    referenceAltitude
            );
            east = enu[0];
            north = enu[1];
            up = enu[2];
        } else {
            east = valueOrZero(fix.x);
            north = valueOrZero(fix.y);
            up = valueOrZero(fix.z);
            double[] wgs84 = CoordinateConverter.enuToWgs84(
                    east,
                    north,
                    up,
                    referenceLatitude,
                    referenceLongitude,
                    referenceAltitude
            );
            latitude = wgs84[0];
            longitude = wgs84[1];
            altitude = wgs84[2];
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
                latitude,
                longitude,
                altitude,
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
        return imuNavigationProcessor.process(sample);
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
        if (recentTrackText != null) {
            recentTrackText.setText(String.format(
                    Locale.US,
                    "轨迹点: E %.2f / N %.2f / V %.2f / Y %s",
                    point.east,
                    point.north,
                    point.speed,
                    point.yaw == null ? "--" : Formatters.oneDecimal(point.yaw)
            ));
        }
        if (recording) {
            try {
                trackRecorder.recordTrack(point, totalDistance, currentProtocol());
            } catch (IOException error) {
                appendLog("ERROR 写入轨迹记录失败: " + error.getMessage());
                stopRecordingIfNeeded(true);
            }
        }
        updateLiveViewsThrottled();
    }

    private List<String> buildReplayRawLines(List<TrackPoint> points, List<ImuSample> samples, List<String> rawFrames) {
        List<String> replayRawLines = new ArrayList<>();
        if (points.isEmpty() || samples.isEmpty() || rawFrames.isEmpty()) {
            return replayRawLines;
        }
        int sampleIndex = 0;
        for (TrackPoint point : points) {
            while (sampleIndex + 1 < samples.size()) {
                long currentDiff = Math.abs(samples.get(sampleIndex).timestampMillis - point.timestampMillis);
                long nextDiff = Math.abs(samples.get(sampleIndex + 1).timestampMillis - point.timestampMillis);
                if (nextDiff > currentDiff) {
                    break;
                }
                sampleIndex++;
            }
            replayRawLines.add(sampleIndex < rawFrames.size() ? rawFrames.get(sampleIndex) : null);
        }
        return replayRawLines;
    }

    private void startFileTrackReplay(List<TrackPoint> points, List<String> rawLines, boolean rawAlreadyRecorded) {
        stopFileReplayTimer();
        trackPoints.clear();
        totalDistance = 0.0;
        previousPoint = null;
        clearMapTrack();
        trajectoryView.clear();
        fileReplayPoints.clear();
        fileReplayPoints.addAll(points);
        fileReplayRawLines.clear();
        fileReplayRawLines.addAll(rawLines);
        fileReplayIndex = 0;
        fileReplayRawAlreadyRecorded = rawAlreadyRecorded;
        status.packetCount = 0;
        framesSinceRateUpdate = 0;
        lastRateUpdateMillis = 0L;
        status.dataRateHz = 1000.0 / FILE_REPLAY_DELAY_MS;

        if (fileReplayPoints.isEmpty()) {
            fileReading = false;
            connected = false;
            status.running = false;
            status.connectionState = "已完成";
            status.message = "文件没有可回放轨迹点";
            stopRecordingIfNeeded(false);
            appendLog("FILE REPLAY DONE: 没有可回放轨迹点");
            updateAllViews();
            return;
        }

        fileReading = true;
        connected = true;
        status.running = true;
        status.connectionState = "文件回放";
        status.message = "文件回放运行：0/" + fileReplayPoints.size() + " 个轨迹点";
        appendLog("FILE REPLAY START: 从第 1 个轨迹点开始");
        updateAllViews();

        fileReplayRunnable = new Runnable() {
            @Override
            public void run() {
                replayNextFilePoint();
            }
        };
        handler.post(fileReplayRunnable);
    }

    private void replayNextFilePoint() {
        if (receivingPaused) {
            status.running = false;
            status.message = "已暂停文件回放：" + fileReplayIndex + "/" + fileReplayPoints.size() + " 个轨迹点";
            updateAllViews();
            return;
        }
        if (!fileReading || fileReplayIndex >= fileReplayPoints.size()) {
            finishFileTrackReplay();
            return;
        }
        int replayIndex = fileReplayIndex;
        TrackPoint point = fileReplayPoints.get(replayIndex);
        if (replayIndex < fileReplayRawLines.size()) {
            String rawLine = fileReplayRawLines.get(replayIndex);
            if (rawLine != null) {
                updateRecentRawLine(rawLine);
                recordReplayRawLineIfNeeded(rawLine);
            }
        }
        fileReplayIndex++;
        status.packetCount = fileReplayIndex;
        status.dataRateHz = 1000.0 / FILE_REPLAY_DELAY_MS;
        status.connectionState = "文件回放";
        status.running = true;
        status.message = "文件回放运行：" + fileReplayIndex + "/" + fileReplayPoints.size() + " 个轨迹点";
        appendTrackPoint(point);
        if (fileReplayIndex >= fileReplayPoints.size()) {
            finishFileTrackReplay();
            return;
        }
        if (fileReplayRunnable != null) {
            handler.postDelayed(fileReplayRunnable, FILE_REPLAY_DELAY_MS);
        }
    }

    private void recordReplayRawLineIfNeeded(String rawLine) {
        if (!recording || fileReplayRawAlreadyRecorded) {
            return;
        }
        try {
            trackRecorder.recordRaw(rawLine);
        } catch (IOException error) {
            appendLog("ERROR 写入回放原始记录失败: " + error.getMessage());
            stopRecordingIfNeeded(true);
        }
    }

    private void finishFileTrackReplay() {
        stopFileReplayTimer();
        fileReading = false;
        connected = false;
        status.running = false;
        status.connectionState = "已完成";
        status.message = "文件回放完成：" + fileReplayPoints.size() + " 个轨迹点";
        status.dataRateHz = 0.0;
        stopRecordingIfNeeded(false);
        appendLog("FILE REPLAY DONE: " + fileReplayPoints.size() + " 个轨迹点");
        updateAllViews();
    }

    private void stopFileReplayTimer() {
        if (fileReplayRunnable != null) {
            handler.removeCallbacks(fileReplayRunnable);
            fileReplayRunnable = null;
        }
    }

    private void markFrameReceived() {
        status.packetCount++;
        updateRate();
        status.connectionState = fileReading ? "读取中" : "已连接";
        status.running = true;
        String activeMessage = fileReading ? "文件读取中" : "接收中";
        status.message = recording ? activeMessage + " / 记录中" : activeMessage;
        updateLiveViewsThrottled();
    }

    private void replaceTrackPoints(List<TrackPoint> points) {
        trackPoints.clear();
        totalDistance = 0.0;
        previousPoint = null;
        int start = Math.max(0, points.size() - MAX_TRACK_POINTS);
        for (int i = start; i < points.size(); i++) {
            TrackPoint point = points.get(i);
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
            if (recording) {
                try {
                    trackRecorder.recordTrack(point, totalDistance, currentProtocol());
                } catch (IOException error) {
                    appendLog("ERROR 写入轨迹记录失败: " + error.getMessage());
                    stopRecordingIfNeeded(true);
                    break;
                }
            }
        }
        if (recentTrackText != null && previousPoint != null) {
            recentTrackText.setText(String.format(
                    Locale.US,
                    "轨迹点: E %.2f / N %.2f / V %.2f / Y %s",
                    previousPoint.east,
                    previousPoint.north,
                    previousPoint.speed,
                    previousPoint.yaw == null ? "--" : Formatters.oneDecimal(previousPoint.yaw)
            ));
        }
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

    private void resetImuNavigationProcessor() {
        imuNavigationProcessor.reset(referenceLatitude, referenceLongitude, referenceAltitude, currentAlgorithmVersion());
    }

    private void updateLiveViewsThrottled() {
        long now = System.currentTimeMillis();
        if (trajectoryView != null && now - lastTrajectoryUpdateMillis >= TRAJECTORY_UPDATE_INTERVAL_MS) {
            trajectoryView.setTrack(trackPoints);
            lastTrajectoryUpdateMillis = now;
        }
        if (aMap != null && now - lastMapUpdateMillis >= MAP_UPDATE_INTERVAL_MS) {
            pushTrackToMap(false);
            lastMapUpdateMillis = now;
        }
        if (now - lastUiUpdateMillis >= UI_UPDATE_INTERVAL_MS) {
            updateAllViews();
            lastUiUpdateMillis = now;
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
        if (mapStatusText != null) {
            mapStatusText.setText("● " + status.connectionState + " | " + Formatters.oneDecimal(status.dataRateHz) + " Hz | 包: " + status.packetCount);
            mapStatusText.setTextColor(connectionColor);
        }

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
            if (mapInfoText != null) {
                mapInfoText.setText("速度 --   距离 0.0 m   航向 --");
            }
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
            if (mapInfoText != null) {
                mapInfoText.setText("速度 " + Formatters.speed(point.speed)
                        + "   距离 " + Formatters.oneDecimal(totalDistance) + " m"
                        + "   航向 " + Formatters.degrees(point.yaw));
            }
        }

        startButton.setEnabled(connected && receivingPaused);
        pauseButton.setEnabled(connected && !receivingPaused);
        startButton.setAlpha(startButton.isEnabled() ? 1f : 0.45f);
        pauseButton.setAlpha(pauseButton.isEnabled() ? 1f : 0.45f);
        recordButton.setText(recording ? "停止记录" : "记录");
        recordButton.setBackground(buttonDrawable(recording ? COLOR_RED : Color.WHITE, COLOR_RED, dp(6)));
        recordButton.setTextColor(recording ? Color.WHITE : COLOR_RED);
        if (latestRecordText != null) {
            latestRecordText.setText("记录文件: " + trackRecorder.latestSummary());
        }
        if (exportRecordButton != null) {
            exportRecordButton.setEnabled(!trackRecorder.latestFiles().isEmpty());
            exportRecordButton.setAlpha(exportRecordButton.isEnabled() ? 1f : 0.45f);
        }

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

    private void pushTrackToMap(boolean forceMoveCamera) {
        if (aMap == null) {
            return;
        }
        List<LatLng> latLngs = new ArrayList<>();
        for (TrackPoint point : trackPoints) {
            if (point.latitude == null || point.longitude == null) {
                continue;
            }
            if (!Double.isFinite(point.latitude) || !Double.isFinite(point.longitude)) {
                continue;
            }
            latLngs.add(new LatLng(point.latitude, point.longitude));
        }
        mapHasTrack = !latLngs.isEmpty();
        if (mapEmptyText != null) {
            mapEmptyText.setVisibility(mapHasTrack ? View.GONE : View.VISIBLE);
            mapEmptyText.setText(mapHasTrack ? "" : "等待轨迹数据，IMU/ENU 将按参考原点映射到地图");
        }
        if (!mapHasTrack) {
            return;
        }
        if (amapPolyline == null) {
            amapPolyline = aMap.addPolyline(new PolylineOptions()
                    .addAll(latLngs)
                    .width(dp(5))
                    .color(COLOR_BLUE));
        } else {
            amapPolyline.setPoints(latLngs);
        }
        LatLng current = latLngs.get(latLngs.size() - 1);
        if (amapMarker == null) {
            amapMarker = aMap.addMarker(new MarkerOptions()
                    .position(current)
                    .icon(BitmapDescriptorFactory.defaultMarker(BitmapDescriptorFactory.HUE_GREEN))
                    .title("当前位置"));
        } else {
            amapMarker.setPosition(current);
        }
        if (mapUserZoomLocked && !forceMoveCamera) {
            return;
        }
        long now = System.currentTimeMillis();
        boolean shouldMoveCamera = forceMoveCamera || lastMapCameraMoveMillis == 0L || now - lastMapCameraMoveMillis >= MAP_CAMERA_INTERVAL_MS;
        if (!shouldMoveCamera) {
            return;
        }
        lastMapCameraMoveMillis = now;
        if (latLngs.size() == 1) {
            aMap.animateCamera(CameraUpdateFactory.newLatLngZoom(current, 18f));
        } else {
            LatLngBounds.Builder builder = LatLngBounds.builder();
            for (LatLng latLng : latLngs) {
                builder.include(latLng);
            }
            aMap.animateCamera(CameraUpdateFactory.newLatLngBounds(builder.build(), dp(48)));
        }
    }

    private void clearMapTrack() {
        mapHasTrack = false;
        mapUserZoomLocked = false;
        if (mapEmptyText != null) {
            mapEmptyText.setVisibility(View.VISIBLE);
            mapEmptyText.setText("等待轨迹数据，IMU/ENU 将按参考原点映射到地图");
        }
        if (amapPolyline != null) {
            amapPolyline.remove();
            amapPolyline = null;
        }
        if (amapMarker != null) {
            amapMarker.remove();
            amapMarker = null;
        }
    }

    private String currentProtocol() {
        return protocolSpinner == null ? "IMU CSV" : String.valueOf(protocolSpinner.getSelectedItem());
    }

    private String currentMode() {
        return modeSpinner == null ? status.mode : String.valueOf(modeSpinner.getSelectedItem());
    }

    private String currentAlgorithmVersion() {
        String value = algorithmSpinner == null ? ImuNavigationProcessor.V1_MATLAB_PORT : String.valueOf(algorithmSpinner.getSelectedItem());
        return ImuNavigationProcessor.normalizeAlgorithmVersion(value);
    }

    private void locateReferenceWithGps() {
        if (!hasLocationPermission()) {
            ActivityCompat.requestPermissions(
                    this,
                    new String[]{Manifest.permission.ACCESS_FINE_LOCATION, Manifest.permission.ACCESS_COARSE_LOCATION},
                    REQUEST_LOCATION_PERMISSION
            );
            return;
        }
        requestReferenceLocation();
    }

    private boolean hasLocationPermission() {
        return ContextCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION) == PackageManager.PERMISSION_GRANTED
                || ContextCompat.checkSelfPermission(this, Manifest.permission.ACCESS_COARSE_LOCATION) == PackageManager.PERMISSION_GRANTED;
    }

    private void requestReferenceLocation() {
        LocationManager locationManager = (LocationManager) getSystemService(Context.LOCATION_SERVICE);
        if (locationManager == null) {
            appendLog("ERROR 系统定位服务不可用");
            return;
        }
        try {
            Location lastLocation = bestLastKnownLocation(locationManager);
            if (lastLocation != null) {
                applyReferenceLocation(lastLocation, "最近定位");
            }

            String provider = null;
            if (locationManager.isProviderEnabled(LocationManager.GPS_PROVIDER)) {
                provider = LocationManager.GPS_PROVIDER;
            } else if (locationManager.isProviderEnabled(LocationManager.NETWORK_PROVIDER)) {
                provider = LocationManager.NETWORK_PROVIDER;
            }
            if (provider == null) {
                appendLog("WARN 请先打开手机定位服务");
                return;
            }
            appendLog("INFO 正在获取 GPS 参考原点...");
            locationManager.requestSingleUpdate(provider, new LocationListener() {
                @Override
                public void onLocationChanged(Location location) {
                    applyReferenceLocation(location, "GPS定位");
                }

                @Override
                public void onStatusChanged(String provider, int status, Bundle extras) {
                }

                @Override
                public void onProviderEnabled(String provider) {
                }

                @Override
                public void onProviderDisabled(String provider) {
                    appendLog("WARN 定位服务已关闭");
                }
            }, Looper.getMainLooper());
        } catch (SecurityException error) {
            appendLog("ERROR 定位权限不可用: " + error.getMessage());
        } catch (IllegalArgumentException error) {
            appendLog("ERROR 定位提供方不可用: " + error.getMessage());
        }
    }

    private Location bestLastKnownLocation(LocationManager locationManager) {
        Location best = null;
        for (String provider : new String[]{LocationManager.GPS_PROVIDER, LocationManager.NETWORK_PROVIDER}) {
            try {
                if (!locationManager.isProviderEnabled(provider)) {
                    continue;
                }
                Location location = locationManager.getLastKnownLocation(provider);
                if (location != null && (best == null || location.getTime() > best.getTime())) {
                    best = location;
                }
            } catch (SecurityException | IllegalArgumentException ignored) {
                // Permission and provider availability are handled by the caller.
            }
        }
        return best;
    }

    private void applyReferenceLocation(Location location, String source) {
        double altitude = location.hasAltitude() ? location.getAltitude() : referenceAltitude;
        referenceLatInput.setText(String.format(Locale.US, "%.6f", location.getLatitude()));
        referenceLonInput.setText(String.format(Locale.US, "%.6f", location.getLongitude()));
        referenceAltInput.setText(String.format(Locale.US, "%.1f", altitude));
        applyReferenceInputsFromUi(source);
    }

    private void applyDefaultReferenceLocation() {
        referenceLatInput.setText(String.format(Locale.US, "%.6f", DEFAULT_REFERENCE_LATITUDE));
        referenceLonInput.setText(String.format(Locale.US, "%.6f", DEFAULT_REFERENCE_LONGITUDE));
        referenceAltInput.setText(String.format(Locale.US, "%.1f", DEFAULT_REFERENCE_ALTITUDE));
        applyReferenceInputsFromUi("默认定位");
    }

    private void applyReferenceInputsFromUi(String source) {
        referenceInitializedFromInput = false;
        if (!applyReferenceFromInputs()) {
            return;
        }
        saveSettings();
        if (trackPoints.isEmpty()) {
            resetImuNavigationProcessor();
        } else {
            appendLog("WARN 当前已有轨迹，新的参考原点建议下次读取/连接前使用");
        }
        if (aMap != null) {
            mapUserZoomLocked = false;
            aMap.animateCamera(CameraUpdateFactory.newLatLngZoom(
                    new LatLng(referenceLatitude, referenceLongitude),
                    18f
            ));
        }
        appendLog("INFO " + source + "已写入参考原点");
        updateAllViews();
    }

    private boolean applyReferenceFromInputs() {
        try {
            double latitude = Double.parseDouble(referenceLatInput.getText().toString().trim());
            double longitude = Double.parseDouble(referenceLonInput.getText().toString().trim());
            double altitude = Double.parseDouble(referenceAltInput.getText().toString().trim());
            if (!Double.isFinite(latitude) || !Double.isFinite(longitude) || !Double.isFinite(altitude)) {
                throw new NumberFormatException("参考原点包含非法数字");
            }
            if (latitude < -90.0 || latitude > 90.0 || longitude < -180.0 || longitude > 180.0) {
                appendLog("ERROR 参考原点经纬度范围不正确");
                return false;
            }
            referenceLatitude = latitude;
            referenceLongitude = longitude;
            referenceAltitude = altitude;
            if (!referenceInitializedFromInput) {
                appendLog("INFO 地图参考原点: "
                        + Formatters.optional(referenceLatitude, 6)
                        + ", "
                        + Formatters.optional(referenceLongitude, 6)
                        + ", "
                        + Formatters.optional(referenceAltitude, 1)
                        + "m");
                referenceInitializedFromInput = true;
            }
            return true;
        } catch (NumberFormatException error) {
            appendLog("ERROR 参考原点必须是数字");
            return false;
        }
    }

    private void loadSettings() {
        SharedPreferences prefs = getSharedPreferences(PREFS_NAME, MODE_PRIVATE);
        hostInput.setText(prefs.getString("host", "192.168.16.254"));
        portInput.setText(prefs.getString("port", "8000"));
        referenceLatInput.setText(prefs.getString("reference_latitude", String.valueOf(DEFAULT_REFERENCE_LATITUDE)));
        referenceLonInput.setText(prefs.getString("reference_longitude", String.valueOf(DEFAULT_REFERENCE_LONGITUDE)));
        referenceAltInput.setText(prefs.getString("reference_altitude", String.valueOf(DEFAULT_REFERENCE_ALTITUDE)));
        setSpinnerValue(transportSpinner, prefs.getString("transport", "TCP Client"));
        setSpinnerValue(protocolSpinner, prefs.getString("protocol", "IMU CSV"));
        setSpinnerValue(modeSpinner, prefs.getString("mode", "IMU 解算模式"));
        setSpinnerValue(algorithmSpinner, prefs.getString("algorithm", ImuNavigationProcessor.V1_MATLAB_PORT));
        applyReferenceFromInputs();
        updateTransportUi();
    }

    private void saveSettings() {
        if (hostInput == null || portInput == null) {
            return;
        }
        getSharedPreferences(PREFS_NAME, MODE_PRIVATE)
                .edit()
                .putString("host", hostInput.getText().toString())
                .putString("port", portInput.getText().toString())
                .putString("transport", currentTransport())
                .putString("protocol", currentProtocol())
                .putString("mode", currentMode())
                .putString("algorithm", currentAlgorithmVersion())
                .putString("reference_latitude", referenceLatInput.getText().toString())
                .putString("reference_longitude", referenceLonInput.getText().toString())
                .putString("reference_altitude", referenceAltInput.getText().toString())
                .apply();
    }

    private void setSpinnerValue(Spinner spinner, String value) {
        if (spinner == null || value == null) {
            return;
        }
        for (int i = 0; i < spinner.getCount(); i++) {
            if (value.equals(String.valueOf(spinner.getItemAtPosition(i)))) {
                spinner.setSelection(i);
                return;
            }
        }
    }

    private void exportLatestRecording() {
        List<File> files = trackRecorder.latestFiles();
        if (files.isEmpty()) {
            appendLog("WARN 暂无可导出的记录文件");
            return;
        }
        if (recording) {
            stopRecordingIfNeeded(true);
        }
        try {
            File exportFile = createRecordingZip(files);
            Uri uri = FileProvider.getUriForFile(this, getPackageName() + ".fileprovider", exportFile);
            Intent intent = new Intent(Intent.ACTION_SEND);
            intent.setType("application/zip");
            intent.putExtra(Intent.EXTRA_STREAM, uri);
            intent.putExtra(Intent.EXTRA_SUBJECT, "IMU 记录数据");
            intent.addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION);
            intent.setClipData(ClipData.newUri(getContentResolver(), exportFile.getName(), uri));
            grantSharePermissions(intent, uri);
            appendLog("INFO 导出记录压缩包: " + exportFile.getName());
            startActivity(Intent.createChooser(intent, "分享 IMU 记录到微信/QQ"));
        } catch (Exception error) {
            appendLog("ERROR 导出记录失败: " + error.getMessage());
        }
    }

    private File createRecordingZip(List<File> files) throws IOException {
        File directory = getExternalCacheDir();
        if (directory == null) {
            directory = getCacheDir();
        }
        if (!directory.exists() && !directory.mkdirs()) {
            throw new IOException("无法创建导出缓存目录");
        }
        String stamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
        File zipFile = new File(directory, "imu_record_" + stamp + ".zip");
        byte[] buffer = new byte[8192];
        try (ZipOutputStream zip = new ZipOutputStream(new FileOutputStream(zipFile))) {
            for (File file : files) {
                if (file == null || !file.exists() || !file.isFile()) {
                    continue;
                }
                zip.putNextEntry(new ZipEntry(file.getName()));
                try (FileInputStream input = new FileInputStream(file)) {
                    int count;
                    while ((count = input.read(buffer)) >= 0) {
                        zip.write(buffer, 0, count);
                    }
                }
                zip.closeEntry();
            }
        }
        return zipFile;
    }

    private void grantSharePermissions(Intent intent, Uri uri) {
        List<ResolveInfo> targets = getPackageManager().queryIntentActivities(intent, 0);
        for (ResolveInfo target : targets) {
            if (target.activityInfo != null && target.activityInfo.packageName != null) {
                grantUriPermission(target.activityInfo.packageName, uri, Intent.FLAG_GRANT_READ_URI_PERMISSION);
            }
        }
    }

    private void updateTransportUi() {
        boolean fileMode = "文件读取 File".equals(currentTransport());
        if (endpointRow != null) {
            endpointRow.setVisibility(fileMode ? View.GONE : View.VISIBLE);
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
        logs.add(time + "  " + normalizeLogMessage(message));
        while (logs.size() > 100) {
            logs.remove(0);
        }
        updateLogText();
    }

    private String normalizeLogMessage(String message) {
        if (message == null || message.trim().isEmpty()) {
            return "INFO --";
        }
        String trimmed = message.trim();
        if (trimmed.startsWith("INFO ") || trimmed.startsWith("WARN ") || trimmed.startsWith("ERROR ")) {
            return trimmed;
        }
        return "INFO " + trimmed;
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
        mapPage.setVisibility(View.GONE);
        devicePage.setVisibility(View.GONE);
        trajectoryTab.setTextColor(Color.WHITE);
        trajectoryTab.setBackground(buttonDrawable(COLOR_BLUE, COLOR_BLUE, dp(8)));
        mapTab.setTextColor(COLOR_MUTED);
        mapTab.setBackground(buttonDrawable(Color.WHITE, COLOR_BORDER, dp(8)));
        deviceTab.setTextColor(COLOR_MUTED);
        deviceTab.setBackground(buttonDrawable(Color.WHITE, COLOR_BORDER, dp(8)));
    }

    private void showMapPage() {
        trajectoryPage.setVisibility(View.GONE);
        mapPage.setVisibility(View.VISIBLE);
        devicePage.setVisibility(View.GONE);
        trajectoryTab.setTextColor(COLOR_MUTED);
        trajectoryTab.setBackground(buttonDrawable(Color.WHITE, COLOR_BORDER, dp(8)));
        mapTab.setTextColor(Color.WHITE);
        mapTab.setBackground(buttonDrawable(COLOR_BLUE, COLOR_BLUE, dp(8)));
        deviceTab.setTextColor(COLOR_MUTED);
        deviceTab.setBackground(buttonDrawable(Color.WHITE, COLOR_BORDER, dp(8)));
        pushTrackToMap(true);
    }

    private void showDevicePage() {
        trajectoryPage.setVisibility(View.GONE);
        mapPage.setVisibility(View.GONE);
        devicePage.setVisibility(View.VISIBLE);
        trajectoryTab.setTextColor(COLOR_MUTED);
        trajectoryTab.setBackground(buttonDrawable(Color.WHITE, COLOR_BORDER, dp(8)));
        mapTab.setTextColor(COLOR_MUTED);
        mapTab.setBackground(buttonDrawable(Color.WHITE, COLOR_BORDER, dp(8)));
        deviceTab.setTextColor(Color.WHITE);
        deviceTab.setBackground(buttonDrawable(COLOR_BLUE, COLOR_BLUE, dp(8)));
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

    private LinearLayout compactFormRow(String label, View input) {
        LinearLayout row = new LinearLayout(this);
        row.setOrientation(LinearLayout.HORIZONTAL);
        row.setGravity(Gravity.CENTER_VERTICAL);
        TextView labelView = text(label, 12, COLOR_MUTED, Typeface.NORMAL);
        row.addView(labelView, new LinearLayout.LayoutParams(dp(48), LinearLayout.LayoutParams.WRAP_CONTENT));
        row.addView(input, new LinearLayout.LayoutParams(0, dp(36), 1f));
        return row;
    }

    private LinearLayout compactStackField(String label, View input) {
        LinearLayout field = new LinearLayout(this);
        field.setOrientation(LinearLayout.VERTICAL);
        TextView labelView = text(label, 11, COLOR_MUTED, Typeface.NORMAL);
        field.addView(labelView, matchWrap());
        field.addView(input, new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT,
                dp(36)
        ));
        return field;
    }

    private LinearLayout.LayoutParams compactFieldParams(int leftMargin) {
        LinearLayout.LayoutParams params = new LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f);
        params.setMargins(leftMargin, 0, 0, 0);
        return params;
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

    private LinearLayout compactCard() {
        LinearLayout card = new LinearLayout(this);
        card.setOrientation(LinearLayout.VERTICAL);
        card.setPadding(dp(10), dp(8), dp(10), dp(8));
        card.setBackground(cardDrawable());
        return card;
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
        button.setBackground(buttonDrawable(Color.WHITE, color, dp(6)));
        return button;
    }

    private Button primaryButton(String text) {
        Button button = new Button(this);
        button.setAllCaps(false);
        button.setText(text);
        button.setTextSize(14);
        button.setTypeface(Typeface.DEFAULT, Typeface.BOLD);
        button.setTextColor(Color.WHITE);
        button.setBackground(buttonDrawable(COLOR_BLUE, COLOR_BLUE, dp(6)));
        return button;
    }

    private Button smallButton(String text) {
        Button button = new Button(this);
        button.setAllCaps(false);
        button.setText(text);
        button.setTextSize(12);
        button.setTextColor(COLOR_BLUE);
        button.setBackground(buttonDrawable(Color.WHITE, COLOR_BORDER, dp(6)));
        return button;
    }

    private Button navButton(String text) {
        Button button = new Button(this);
        button.setAllCaps(false);
        button.setText(text);
        button.setTextSize(15);
        button.setTypeface(Typeface.DEFAULT, Typeface.BOLD);
        button.setBackground(buttonDrawable(Color.WHITE, COLOR_BORDER, dp(8)));
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

    private StateListDrawable buttonDrawable(int fill, int stroke, int radius) {
        StateListDrawable drawable = new StateListDrawable();
        drawable.addState(new int[]{android.R.attr.state_pressed}, plainDrawable(pressedColor(fill), stroke, radius));
        drawable.addState(new int[]{android.R.attr.state_focused}, plainDrawable(pressedColor(fill), stroke, radius));
        drawable.addState(new int[]{-android.R.attr.state_enabled}, plainDrawable(disabledColor(fill), disabledColor(stroke), radius));
        drawable.addState(new int[]{}, plainDrawable(fill, stroke, radius));
        return drawable;
    }

    private int pressedColor(int color) {
        return blendColor(color, Color.BLACK, isLightColor(color) ? 0.08f : 0.18f);
    }

    private int disabledColor(int color) {
        return blendColor(color, Color.WHITE, 0.55f);
    }

    private boolean isLightColor(int color) {
        return Color.red(color) + Color.green(color) + Color.blue(color) > 600;
    }

    private int blendColor(int from, int to, float ratio) {
        float inverse = 1f - ratio;
        return Color.rgb(
                Math.round(Color.red(from) * inverse + Color.red(to) * ratio),
                Math.round(Color.green(from) * inverse + Color.green(to) * ratio),
                Math.round(Color.blue(from) * inverse + Color.blue(to) * ratio)
        );
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

    private LinearLayout.LayoutParams compactSectionParams() {
        LinearLayout.LayoutParams params = matchWrap();
        params.setMargins(0, dp(6), 0, 0);
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

    private int packetIdFromLine(String line) throws ProtocolParseException {
        int commaIndex = line.indexOf(',');
        String firstField = commaIndex >= 0 ? line.substring(0, commaIndex) : line;
        try {
            return (int) Math.round(Double.parseDouble(firstField.trim()));
        } catch (NumberFormatException error) {
            throw new ProtocolParseException("包号字段无效。");
        }
    }

    private String trimForDisplay(String value, int maxLength) {
        if (value == null) {
            return "--";
        }
        String trimmed = value.trim();
        if (trimmed.length() <= maxLength) {
            return trimmed;
        }
        return trimmed.substring(0, Math.max(0, maxLength - 3)) + "...";
    }

    private double normalizeDegrees(double value) {
        double normalized = value % 360.0;
        return normalized < 0.0 ? normalized + 360.0 : normalized;
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
                stopRecordingIfNeeded(false);
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
                stopRecordingIfNeeded(false);
                appendLog("ERROR TCP: " + status.message);
                updateAllViews();
            });
        }
    }
}
