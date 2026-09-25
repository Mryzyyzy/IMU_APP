%% ========================================================================
%  Boat INS SCH16T-K01 / BMM350
%
%  Version: V3
%
%  Application:
%  Chengdu-Yibin Hejiangmen Yangtze River Boat Test
%
%  Navigation Frame:
%  ENU
%
%  Body Frame:
%  FLU
%
%      Xb : Forward
%      Yb : Left
%      Zb : Up
%
%  IMU Installation:
%
%      Xi : Left
%      Yi : Backward
%      Zi : Up
%
%  Therefore:
%
%      Xb = -Yi
%      Yb =  Xi
%      Zb =  Zi
%
%  Features:
%
%  1. Raw IMU output
%  2. Initial alignment
%  3. Gyroscope bias compensation
%  4. Quaternion attitude propagation
%  5. Accelerometer attitude correction
%  6. Ship lateral / vertical velocity constraint
%  7. ZUPT
%  8. Pure INS trajectory
%  9. GPS trajectory comparison
%  10. Eight result figures
%  11. CSV and MAT result output
%
% ========================================================================

clear;
clc;
close all;


%% ========================================================================
% 1. CONFIGURATION
% ========================================================================

cfg = ConfigBoatINS();


%% ========================================================================
% 2. LOAD IMU
% ========================================================================

imu = LoadBoatIMU(cfg);

fprintf('\n');
fprintf('IMU Samples       : %d\n', imu.N);
fprintf('Sampling Rate     : %.2f Hz\n', imu.fs);
fprintf('Duration          : %.2f s\n', imu.t(end));


%% ========================================================================
% 3. LOAD GPS
% ========================================================================

gps = LoadBoatGPS(cfg);

gpsENU = [];

if ~isempty(gps.lat)

    gpsENU = GPS2ENU(gps);

    fprintf('\nGPS Samples       : %d\n', gps.N);

else

    fprintf('\nGPS data unavailable.\n');

end


%% ========================================================================
% 4. IMU INSTALLATION TRANSFORMATION
% ========================================================================

[acc_b, gyro_b, mag_b] = ...
    TransformIMUToBody( ...
    imu.acc, ...
    imu.gyro, ...
    imu.mag, ...
    cfg);


%% ========================================================================
% 5. STATIC SEGMENT DETECTION
% ========================================================================

staticInfo = ...
    DetectStaticSegment( ...
    acc_b, ...
    gyro_b, ...
    imu.fs, ...
    cfg);


fprintf('\n');
fprintf('Static Alignment Window\n');

fprintf('Start Time = %.2f s\n', ...
    staticInfo.startTime);

fprintf('End Time   = %.2f s\n', ...
    staticInfo.endTime);


%% ========================================================================
% 6. INITIAL ALIGNMENT
% ========================================================================

init = InitialAlignmentBoat( ...
    acc_b, ...
    gyro_b, ...
    mag_b, ...
    staticInfo, ...
    gpsENU, ...
    cfg);


%% ========================================================================
% 7. RUN INS
% ========================================================================

fprintf('\n');
fprintf('====================================================\n');
fprintf('Starting Boat INS Mechanization...\n');
fprintf('====================================================\n');


tic;

result = BoatINSMechanization( ...
    imu.t, ...
    acc_b, ...
    gyro_b, ...
    mag_b, ...
    init, ...
    cfg);

elapsedTime = toc;


fprintf('\n');
fprintf('====================================================\n');
fprintf('INS Mechanization Completed\n');
fprintf('Processing Time = %.2f s\n', elapsedTime);
fprintf('====================================================\n');


%% ========================================================================
% 8. GPS COMPARISON
% ========================================================================

evalResult = [];

if ~isempty(gpsENU)

    [insSync, gpsSync] = ...
        SynchronizeINSandGPS( ...
        result, ...
        gpsENU);

    if ~isempty(insSync)

        evalResult = ...
            EvaluateBoatINS( ...
            insSync, ...
            gpsSync);

        fprintf('\n');
        fprintf('GPS Comparison Result\n');

        fprintf('Horizontal RMSE = %.3f m\n', ...
            evalResult.horizontalRMSE);

        fprintf('3D RMSE         = %.3f m\n', ...
            evalResult.posRMSE);

        fprintf('Final Horizontal Error = %.3f m\n', ...
            evalResult.finalHorizontalError);

    else

        fprintf('\nGPS time synchronization unavailable.\n');

    end

else

    insSync = [];
    gpsSync = [];

end


%% ========================================================================
% 9. SAVE RESULTS
% ========================================================================

ExportBoatINSResults( ...
    imu, ...
    result, ...
    gpsENU, ...
    evalResult, ...
    cfg);


%% ========================================================================
% 10. FIGURE 1
% RAW ACCELEROMETER
% ========================================================================

fprintf('\nGenerating Figure 1: Raw Accelerometer...\n');

figure(1);
clf;

plot( ...
    imu.t, ...
    imu.acc(:,1) / cfg.g0, ...
    'LineWidth', 0.8);

hold on;

plot( ...
    imu.t, ...
    imu.acc(:,2) / cfg.g0, ...
    'LineWidth', 0.8);

plot( ...
    imu.t, ...
    imu.acc(:,3) / cfg.g0, ...
    'LineWidth', 0.8);

grid on;

xlabel('Time [s]');

ylabel('Acceleration [g]');

title('Raw Accelerometer');

legend('X','Y','Z');

SaveFigure( ...
    figure(1), ...
    'Figure1_RawAccelerometer.png', ...
    cfg);


%% ========================================================================
% 11. FIGURE 2
% RAW GYROSCOPE
% ========================================================================

fprintf('Generating Figure 2: Raw Gyroscope...\n');

figure(2);
clf;

plot( ...
    imu.t, ...
    rad2deg(imu.gyro(:,1)), ...
    'LineWidth', 0.8);

hold on;

plot( ...
    imu.t, ...
    rad2deg(imu.gyro(:,2)), ...
    'LineWidth', 0.8);

plot( ...
    imu.t, ...
    rad2deg(imu.gyro(:,3)), ...
    'LineWidth', 0.8);

grid on;

xlabel('Time [s]');

ylabel('Angular Rate [deg/s]');

title('Raw Gyroscope');

legend('X','Y','Z');

SaveFigure( ...
    figure(2), ...
    'Figure2_RawGyroscope.png', ...
    cfg);


%% ========================================================================
% 12. FIGURE 3
% RAW MAGNETOMETER
% ========================================================================

fprintf('Generating Figure 3: Raw Magnetometer...\n');

figure(3);
clf;

plot( ...
    imu.t, ...
    imu.mag(:,1), ...
    'LineWidth', 0.8);

hold on;

plot( ...
    imu.t, ...
    imu.mag(:,2), ...
    'LineWidth', 0.8);

plot( ...
    imu.t, ...
    imu.mag(:,3), ...
    'LineWidth', 0.8);

grid on;

xlabel('Time [s]');

ylabel('Magnetic Field [uT]');

title('Raw Magnetometer');

legend('X','Y','Z');

SaveFigure( ...
    figure(3), ...
    'Figure3_RawMagnetometer.png', ...
    cfg);


%% ========================================================================
% 13. FIGURE 4
% ATTITUDE
% ========================================================================

fprintf('Generating Figure 4: Boat Attitude...\n');

figure(4);
clf;


subplot(3,1,1);

plot( ...
    result.t, ...
    result.euler(:,1), ...
    'LineWidth', 1);

grid on;

ylabel('Roll [deg]');

title('Boat Attitude');


subplot(3,1,2);

plot( ...
    result.t, ...
    result.euler(:,2), ...
    'LineWidth', 1);

grid on;

ylabel('Pitch [deg]');


subplot(3,1,3);

plot( ...
    result.t, ...
    result.euler(:,3), ...
    'LineWidth', 1);

grid on;

ylabel('Yaw [deg]');

xlabel('Time [s]');

SaveFigure( ...
    figure(4), ...
    'Figure4_Attitude.png', ...
    cfg);


%% ========================================================================
% 14. FIGURE 5
% VELOCITY
% ========================================================================

fprintf('Generating Figure 5: INS Velocity...\n');

figure(5);
clf;

plot( ...
    result.t, ...
    result.vel(:,1), ...
    'LineWidth', 1);

hold on;

plot( ...
    result.t, ...
    result.vel(:,2), ...
    'LineWidth', 1);

plot( ...
    result.t, ...
    result.vel(:,3), ...
    'LineWidth', 1);

grid on;

xlabel('Time [s]');

ylabel('Velocity [m/s]');

title('ENU Velocity');

legend('East','North','Up');

SaveFigure( ...
    figure(5), ...
    'Figure5_Velocity.png', ...
    cfg);


%% ========================================================================
% 15. FIGURE 6
% INS 2D TRAJECTORY
% ========================================================================

fprintf('Generating Figure 6: INS 2D Trajectory...\n');

figure(6);
clf;

plot( ...
    result.pos(:,1), ...
    result.pos(:,2), ...
    'LineWidth', 1.5);

grid on;

axis equal;

xlabel('East [m]');

ylabel('North [m]');

title('Pure INS 2D Boat Trajectory');

SaveFigure( ...
    figure(6), ...
    'Figure6_INS2DTrajectory.png', ...
    cfg);


%% ========================================================================
% 16. FIGURE 7
% INS 3D TRAJECTORY
% ========================================================================

fprintf('Generating Figure 7: INS 3D Trajectory...\n');

figure(7);
clf;

plot3( ...
    result.pos(:,1), ...
    result.pos(:,2), ...
    result.pos(:,3), ...
    'LineWidth', 1.5);

grid on;

axis equal;

xlabel('East [m]');

ylabel('North [m]');

zlabel('Up [m]');

title('Pure INS 3D Boat Trajectory');

view(3);

SaveFigure( ...
    figure(7), ...
    'Figure7_INS3DTrajectory.png', ...
    cfg);


%% ========================================================================
% 17. FIGURE 8
% INS VS GPS
% ========================================================================

fprintf('Generating Figure 8: INS vs GPS...\n');

figure(8);
clf;

plot( ...
    result.pos(:,1), ...
    result.pos(:,2), ...
    'LineWidth', 1.5);

hold on;


if ~isempty(gpsENU)

    plot( ...
        gpsENU.pos(:,1), ...
        gpsENU.pos(:,2), ...
        'LineWidth', 1.5);

    legend('Pure INS','GPS');

else

    legend('Pure INS');

end


grid on;

axis equal;

xlabel('East [m]');

ylabel('North [m]');

title('Boat Trajectory: Pure INS vs GPS');

SaveFigure( ...
    figure(8), ...
    'Figure8_INS_vs_GPS.png', ...
    cfg);


%% ========================================================================
% 18. FINISH
% ========================================================================

fprintf('\n');
fprintf('====================================================\n');

fprintf('ALL PROCESSING COMPLETED SUCCESSFULLY.\n');

fprintf('8 FIGURES GENERATED.\n');

fprintf('Results Folder:\n');

fprintf('%s\n', cfg.outputFolder);

fprintf('====================================================\n');


%% ========================================================================
% FUNCTION
% CONFIGURATION
% ========================================================================

function cfg = ConfigBoatINS()

cfg.imuFile = ...
    'E:\长江测试\长江测试文件\长江测试文件\第二次\SKT1_100HZ_第二次.log';

cfg.gpsFile = ...
    'E:\长江测试\代码分析\LC29H_Test2_Position_Result.csv';

cfg.outputFolder = ...
    'Boat_INS_Result_V3';


if ~exist(cfg.outputFolder,'dir')

    mkdir(cfg.outputFolder);

end


%% IMU

cfg.fs = 100;

cfg.dt = 1 / cfg.fs;

cfg.g0 = 9.80665;


%% ================================================================
% IMU FRAME → BODY FRAME
%
% Body FLU
%
% Xi = Left
% Yi = Backward
% Zi = Up
%
% Xb = Forward = -Yi
% Yb = Left    =  Xi
% Zb = Up      =  Zi
% ================================================================

cfg.C_b_i = [ ...
     0  -1   0;
     1   0   0;
     0   0   1];


%% GYRO BIAS

cfg.gyroBiasUser_i = ...
    deg2rad([ ...
    -0.0300;
     0.0400;
     0.0000]);

cfg.gyroBiasUser_b = ...
    cfg.C_b_i * cfg.gyroBiasUser_i;


%% STATIC DETECTION

cfg.staticSearchTime = 60;

cfg.staticMinDuration = 10;

cfg.accStaticThreshold = 0.05 * cfg.g0;

cfg.gyroStaticThreshold = ...
    deg2rad(0.8);


%% BIAS

cfg.gyroBiasMeasuredWeight = 0.8;


%% ATTITUDE

cfg.enableAccCorrection = true;

cfg.KpAcc = 0.08;

cfg.KiBias = 0.00001;

cfg.accNormTolerance = ...
    0.12 * cfg.g0;


%% MAGNETOMETER

% 船载环境默认关闭全程磁航向修正

cfg.enableMagYawCorrection = false;

cfg.KpMagYaw = 0.02;

cfg.magNormTolerance = 0.10;


%% ================================================================
% GPS INITIALIZATION
%
% true:
% GPS仅用于初始航向和速度
%
% 后续INS运行过程中不使用GPS更新
% ================================================================

cfg.useGPSInitialYaw = true;

cfg.useGPSInitialVelocity = false;


%% SHIP CONSTRAINT

cfg.enableNHC = true;

cfg.nhcLateralGain = 0.30;

cfg.nhcVerticalGain = 0.70;

cfg.maxVerticalVelocity = 0.5;


%% ZUPT

cfg.enableZUPT = true;

cfg.zuptAccThreshold = ...
    0.04 * cfg.g0;

cfg.zuptGyroThreshold = ...
    deg2rad(0.5);


%% SPEED LIMIT

cfg.maxBoatSpeed = 20;


%% PROGRESS

cfg.progressInterval = 10;

end


%% ========================================================================
% LOAD IMU
% ========================================================================

function imu = LoadBoatIMU(cfg)

fprintf('\nReading IMU file...\n');


fid = fopen(cfg.imuFile,'r');

if fid == -1

    error( ...
        'Cannot open IMU file:\n%s', ...
        cfg.imuFile);

end


data = [];


while ~feof(fid)

    line = fgetl(fid);

    if ~ischar(line)

        continue;

    end


    values = sscanf( ...
        line, ...
        '%f,%f,%f,%f,%f,%f,%f,%f,%f,%f');


    if numel(values) == 10

        data(end+1,:) = values';

    end

end


fclose(fid);


if isempty(data)

    error('No valid IMU data.');

end


valid = all(isfinite(data),2);

data = data(valid,:);


%% TIME

index = data(:,1);

t = ...
    (index - index(1)) ...
    / cfg.fs;


%% ACC

acc = ...
    data(:,2:4) ...
    * cfg.g0;


%% GYRO

gyro = ...
    deg2rad(data(:,5:7));


%% MAG

mag = data(:,8:10);


%% OUTPUT

imu.index = index;

imu.t = t;

imu.acc = acc;

imu.gyro = gyro;

imu.mag = mag;

imu.fs = cfg.fs;

imu.N = length(t);

end


%% ========================================================================
% LOAD GPS
% ========================================================================

function gps = LoadBoatGPS(cfg)

gps = struct();

gps.lat = [];

gps.lon = [];

gps.h = [];

gps.t = [];

gps.N = 0;


if ~exist(cfg.gpsFile,'file')

    fprintf('\nGPS file not found:\n%s\n', ...
        cfg.gpsFile);

    return;

end


fprintf('\nReading GPS file...\n');


try

    T = readtable(cfg.gpsFile);

catch ME

    warning( ...
        'Cannot read GPS file: %s', ...
        ME.message);

    return;

end


names = lower( ...
    string(T.Properties.VariableNames));


%% LATITUDE

latID = find( ...
    contains(names,'lat') | ...
    contains(names,'latitude'), ...
    1);


%% LONGITUDE

lonID = find( ...
    contains(names,'lon') | ...
    contains(names,'longitude'), ...
    1);


%% HEIGHT

hID = find( ...
    contains(names,'height') | ...
    contains(names,'alt') | ...
    contains(names,'altitude'), ...
    1);


%% TIME

timeID = find( ...
    contains(names,'time') | ...
    contains(names,'timestamp'), ...
    1);


if isempty(latID) || isempty(lonID)

    warning( ...
        'GPS latitude/longitude columns cannot be identified.');

    return;

end


gps.lat = T{:,latID};

gps.lon = T{:,lonID};


%% Height

if ~isempty(hID)

    gps.h = T{:,hID};

else

    gps.h = zeros( ...
        length(gps.lat),1);

end


%% Time

if ~isempty(timeID)

    tempTime = T{:,timeID};

    if isnumeric(tempTime)

        gps.t = tempTime;

    else

        gps.t = ...
            (0:length(gps.lat)-1)';

    end

else

    gps.t = ...
        (0:length(gps.lat)-1)';

end


%% Remove invalid

valid = ...
    isfinite(gps.lat) & ...
    isfinite(gps.lon);


gps.lat = gps.lat(valid);

gps.lon = gps.lon(valid);

gps.h = gps.h(valid);

gps.t = gps.t(valid);


%% Convert NMEA DDMM.MMMM if necessary

if median(abs(gps.lat)) > 90

    gps.lat = ...
        NMEAToDegree(gps.lat);

end


if median(abs(gps.lon)) > 180

    gps.lon = ...
        NMEAToDegree(gps.lon);

end


%% Normalize time

gps.t = ...
    gps.t - gps.t(1);


gps.N = ...
    length(gps.lat);


fprintf('GPS samples: %d\n', ...
    gps.N);

end


%% ========================================================================
% NMEA DEGREE CONVERSION
% ========================================================================

function degree = NMEAToDegree(x)

degreePart = floor(x / 100);

minutePart = ...
    x - degreePart * 100;

degree = ...
    degreePart ...
    + minutePart / 60;

end


%% ========================================================================
% GPS TO ENU
% ========================================================================

function gpsENU = GPS2ENU(gps)

lat0 = gps.lat(1);

lon0 = gps.lon(1);

h0 = gps.h(1);


a = 6378137.0;

e2 = ...
    6.69437999014e-3;


lat0Rad = ...
    deg2rad(lat0);


sinLat = ...
    sin(lat0Rad);


Rn = ...
    a / sqrt( ...
    1 - e2 * sinLat^2);


Rm = ...
    a * (1-e2) / ...
    (1 - e2*sinLat^2)^(3/2);


dLat = ...
    deg2rad( ...
    gps.lat - lat0);


dLon = ...
    deg2rad( ...
    gps.lon - lon0);


North = ...
    dLat * Rm;


East = ...
    dLon ...
    .* Rn ...
    .* cos(lat0Rad);


Up = ...
    gps.h - h0;


gpsENU.t = ...
    gps.t(:);


gpsENU.pos = [ ...
    East ...
    North ...
    Up];


gpsENU.N = ...
    length(gps.t);

end


%% ========================================================================
% IMU TO BODY
% ========================================================================

function [acc_b,gyro_b,mag_b] = ...
    TransformIMUToBody( ...
    acc_i, ...
    gyro_i, ...
    mag_i, ...
    cfg)

N = size(acc_i,1);


acc_b = ...
    (cfg.C_b_i * acc_i')';


gyro_b = ...
    (cfg.C_b_i * gyro_i')';


mag_b = ...
    (cfg.C_b_i * mag_i')';


if size(acc_b,1) ~= N

    error('IMU transformation failed.');

end

end


%% ========================================================================
% STATIC DETECTION
% ========================================================================

function info = DetectStaticSegment( ...
    acc, ...
    gyro, ...
    fs, ...
    cfg)

N = size(acc,1);


searchN = min( ...
    round(cfg.staticSearchTime * fs), ...
    N);


accNorm = ...
    vecnorm( ...
    acc(1:searchN,:), ...
    2,2);


gyroNorm = ...
    vecnorm( ...
    gyro(1:searchN,:), ...
    2,2);


accError = ...
    abs(accNorm - cfg.g0);


staticFlag = ...
    accError < cfg.accStaticThreshold ...
    & gyroNorm < cfg.gyroStaticThreshold;


minSamples = ...
    round(cfg.staticMinDuration * fs);


bestStart = 1;

bestEnd = ...
    min(N,minSamples);


currentStart = 0;


for k = 1:searchN

    if staticFlag(k)

        if currentStart == 0

            currentStart = k;

        end

    else

        if currentStart > 0

            duration = ...
                k - currentStart;


            if duration >= minSamples

                bestStart = ...
                    currentStart;

                bestEnd = ...
                    k - 1;

                break;

            end


            currentStart = 0;

        end

    end

end


%% If static segment continues to end

if currentStart > 0 && ...
        bestEnd == min(N,minSamples)

    duration = ...
        searchN - currentStart + 1;


    if duration >= minSamples

        bestStart = ...
            currentStart;

        bestEnd = ...
            searchN;

    end

end


info.startIndex = bestStart;

info.endIndex = bestEnd;

info.startTime = ...
    (bestStart-1) / fs;

info.endTime = ...
    (bestEnd-1) / fs;

end


%% ========================================================================
% INITIAL ALIGNMENT
% ========================================================================

function init = InitialAlignmentBoat( ...
    acc_b, ...
    gyro_b, ...
    mag_b, ...
    staticInfo, ...
    gpsENU, ...
    cfg)


idx = ...
    staticInfo.startIndex : ...
    staticInfo.endIndex;


%% Mean values

accMean = ...
    mean(acc_b(idx,:),1)';


gyroMean = ...
    mean(gyro_b(idx,:),1)';


magMean = ...
    mean(mag_b(idx,:),1)';


%% Gyro bias

alpha = ...
    cfg.gyroBiasMeasuredWeight;


gyroBias = ...
    alpha * gyroMean ...
    + (1-alpha) ...
    * cfg.gyroBiasUser_b;


%% ================================================================
% ROLL / PITCH
%
% Body FLU
% Navigation ENU
%
% Static specific force:
%
% f = [0 0 +g]
% ================================================================

f = ...
    accMean ...
    / norm(accMean);


fx = f(1);

fy = f(2);

fz = f(3);


roll = ...
    atan2(fy,fz);


pitch = ...
    atan2( ...
    -fx, ...
    sqrt(fy^2 + fz^2));


%% ================================================================
% INITIAL YAW
% ================================================================

yaw = 0;


%% Preferred:
% GPS initial course

if cfg.useGPSInitialYaw && ...
        ~isempty(gpsENU)

    yaw = ...
        EstimateGPSInitialYaw( ...
        gpsENU);

    fprintf('\nInitial yaw obtained from GPS course.\n');


else

    %% Magnetometer yaw only used for initialization

    mx = magMean(1);

    my = magMean(2);

    mz = magMean(3);


    mxh = ...
        mx*cos(pitch) ...
        + my*sin(roll)*sin(pitch) ...
        + mz*cos(roll)*sin(pitch);


    myh = ...
        my*cos(roll) ...
        - mz*sin(roll);


    yaw = ...
        atan2(myh,mxh);

    fprintf('\nInitial yaw obtained from magnetometer.\n');

end


%% Quaternion

q0 = ...
    EulerToQuaternion( ...
    roll, ...
    pitch, ...
    yaw);


%% Initial velocity

v0 = ...
    [0;0;0];


if cfg.useGPSInitialVelocity && ...
        ~isempty(gpsENU)

    v0 = ...
        EstimateGPSInitialVelocity( ...
        gpsENU);

end


%% Output

init.accMean = accMean;

init.gyroBias = gyroBias;

init.magMean = magMean;

init.roll = roll;

init.pitch = pitch;

init.yaw = yaw;

init.q0 = q0;

init.v0 = v0;


fprintf('\n');
fprintf('====================================================\n');

fprintf('Initial Alignment Result\n');

fprintf('====================================================\n');

fprintf('Mean Acceleration [m/s^2]\n');

fprintf('X = %.6f\n', ...
    accMean(1));

fprintf('Y = %.6f\n', ...
    accMean(2));

fprintf('Z = %.6f\n', ...
    accMean(3));


fprintf('\nInitial Euler [deg]\n');

fprintf('Roll  = %.3f\n', ...
    rad2deg(roll));

fprintf('Pitch = %.3f\n', ...
    rad2deg(pitch));

fprintf('Yaw   = %.3f\n', ...
    rad2deg(yaw));


fprintf('\nGyro Bias [deg/s]\n');

disp(rad2deg(gyroBias)');


fprintf('\nInitial Velocity [m/s]\n');

disp(v0');

end


%% ========================================================================
% ESTIMATE GPS INITIAL YAW
% ========================================================================

function yaw = EstimateGPSInitialYaw(gpsENU)

pos = ...
    gpsENU.pos;


yaw = 0;


for k = 2:size(pos,1)

    dE = ...
        pos(k,1) ...
        - pos(1,1);


    dN = ...
        pos(k,2) ...
        - pos(1,2);


    distance = ...
        hypot(dE,dN);


    if distance > 10

        yaw = ...
            atan2(dN,dE);

        return;

    end

end

end


%% ========================================================================
% GPS INITIAL VELOCITY
% ========================================================================

function v0 = EstimateGPSInitialVelocity(gpsENU)

v0 = [0;0;0];


if length(gpsENU.t) < 2

    return;

end


dt = ...
    gpsENU.t(2) ...
    - gpsENU.t(1);


if dt <= 0

    return;

end


v0 = ...
    (gpsENU.pos(2,:) ...
    - gpsENU.pos(1,:))' ...
    / dt;

end


%% ========================================================================
% BOAT INS MECHANIZATION
% ========================================================================

function result = BoatINSMechanization( ...
    t, ...
    acc, ...
    gyro, ...
    mag, ...
    init, ...
    cfg)


N = length(t);


%% MEMORY

q = zeros(N,4);

euler = zeros(N,3);

vel = zeros(N,3);

pos = zeros(N,3);

gyroBias = zeros(N,3);

accWeightHistory = zeros(N,1);

magWeightHistory = zeros(N,1);

isStaticHistory = false(N,1);


%% INITIAL STATE

q(1,:) = ...
    init.q0';


vel(1,:) = ...
    init.v0';


pos(1,:) = ...
    [0 0 0];


gyroBias(1,:) = ...
    init.gyroBias';


euler(1,:) = ...
    rad2deg([ ...
    init.roll ...
    init.pitch ...
    init.yaw]);


%% MAGNETIC REFERENCE

magRef = ...
    norm(init.magMean);


%% PROGRESS

progressStep = ...
    max(1, ...
    round(N / cfg.progressInterval));


%% MAIN LOOP

for k = 2:N


    %% Progress

    if mod(k,progressStep) == 0

        fprintf( ...
            'INS Progress: %3.0f %%\n', ...
            100*k/N);

    end


    %% TIME STEP

    dt = ...
        t(k) ...
        - t(k-1);


    if dt <= 0 || dt > 0.05

        dt = cfg.dt;

    end


    %% PREVIOUS STATE

    qPrev = ...
        q(k-1,:)';


    Cnb = ...
        QuaternionToDCM(qPrev);


    %% GYRO

    omega = ...
        gyro(k,:)' ...
        - gyroBias(k-1,:)';


    %% STATIC DETECTION

    accNorm = ...
        norm(acc(k,:));


    gyroNorm = ...
        norm(omega);


    isStatic = ...
        abs(accNorm - cfg.g0) ...
        < cfg.zuptAccThreshold ...
        && gyroNorm ...
        < cfg.zuptGyroThreshold;


    isStaticHistory(k) = ...
        isStatic;


    %% ================================================================
    % ATTITUDE CORRECTION
    % ================================================================

    correction = ...
        [0;0;0];


    %% ACCELEROMETER

    accWeight = 0;


    if cfg.enableAccCorrection


        f_b = ...
            acc(k,:)';


        fNorm = ...
            norm(f_b);


        deviation = ...
            abs(fNorm - cfg.g0);


        if deviation < cfg.accNormTolerance


            accWeight = ...
                1 ...
                - deviation ...
                / cfg.accNormTolerance;


            fUnit = ...
                f_b ...
                / max(fNorm,1e-10);


            %% Expected gravity specific force
            %
            % Navigation ENU
            %
            % static f_n = [0 0 +1]

            fExpected_b = ...
                Cnb' ...
                * [0;0;1];


            fExpected_b = ...
                fExpected_b ...
                / norm(fExpected_b);


            %% ====================================================
            % IMPORTANT
            %
            % Error direction:
            %
            % predicted × measured
            %
            % Previous code used the reverse direction.
            % ====================================================

            eAcc = ...
                cross( ...
                fExpected_b, ...
                fUnit);


            correction = ...
                correction ...
                + cfg.KpAcc ...
                * accWeight ...
                * eAcc;


        end

    end


    accWeightHistory(k) = ...
        accWeight;


    %% ================================================================
    % MAGNETOMETER
    %
    % Disabled by default for metal boat
    % ================================================================

    magWeight = 0;


    if cfg.enableMagYawCorrection


        m_b = ...
            mag(k,:)';


        mNorm = ...
            norm(m_b);


        normError = ...
            abs(mNorm-magRef) ...
            / max(magRef,1e-10);


        if normError < cfg.magNormTolerance


            magWeight = ...
                1 ...
                - normError ...
                / cfg.magNormTolerance;


            eulerCurrent = ...
                DCMToEulerENU( ...
                Cnb);


            roll = ...
                eulerCurrent(1);


            pitch = ...
                eulerCurrent(2);


            yawCurrent = ...
                eulerCurrent(3);


            mx = m_b(1);

            my = m_b(2);

            mz = m_b(3);


            mxh = ...
                mx*cos(pitch) ...
                + my*sin(roll)*sin(pitch) ...
                + mz*cos(roll)*sin(pitch);


            myh = ...
                my*cos(roll) ...
                - mz*sin(roll);


            yawMag = ...
                atan2(myh,mxh);


            yawError = ...
                WrapAngle( ...
                yawMag-yawCurrent);


            yawCorrection = ...
                cfg.KpMagYaw ...
                * magWeight ...
                * yawError;


            correction(3) = ...
                correction(3) ...
                + yawCorrection;

        end

    end


    magWeightHistory(k) = ...
        magWeight;


    %% ================================================================
    % GYRO BIAS UPDATE
    % ================================================================

    gyroBiasNew = ...
        gyroBias(k-1,:)' ...
        - cfg.KiBias ...
        * correction ...
        * dt;


    gyroBias(k,:) = ...
        gyroBiasNew';


    %% ================================================================
    % QUATERNION PROPAGATION
    % ================================================================

    omegaCorrected = ...
        omega ...
        + correction;


    omegaQuat = ...
        [0;omegaCorrected];


    qDot = ...
        0.5 ...
        * QuaternionMultiply( ...
        qPrev, ...
        omegaQuat);


    qNew = ...
        qPrev ...
        + qDot*dt;


    qNew = ...
        QuaternionNormalize(qNew);


    q(k,:) = ...
        qNew';


    %% ================================================================
    % ATTITUDE
    % ================================================================

    Cnb = ...
        QuaternionToDCM(qNew);


    eulerRad = ...
        DCMToEulerENU( ...
        Cnb);


    euler(k,:) = ...
        rad2deg(eulerRad)';


    %% ================================================================
    % SPECIFIC FORCE
    % ================================================================

    fNav = ...
        Cnb ...
        * acc(k,:)';


    %% ENU gravity

    gNav = ...
        [0;0;-cfg.g0];


    %% Navigation acceleration

    accNav = ...
        fNav ...
        + gNav;


    %% ================================================================
    % VELOCITY PROPAGATION
    % ================================================================

    velPred = ...
        vel(k-1,:)' ...
        + accNav*dt;


    %% ZUPT

    if cfg.enableZUPT && isStatic

        velPred = ...
            [0;0;0];

    end


    %% ================================================================
    % BOAT NON-HOLONOMIC CONSTRAINT
    %
    % Body FLU:
    %
    % X = Forward
    % Y = Left
    % Z = Up
    % ================================================================

    if cfg.enableNHC


        velBody = ...
            Cnb' ...
            * velPred;


        %% Lateral velocity

        velBody(2) = ...
            velBody(2) ...
            * (1-cfg.nhcLateralGain);


        %% Vertical velocity

        velBody(3) = ...
            velBody(3) ...
            * (1-cfg.nhcVerticalGain);


        %% Limit vertical

        if abs(velBody(3)) ...
                > cfg.maxVerticalVelocity

            velBody(3) = ...
                sign(velBody(3)) ...
                * cfg.maxVerticalVelocity;

        end


        %% Back to navigation

        velPred = ...
            Cnb ...
            * velBody;

    end


    %% SPEED PROTECTION

    speed = ...
        norm(velPred);


    if speed > cfg.maxBoatSpeed

        velPred = ...
            velPred ...
            / speed ...
            * cfg.maxBoatSpeed;

    end


    vel(k,:) = ...
        velPred';


    %% ================================================================
    % POSITION
    % ================================================================

    pos(k,:) = ...
        pos(k-1,:) ...
        + 0.5 ...
        * (vel(k,:) ...
        + vel(k-1,:)) ...
        * dt;


end


%% ================================================================
% UNWRAP YAW
% ================================================================

yaw = ...
    deg2rad( ...
    euler(:,3));


yaw = ...
    unwrap(yaw);


euler(:,3) = ...
    rad2deg(yaw);


%% OUTPUT

result = struct();


result.t = t;

result.q = q;

result.euler = euler;

result.vel = vel;

result.pos = pos;

result.gyroBias = gyroBias;

result.accWeight = accWeightHistory;

result.magWeight = magWeightHistory;

result.isStatic = isStaticHistory;

end


%% ========================================================================
% SYNCHRONIZE INS AND GPS
% ========================================================================

function [insSync,gpsSync] = ...
    SynchronizeINSandGPS( ...
    result, ...
    gpsENU)


insSync = [];

gpsSync = [];


if isempty(gpsENU)

    return;

end


gpsTime = ...
    gpsENU.t(:);


%% If GPS time is invalid,
% use normalized relative time

if any(diff(gpsTime) <= 0)

    gpsTime = ...
        (0:length(gpsTime)-1)';

end


%% Keep overlapping data

valid = ...
    gpsTime >= result.t(1) ...
    & gpsTime <= result.t(end);


gpsTime = ...
    gpsTime(valid);


gpsPos = ...
    gpsENU.pos(valid,:);


if length(gpsTime) < 2

    return;

end


insPos = ...
    interp1( ...
    result.t, ...
    result.pos, ...
    gpsTime, ...
    'linear');


insVel = ...
    interp1( ...
    result.t, ...
    result.vel, ...
    gpsTime, ...
    'linear');


insEuler = ...
    interp1( ...
    result.t, ...
    result.euler, ...
    gpsTime, ...
    'linear');


insSync.t = ...
    gpsTime;

insSync.pos = ...
    insPos;

insSync.vel = ...
    insVel;

insSync.euler = ...
    insEuler;


gpsSync.t = ...
    gpsTime;

gpsSync.pos = ...
    gpsPos;

end


%% ========================================================================
% EVALUATE
% ========================================================================

function evalResult = ...
    EvaluateBoatINS( ...
    ins, ...
    gps)


errorPos = ...
    ins.pos ...
    - gps.pos;


horizontalError = ...
    hypot( ...
    errorPos(:,1), ...
    errorPos(:,2));


error3D = ...
    sqrt( ...
    sum(errorPos.^2,2));


evalResult.horizontalError = ...
    horizontalError;


evalResult.positionError3D = ...
    error3D;


evalResult.horizontalRMSE = ...
    sqrt(mean(horizontalError.^2));


evalResult.posRMSE = ...
    sqrt(mean(error3D.^2));


evalResult.finalHorizontalError = ...
    horizontalError(end);


evalResult.finalError = ...
    error3D(end);

end


%% ========================================================================
% EXPORT RESULTS
% ========================================================================

function ExportBoatINSResults( ...
    imu, ...
    result, ...
    gpsENU, ...
    evalResult, ...
    cfg)


folder = ...
    cfg.outputFolder;


if ~exist(folder,'dir')

    mkdir(folder);

end


%% RAW IMU

rawTable = table( ...
    imu.t, ...
    imu.acc(:,1), ...
    imu.acc(:,2), ...
    imu.acc(:,3), ...
    rad2deg(imu.gyro(:,1)), ...
    rad2deg(imu.gyro(:,2)), ...
    rad2deg(imu.gyro(:,3)), ...
    imu.mag(:,1), ...
    imu.mag(:,2), ...
    imu.mag(:,3));


rawTable.Properties.VariableNames = { ...
    'Time_s', ...
    'AccX_mps2', ...
    'AccY_mps2', ...
    'AccZ_mps2', ...
    'GyroX_degps', ...
    'GyroY_degps', ...
    'GyroZ_degps', ...
    'MagX_uT', ...
    'MagY_uT', ...
    'MagZ_uT'};


writetable( ...
    rawTable, ...
    fullfile( ...
    folder, ...
    'Raw_IMU_Data.csv'));


%% ATTITUDE

attTable = table( ...
    result.t, ...
    result.euler(:,1), ...
    result.euler(:,2), ...
    result.euler(:,3));


attTable.Properties.VariableNames = { ...
    'Time_s', ...
    'Roll_deg', ...
    'Pitch_deg', ...
    'Yaw_deg'};


writetable( ...
    attTable, ...
    fullfile( ...
    folder, ...
    'INS_Attitude.csv'));


%% VELOCITY

velTable = table( ...
    result.t, ...
    result.vel(:,1), ...
    result.vel(:,2), ...
    result.vel(:,3));


velTable.Properties.VariableNames = { ...
    'Time_s', ...
    'East_mps', ...
    'North_mps', ...
    'Up_mps'};


writetable( ...
    velTable, ...
    fullfile( ...
    folder, ...
    'INS_Velocity.csv'));


%% POSITION

posTable = table( ...
    result.t, ...
    result.pos(:,1), ...
    result.pos(:,2), ...
    result.pos(:,3));


posTable.Properties.VariableNames = { ...
    'Time_s', ...
    'East_m', ...
    'North_m', ...
    'Up_m'};


writetable( ...
    posTable, ...
    fullfile( ...
    folder, ...
    'INS_Position.csv'));


%% GPS

if ~isempty(gpsENU)

    gpsTable = table( ...
        gpsENU.t, ...
        gpsENU.pos(:,1), ...
        gpsENU.pos(:,2), ...
        gpsENU.pos(:,3));


    gpsTable.Properties.VariableNames = { ...
        'Time_s', ...
        'East_m', ...
        'North_m', ...
        'Up_m'};


    writetable( ...
        gpsTable, ...
        fullfile( ...
        folder, ...
        'GPS_ENU.csv'));

end


%% MAT

save( ...
    fullfile( ...
    folder, ...
    'Boat_INS_Result.mat'), ...
    'result', ...
    'imu', ...
    'gpsENU', ...
    'evalResult');


%% Evaluation

if ~isempty(evalResult)

    fid = fopen( ...
        fullfile( ...
        folder, ...
        'Evaluation.txt'), ...
        'w');


    fprintf( ...
        fid, ...
        'Horizontal RMSE = %.6f m\n', ...
        evalResult.horizontalRMSE);


    fprintf( ...
        fid, ...
        '3D Position RMSE = %.6f m\n', ...
        evalResult.posRMSE);


    fprintf( ...
        fid, ...
        'Final Horizontal Error = %.6f m\n', ...
        evalResult.finalHorizontalError);


    fprintf( ...
        fid, ...
        'Final 3D Error = %.6f m\n', ...
        evalResult.finalError);


    fclose(fid);

end


fprintf('\nResults exported to:\n%s\n', ...
    folder);

end


%% ========================================================================
% SAVE FIGURE
% ========================================================================

function SaveFigure( ...
    figHandle, ...
    fileName, ...
    cfg)


if ~ishandle(figHandle)

    return;

end


try

    saveas( ...
        figHandle, ...
        fullfile( ...
        cfg.outputFolder, ...
        fileName));

catch

end

end


%% ========================================================================
% QUATERNION MULTIPLY
% ========================================================================

function q = ...
    QuaternionMultiply(q1,q2)


w1 = q1(1);

x1 = q1(2);

y1 = q1(3);

z1 = q1(4);


w2 = q2(1);

x2 = q2(2);

y2 = q2(3);

z2 = q2(4);


q = [ ...
    w1*w2 - x1*x2 - y1*y2 - z1*z2;
    w1*x2 + x1*w2 + y1*z2 - z1*y2;
    w1*y2 - x1*z2 + y1*w2 + z1*x2;
    w1*z2 + x1*y2 - y1*x2 + z1*w2];

end


%% ========================================================================
% QUATERNION NORMALIZE
% ========================================================================

function q = ...
    QuaternionNormalize(q)


q = ...
    q / max(norm(q),1e-12);

end


%% ========================================================================
% QUATERNION TO DCM
% ========================================================================

function C = ...
    QuaternionToDCM(q)


q = ...
    QuaternionNormalize(q);


q0 = q(1);

q1 = q(2);

q2 = q(3);

q3 = q(4);


C = [ ...

1 - 2*(q2^2+q3^2), ...
2*(q1*q2-q0*q3), ...
2*(q1*q3+q0*q2);

2*(q1*q2+q0*q3), ...
1 - 2*(q1^2+q3^2), ...
2*(q2*q3-q0*q1);

2*(q1*q3-q0*q2), ...
2*(q2*q3+q0*q1), ...
1 - 2*(q1^2+q2^2)];

end


%% ========================================================================
% EULER TO QUATERNION
% ========================================================================

function q = ...
    EulerToQuaternion( ...
    roll, ...
    pitch, ...
    yaw)


cr = cos(roll/2);

sr = sin(roll/2);


cp = cos(pitch/2);

sp = sin(pitch/2);


cy = cos(yaw/2);

sy = sin(yaw/2);


q0 = ...
    cr*cp*cy ...
    + sr*sp*sy;


q1 = ...
    sr*cp*cy ...
    - cr*sp*sy;


q2 = ...
    cr*sp*cy ...
    + sr*cp*sy;


q3 = ...
    cr*cp*sy ...
    - sr*sp*cy;


q = ...
    [q0;q1;q2;q3];


q = ...
    QuaternionNormalize(q);

end


%% ========================================================================
% DCM TO EULER
% ========================================================================

function euler = ...
    DCMToEulerENU(C)


value = ...
    -C(3,1);


value = ...
    max( ...
    -1, ...
    min(1,value));


pitch = ...
    asin(value);


if abs(cos(pitch)) > 1e-6

    roll = ...
        atan2( ...
        C(3,2), ...
        C(3,3));


    yaw = ...
        atan2( ...
        C(2,1), ...
        C(1,1));

else

    roll = 0;

    yaw = ...
        atan2( ...
        -C(1,2), ...
        C(2,2));

end


euler = ...
    [roll;pitch;yaw];

end


%% ========================================================================
% ANGLE WRAP
% ========================================================================

function angle = ...
    WrapAngle(angle)


angle = ...
    atan2( ...
    sin(angle), ...
    cos(angle));

end