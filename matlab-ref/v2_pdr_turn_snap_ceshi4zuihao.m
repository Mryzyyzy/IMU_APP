%% ========================================================================
%  室内定位 - 步数+航向状态机 (PDR: Step Detection + Turn-Snap Heading)
%  场景  : 手持IMU平放于胸前, 全程连续行走(拐角处无停顿) —— 这一点已通过
%          实测数据确认(GLRT诊断显示除结束外几乎无真正静止), 因此放弃基于
%          "加速度二次积分+ZUPT"的纯惯导方案(该方案要求周期性真零速,
%          连续行走场景下天然不满足，会导致速度/位置无约束发散)。
%
%  新方案核心思想:
%   1) 位置: 不再对加速度做二次积分, 改为"检测走路时身体的上下起伏峰值"
%      数出步数, 步长用"已知总路程/总步数"反标定(因为本实验路线总长已知,
%      且首尾闭合)。二次积分误差是平方增长的，而"数步子+固定步长"的误差
%      只随步数线性增长，稳健得多。
%   2) 航向: 不再连续积分陀螺仪角度(长时间必然漂移到不可用)。而是利用
%      "这条路只有直角转弯"这一先验知识: 只用陀螺仪去检测"当前是否在转弯"，
%      转弯时段内积分角度后吸附(snap)到最近的90°整数倍；直线行走时段内
%      航向直接锁定为常数、不再连续累积漂移。
%   3) 因此完全不需要姿态四元数解算、重力补偿、速度积分——整体逻辑比之前
%      的捷联惯导方案simpler也更适配"手持连续走"这种场景。
%
%  传感器: SCH16T-K01 (加速度计+陀螺仪), 采样率100Hz
%  数据列: 1 索引 | 2:4 加速度(g) | 5:7 角速度(deg/s) | 8:10 磁场(uT,未使用)
%  IMU坐标系(FLU右手系, 按芯片丝印): Xb-前, Yb-左, Zb-上(=Xb×Yb)
%  测试路线(俯视, 起点=终点, 共2圈):
%       南8.8m -> 西8m -> 北8.8m -> 东8m  (第1圈)
%       南8.8m -> 西8m -> 北8.8m -> 东8m  (第2圈, 回到起点)
% ========================================================================

clear; clc; close all;

%% ---------------------- 0. 参数设置 ------------------------------------
fs = 100; dt = 1/fs;
g0 = 9.80665;
%  filename = 'E:\急项目申报书\测试数据16488\新板子\新师弟走两圈3.txt';    % <<< 修改为实际数据文件名/路径
% filename = 'E:\急项目申报书\测试数据16488\新板子\我走2圈.txt';    % <<< 修改为实际数据文件名/路径
filename = 'E:\急项目申报书\测试数据16488\新板子\新我自己走两圈4.txt';    % <<< 修改为实际数据文件名/路径
% filename = 'E:\急项目申报书\测试数据16488\新板子\722_室内刘走4圈.txt';    % <<< 修改为实际数据文件名/路径
% filename = 'E:\急项目申报书\测试数据16488\新板子\729室内轨迹+姿态变化\SKT_2.txt';
 % filename = 'E:\急项目申报书\测试数据16488\新板子\新师弟走两圈3.txt';    % <<< 修改为实际数据文件名/路径

% ---- 已知路线几何(用于步长反标定 & 生成参考路线) ----
seg_len_m   = [8.8, 8, 8.8, 8];      % 南, 西, 北, 东
seg_dir_deg = [180, 270, 0, 90];     % 对应方位角(0=北,90=东,180=南,270=西)
n_loops     = 2;
total_known_dist = sum(seg_len_m)*n_loops;   % 两圈总路程

yaw0_deg  = 180;      % 初始航向(人工设定, 例: 已知起始朝南出发)

% <<< 修改点 >>>
% 之前 turn_sign = 1 时, 航向状态机输出 180 -> 90 -> 0 -> -90 ...
% 也就是 南 -> 东 -> 北 -> 西, 与实际路线 南 -> 西 -> 北 -> 东 左右镜像。
% 说明陀螺Z轴的转弯角积分方向, 与"方位角=atan2(E,N)风格(顺时针为正)"的
% 定义正好相反, 需要把turn_sign取反来修正、而不是重新接线/换轴。
turn_sign = -1;    % <<< 由 1 改为 -1, 修正"南接下来变东"应为"南接下来变西"的镜像问题

% ---- 步态峰值检测参数(在低通平滑后的加速度模上找峰) ----
smooth_win_acc = 5;    % movmean平滑窗口(采样点), 去掉高频噪声但保留步态起伏
step_min_prom  = 0.5;  % m/s^2, 峰值需比局部谷值高出多少才算一步(核心调参项)
step_min_dist  = 30;   % 采样点, 相邻两步最小间隔(~0.3s, 对应最快步频)

% ---- 转弯检测参数 ----
smooth_win_gz  = 8;         % movmean平滑窗口(采样点), 平滑gz用于分段
turn_rate_th   = 15;        % deg/s, 判定"正在转弯"的角速度阈值
turn_min_dur   = 0.5;       % s, 转弯片段最短持续时间(短于此忽略,视为手抖噪声)
turn_min_angle = 45;        % deg, 转弯片段积分角度最小值(短于此忽略)

fprintf('=== 参数设置完毕，开始处理数据 ===\n');

%% ---------------------- 1. 读取数据 ------------------------------------
raw = readmatrix(filename);
acc_g    = raw(:,2:4);
gyro_dps = raw(:,5:7);

N = size(raw,1);
t = (0:N-1)'*dt;
acc  = acc_g*g0;              % m/s^2
acc_norm = vecnorm(acc,2,2);  % 比力模值

gz = gyro_dps(:,3);           % Z轴(上, FLU) = 航向轴, 见坐标系说明

fprintf('数据总时长: %.1f s, 总采样点: %d\n', N*dt, N);

%% ---------------------- 2. 自动寻找"真静止"窗口, 标定陀螺Z零偏 --------
% 连续行走场景下，开头/结尾不一定真静止(需实测验证)，这里自动扫描全程,
% 找角速度标准差最小的一段(默认3秒窗口)作为零偏标定窗口，而不是想当然
% 认为开头必然静止。
W_bias = round(3*fs);
best_std = inf; best_start = 1;
step_scan = max(1, round(0.5*fs));   % 每0.5秒扫一次，加速搜索
for s = 1:step_scan:(N-W_bias)
    seg = gz(s:s+W_bias-1);
    sd = std(seg);
    if sd < best_std
        best_std = sd;
        best_start = s;
    end
end
bias_idx = best_start:(best_start+W_bias-1);
gyro_bias_yaw = mean(gz(bias_idx));

fprintf('自动选取的零偏标定窗口: %.2f - %.2f s (gz std=%.3f deg/s)\n', ...
    bias_idx(1)/fs, bias_idx(end)/fs, best_std);
fprintf('陀螺Z轴零偏估计: %.4f deg/s\n', gyro_bias_yaw);
if best_std > 3
    fprintf(['警告: 全程未找到std<3deg/s的"较静止"窗口(当前最优std=%.2f)，\n' ...
             '      说明可能确实全程都在运动，零偏估计精度有限，\n' ...
             '      建议后续测试时首尾各留3~5秒真正静止不动的时间。\n'], best_std);
end

gz_c = gz - gyro_bias_yaw;    % 去零偏后的角速度, deg/s

%% ---------------------- 2.5 姿态角诊断(验证原始数据真实性, 不参与定位解算) ----
% 说明: 这一节纯粹是"诊断/取信"用途——用最简单的加速度计静态公式估算
% roll/pitch, 以及不做90°吸附的连续陀螺积分航向, 目的是让你直接看到
% 原始传感器数据里的真实抖动/漂移。这些曲线不会像最终轨迹那样"理想笔直",
% 恰恰可以证明程序确实在用你采集的真实数据, 只是第5~6步为了防止漂移
% 主动做了"归一化/吸附"处理。
ax = acc(:,1); ay = acc(:,2); az = acc(:,3);
roll_deg  = atan2d(ay, az);                      % 绕Xb轴, 加速度计静态估计(有噪声, 行走时会更抖)
pitch_deg = atan2d(-ax, sqrt(ay.^2+az.^2));      % 绕Yb轴, 同上

yaw_raw_deg = yaw0_deg + turn_sign*cumtrapz(t, gz_c);  % 连续积分航向(不吸附, 会漂移), 仅作对比

fprintf('\n--- 姿态角诊断(仅用于验证数据真实性) ---\n');
fprintf('Roll 范围: [%.1f, %.1f] deg, Pitch 范围: [%.1f, %.1f] deg\n', ...
    min(roll_deg), max(roll_deg), min(pitch_deg), max(pitch_deg));
fprintf('连续积分航向(未吸附)终值: %.1f deg\n', yaw_raw_deg(end));
fprintf('(若与后面吸附后航向状态机的终值相差较大, 说明纯陀螺积分确实会漂移,\n');
fprintf(' 这正是本方案放弃纯陀螺积分、改用"检测转弯+90吸附"的原因)\n');

%% ---------------------- 3. 步态检测(数步子) + 变步长(Weinberg模型) -----
acc_norm_s = movmean(acc_norm, smooth_win_acc);
step_idx = local_findpeaks(acc_norm_s, g0+step_min_prom, step_min_dist);

n_steps = length(step_idx);

% <<< 修改点 >>>
% 不再用"总路程/总步数"给每一步同一个数, 而是用每一步区间内加速度的
% 峰谷差算出这一步的"相对步幅"(Weinberg模型: step_len ∝ (acc_max-acc_min)^(1/4),
% 这是步态分析里常用的经验公式: 走得越猛/迈得越大, 上下起伏的加速度峰谷差
% 越大)。再用总路程对这批相对步幅整体乘一个缩放系数K做标定——K只做整体
% 比例缩放, 不抹平逐步差异, 因此每一步的步长都直接来自你的原始加速度数据、
% 彼此各不相同。
step_bounds = [1; step_idx(:)];         % 每一步的起始/结束采样点(用上一步到这一步的窗口)
raw_amp = zeros(n_steps,1);
for i = 1:n_steps
    seg = acc_norm_s(step_bounds(i):step_bounds(i+1));
    raw_amp(i) = max(seg) - min(seg);
end
raw_amp(raw_amp<=0) = median(raw_amp(raw_amp>0));   % 防止个别窗口异常导致除0/负数
raw_step_shape = raw_amp.^0.25;                     % Weinberg形状因子(相对步幅, 逐步不同)

K = total_known_dist / sum(raw_step_shape);         % 整体标定系数(只缩放比例, 不覆盖差异)
step_len_i = K * raw_step_shape;                    % 每一步单独的步长(m), 来自真实数据
step_len_mean = mean(step_len_i);                   % 仅用于打印参考

fprintf('\n--- 步态检测结果(变步长, Weinberg模型) ---\n');
fprintf('检测到步数: %d\n', n_steps);
fprintf('步长范围: [%.3f, %.3f] m, 平均步长: %.3f m (整体标定至总路程 %.1fm, 但逐步不同)\n', ...
    min(step_len_i), max(step_len_i), step_len_mean, total_known_dist);
fprintf('平均步频: %.2f 步/秒\n', n_steps/(N*dt));

%% ---------------------- 4. 转弯检测 + 90°吸附 ---------------------------
gz_s = movmean(gz_c, smooth_win_gz);
active = abs(gz_s) > turn_rate_th;

% 提取连续active片段
turn_list = [];   % 每行: [start_idx, end_idx, raw_angle_deg, snap_angle_deg]
in_seg = false; seg_start = 1;
for k = 1:N
    if active(k) && ~in_seg
        seg_start = k; in_seg = true;
    end
    if (~active(k) || k==N) && in_seg
        seg_end = k - (~active(k));  % 若因k==N结束仍active，则含k本身
        if ~active(k), seg_end = k-1; else, seg_end = k; end
        dur = (seg_end-seg_start)/fs;
        raw_angle = sum(gz_c(seg_start:seg_end))*dt;
        if dur >= turn_min_dur && abs(raw_angle) >= turn_min_angle
            snap_angle = round(raw_angle/90)*90;
            turn_list(end+1,:) = [seg_start, seg_end, raw_angle, snap_angle]; %#ok<SAGROW>
        end
        in_seg = false;
    end
end
n_turns = size(turn_list,1);

fprintf('\n--- 转弯检测结果 ---\n');
fprintf('检测到有效转弯次数: %d  (2圈矩形路线理论应为 %d 次)\n', n_turns, 4*n_loops);
for i = 1:n_turns
    fprintf('  转弯%d: t=%.2f~%.2fs, 原始积分角=%.1f°, 吸附后=%.0f°\n', ...
        i, turn_list(i,1)/fs, turn_list(i,2)/fs, turn_list(i,3), turn_list(i,4));
end
if n_turns ~= 4*n_loops
    fprintf(['提示: 检测到的转弯次数与理论值不符，可尝试调整 turn_rate_th /\n' ...
             '      turn_min_dur / turn_min_angle 后重新运行，并对照下面的\n' ...
             '      "转弯检测诊断图"人工核实每个转弯是否被正确框出。\n']);
end

%% ---------------------- 5. 航向: 分段90°锚点 + 段内真实陀螺波动 --------
% boundaries: 每个转弯的中点时刻, 作为切换航向的分界点
% headings_anchor(i): 第i个直线行走段的"90°吸附后"参考航向(仅用于防漂移锚定)
if n_turns > 0
    turn_mid = round(mean(turn_list(:,1:2),2));
else
    turn_mid = [];
end
boundaries = [1; turn_mid; N+1];
headings_anchor = zeros(n_turns+1,1);
headings_anchor(1) = yaw0_deg;
cum = yaw0_deg;
for i = 1:n_turns
    cum = cum + turn_sign*turn_list(i,4);
    headings_anchor(i+1) = cum;
end

fprintf('\n航向分段锚点(deg, 90°吸附): '); fprintf('%.0f  ', headings_anchor); fprintf('\n');

% <<< 修改点 >>>
% 之前直线段内航向是常数(锁死不变), 现在改为: 段内每一采样点的航向
% 直接取"陀螺仪连续积分出的真实航向"(yaw_raw_deg, 第2.5节已算好, 只做过
% 零偏修正、完全没有吸附/平滑), 只把整段的均值强制拉到该段的90°吸附值上——
% 也就是只锚定"直流分量"(防止长期漂移发散), 段内相对于均值的真实波动/抖动
% 完全保留、直接来自你的原始陀螺数据。
% 公式: heading(t) = anchor(段) + [yaw_raw(t) - mean(yaw_raw(段))]
heading_used_deg = zeros(N,1);
for seg_i = 1:length(boundaries)-1
    idx_range = boundaries(seg_i):boundaries(seg_i+1)-1;
    seg_mean_raw = mean(yaw_raw_deg(idx_range));
    heading_used_deg(idx_range) = headings_anchor(seg_i) + (yaw_raw_deg(idx_range) - seg_mean_raw);
end

%% ---------------------- 6. 按步重建轨迹(变步长 + 真实航向波动) ---------
pos = zeros(n_steps+1, 2);   % [N E]
for i = 1:n_steps
    p = step_idx(i);
    hd = deg2rad(heading_used_deg(p));   % 该步发生时刻的真实(锚定后)航向
    dN = step_len_i(i)*cos(hd);          % 该步的真实(标定后)步长
    dE = step_len_i(i)*sin(hd);
    pos(i+1,:) = pos(i,:) + [dN, dE];
end

%% ---------------------- 7. 生成参考矩形路线(真值) ----------------------
ref_pts = [0 0];
for loop = 1:n_loops
    for s = 1:length(seg_len_m)
        theta = deg2rad(seg_dir_deg(s));
        ref_pts(end+1,:) = ref_pts(end,:) + seg_len_m(s)*[cos(theta), sin(theta)]; %#ok<SAGROW>
    end
end

%% ---------------------- 8. 精度评估 ------------------------------------
closure_err = pos(end,:) - pos(1,:);
closure_dist = norm(closure_err);
fprintf('\n--- 精度评估 ---\n');
fprintf('闭合误差: dN=%.3f m, dE=%.3f m, 距离=%.3f m\n', closure_err(1), closure_err(2), closure_dist);
fprintf('闭合误差占总路程比例: %.2f%%\n', 100*closure_dist/total_known_dist);

%% ---------------------- 9. 绘图 ----------------------------------------
figure('Name','轨迹对比(PDR新方案)','Color','w');
plot(ref_pts(:,2), ref_pts(:,1), 'k--o', 'LineWidth',1.5,'MarkerSize',4,'DisplayName','参考矩形路线(真值)');
hold on;
plot(pos(:,2), pos(:,1), 'r.-', 'LineWidth',1.3,'MarkerSize',10,'DisplayName','PDR估计轨迹(每点=一步)');
plot(pos(1,2), pos(1,1), 'gs','MarkerSize',10,'MarkerFaceColor','g','DisplayName','起点');
plot(pos(end,2), pos(end,1), 'b^','MarkerSize',10,'MarkerFaceColor','b','DisplayName','终点(估计)');
xlabel('东向 E (m)'); ylabel('北向 N (m)');
title(sprintf('PDR轨迹 vs 真值矩形 (闭合误差 %.2fm, %.1f%%)', closure_dist, 100*closure_dist/total_known_dist));
legend('Location','best'); axis equal; grid on;

figure('Name','步态检测诊断','Color','w');
subplot(2,1,1);
plot(t, acc_norm_s, 'b'); hold on;
plot(t(step_idx), acc_norm_s(step_idx), 'r^', 'MarkerFaceColor','r','MarkerSize',5);
yline(g0+step_min_prom, 'k--');
xlabel('时间 (s)'); ylabel('加速度模值(平滑, m/s^2)');
title(sprintf('步态检测: 共检出 %d 步 (红三角=检测到的步)', n_steps));
legend('平滑加速度模','检测到的步','检测阈值参考线','Location','best'); grid on;

subplot(2,1,2);
stem(t(step_idx), step_len_i, 'filled', 'MarkerSize',4); hold on;
yline(step_len_mean, 'k--');
xlabel('时间 (s)'); ylabel('该步步长 (m)');
title(sprintf('逐步步长(Weinberg变步长, 范围[%.2f,%.2f]m, 不再是恒定值)', min(step_len_i), max(step_len_i)));
legend('每步真实步长','整体平均值(参考)','Location','best'); grid on;

figure('Name','转弯检测诊断','Color','w');
plot(t, gz_s, 'b'); hold on;
yline(turn_rate_th,'k--'); yline(-turn_rate_th,'k--');
for i = 1:n_turns
    xline(turn_list(i,1)/fs, 'r-');
    xline(turn_list(i,2)/fs, 'r-');
end
xlabel('时间 (s)'); ylabel('Z轴角速度(平滑, deg/s)');
title(sprintf('转弯检测: 共检出 %d 次转弯(红色竖线框出的区间)', n_turns));
grid on;

figure('Name','航向: 锚点 vs 实际使用(含真实波动)','Color','w');
stairs_t = [0; turn_mid/fs; N/fs];
stairs_h = [headings_anchor; headings_anchor(end)];
stairs(stairs_t, stairs_h, 'k--', 'LineWidth',1.2, 'DisplayName','90°吸附锚点(段均值)'); hold on;
plot(t, heading_used_deg, 'b', 'LineWidth', 0.8, 'DisplayName','实际参与定位的航向(锚点+真实波动)');
xlabel('时间 (s)'); ylabel('航向 (deg, 罗盘方位角)');
title('黑虚线=每段90°吸附均值(防漂移锚点)  蓝线=实际用于轨迹计算的航向(保留段内真实抖动)');
legend('Location','best'); grid on;

figure('Name','姿态角诊断(验证原始数据/说明为何最终轨迹被理想化)','Color','w');
subplot(3,1,1);
plot(t, roll_deg, 'b'); grid on;
xlabel('时间 (s)'); ylabel('Roll (deg)');
title('Roll: 加速度计静态估计(手持行走时天然有抖动, 未做任何平滑)');

subplot(3,1,2);
plot(t, pitch_deg, 'r'); grid on;
xlabel('时间 (s)'); ylabel('Pitch (deg)');
title('Pitch: 加速度计静态估计(同上, 用于确认原始数据确实在动/抖)');

subplot(3,1,3);
plot(t, yaw_raw_deg, 'Color',[0.5 0.5 0.5], 'LineWidth', 1, 'DisplayName','连续积分航向(未吸附, 会漂移)');
hold on;
stairs(stairs_t, stairs_h, 'b', 'LineWidth', 1.8, 'DisplayName','状态机航向(吸附90°后, 用于最终轨迹)');
xlabel('时间 (s)'); ylabel('航向 (deg)');
title('灰线=原始陀螺连续积分(真实抖动+漂移) vs 蓝线=90°吸附后(最终定位所用)');
legend('Location','best'); grid on;

%% ========================================================================
%  本地函数: 简易峰值检测(不依赖Signal Processing Toolbox的findpeaks)
% ========================================================================
function idx = local_findpeaks(x, min_height, min_dist)
% 找局部极大值, 且高度>=min_height, 相邻峰值间隔>=min_dist(采样点)
% 若两峰间隔过近，保留较高的一个。
idx = [];
N = length(x);
for i = 2:N-1
    if x(i) > x(i-1) && x(i) >= x(i+1) && x(i) >= min_height
        if isempty(idx) || (i - idx(end)) >= min_dist
            idx(end+1) = i; %#ok<AGROW>
        elseif x(i) > x(idx(end))
            idx(end) = i;
        end
    end
end
end