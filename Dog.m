% 定义机器人的参数
l1 = 73;  % 大腿长度 (mm)
l2 = 60;  % 小腿长度 (mm)
L = 134;  % 前后腿之间的距离 (mm)
W = 65;   % 左右腿之间的距离 (mm)

% 定义步态周期参数
T = 1;    % 步态周期 (s)
dt = 0.1; % 时间步长 (s)
t = 0:dt:T; % 时间向量

% 定义步态（假设前腿与后腿的简单步态：正弦波）
stride = 30;  % 步态幅度 (mm)
z0 = 50;      % 默认腿的高度 (mm)

% 初始化存储角度的矩阵
angles = zeros(length(t), 12);

% 遍历每个时间步长
for i = 1:length(t)
    % 定义每条腿的目标位置 (简单的例子，前腿上下移动)
    x_fr = stride * sin(2*pi*t(i)/T);
    z_fr = z0 + stride/2 * cos(2*pi*t(i)/T);

    x_fl = -stride * sin(2*pi*t(i)/T);
    z_fl = z0 - stride/2 * cos(2*pi*t(i)/T);

    x_br = -stride * sin(2*pi*t(i)/T);
    z_br = z0 - stride/2 * cos(2*pi*t(i)/T);

    x_bl = stride * sin(2*pi*t(i)/T);
    z_bl = z0 + stride/2 * cos(2*pi*t(i)/T);

    % 使用逆运动学计算每个舵机的角度
    [ifr_f, ifr_t, ifr_s] = inverse_kinematics(x_fr, z_fr, l1, l2);
    [ifl_f, ifl_t, ifl_s] = inverse_kinematics(x_fl, z_fl, l1, l2);
    [ibr_f, ibr_t, ibr_s] = inverse_kinematics(x_br, z_br, l1, l2);
    [ibl_f, ibl_t, ibl_s] = inverse_kinematics(x_bl, z_bl, l1, l2);

    % 存储计算的角度
    angles(i, :) = [ifr_f, ifr_t, ifr_s, ibr_f, ibr_t, ibr_s, ibl_f, ibl_t, ibl_s, ifl_f, ifl_t, ifl_s];
end

% 显示结果
disp('舵机角度变化：');
disp(angles);

function [theta1, theta2, theta3] = inverse_kinematics(x, z, l1, l2)
    % 计算逆运动学角度，theta1: 髋关节，theta2: 膝关节，theta3: 踝关节
    D = (x^2 + z^2 - l1^2 - l2^2) / (2 * l1 * l2);
    theta2 = atan2(-sqrt(1-D^2), D);  % 膝关节角度
    theta1 = atan2(z, x) - atan2(l2*sin(theta2), l1+l2*cos(theta2)); % 髋关节角度
    theta3 = -theta1 - theta2; % 踝关节角度 (假设简化为反向)

    % 将角度转换为度数
    theta1 = rad2deg(theta1);
    theta2 = rad2deg(theta2);
    theta3 = rad2deg(theta3);
end
