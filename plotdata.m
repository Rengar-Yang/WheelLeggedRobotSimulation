
%%
hold on;
plot(data_array1);
plot(data_array2);
hold off;
a=var(data_array1)
b=var(data_array2)
legend('With Kalman', 'Without Kalman');

%%
compensate=0;
Width = 2;
total_data_points = numel(data_array1); % 获取数据总数
sampling_rate = 50; % 每秒的数据点数
time_seconds1 = (0:total_data_points-1) / sampling_rate-compensate; % 计算时间向量，以秒为单位

total_data_points = numel(out.Reference); % 获取数据总数
sampling_rate = 5000; % 每秒的数据点数
time_seconds2 = (0:total_data_points-1) / sampling_rate-compensate; % 计算时间向量，以秒为单位

figure('Position', [0, 0, 400, 300]);
% yyaxis left; % 设置左边的纵坐标轴
xlabel('Time (Seconds)','FontSize', 16); % 设置横坐标标签
ylabel('Angle(Degree)','FontSize', 16); % 设置纵坐标标签
xlim([0, 40]);  % 设置x轴显示范围
ylim([-50, 40]);  % 设置y轴显示范围
set(gca, 'GridLineWidth', 1.5); % 设置网格线的粗细为 1.5
grid on;
hold on;
plot(time_seconds1,data_array1,'LineWidth', Width);
% plot(17.58, 32.99, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 5); % 使用红色圆形标记点

yyaxis right; % 设置左边的纵坐标轴
xlabel('Time (Seconds)','FontSize', 16); % 设置横坐标标签
ylabel('Start Signal','FontSize', 16); % 设置纵坐标标签
ylim([-0.5, 5]); % 设置右边纵坐标的范围为0到5
plot(time_seconds2,out.Reference,'r','LineWidth', Width);

% plot(26.38, 14.95, 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 5); % 使用红色圆形标记点
legend('Pitch angle','Start signal','FontSize', 10);



hold off;



% 将两个图形的纵坐标轴对齐
ax1 = gca; % 获取第一个图形的坐标轴
ax1.YAxis(1).Color = 'k'; % 设置第一个图形的左侧纵坐标轴颜色为黑色

ax2 = ax1; % 将第一个图形的坐标轴赋给第二个图形
ax2.YAxis(2).Color = 'r'; % 设置第二个图形的右侧纵坐标轴颜色为红色
%%
compensate=30;
total_data_points = numel(data_array1); % 获取数据总数
sampling_rate = 50; % 每秒的数据点数
time_seconds1 = (0:total_data_points-1) / sampling_rate-compensate; % 计算时间向量，以秒为单位
figure('Position', [0, 0, 400, 300]);
% yyaxis left; % 设置左边的纵坐标轴
xlabel('Time (seconds)'); % 设置横坐标标签
ylabel('Angle'); % 设置纵坐标标签
xlim([0, 15]);  % 设置x轴显示范围
ylim([-10, 50]);  % 设置y轴显示范围
set(gca, 'GridLineWidth', 1.5); % 设置网格线的粗细为 1.5
grid on;
hold on;
plot(time_seconds1,data_array1,'LineWidth', Width);
plot(6.06, 18.28, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 5); % 使用红色圆形标记点
% plot(28.2, 17.87, 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 5); % 使用红色圆形标记点
legend('Pitch angle','Collision','FontSize', 10);
hold off;

%%
compensate=137;
Width = 2;
total_data_points = numel(data_array1); % 获取数据总数
sampling_rate = 50; % 每秒的数据点数
time_seconds1 = (0:total_data_points-1) / sampling_rate-compensate; % 计算时间向量，以秒为单位

total_data_points = numel(data_array2); % 获取数据总数
sampling_rate = 50; % 每秒的数据点数
time_seconds2 = (0:total_data_points-1) / sampling_rate-compensate; % 计算时间向量，以秒为单位

figure('Position', [0, 0, 400, 300]);
% yyaxis left; % 设置左边的纵坐标轴
xlabel('Time (Seconds)','FontSize', 16); % 设置横坐标标签
ylabel('Angle(Degree)','FontSize', 16); % 设置纵坐标标签
xlim([0, 60]);  % 设置x轴显示范围
ylim([-20, 35]);  % 设置y轴显示范围
set(gca, 'GridLineWidth', 1.5); % 设置网格线的粗细为 1.5
grid on;
hold on;
plot(time_seconds1,data_array1,'LineWidth', Width);

yyaxis right; % 设置左边的纵坐标轴
xlabel('Time (Seconds)','FontSize', 16); % 设置横坐标标签
ylabel('Angle(Degree)','FontSize', 16); % 设置纵坐标标签
ylim([-35, 65]); % 设置右边纵坐标的范围为0到5
plot(time_seconds2,data_array2,'r','LineWidth', Width);

% plot(26.38, 14.95, 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 5); % 使用红色圆形标记点
legend('Pitch angle','Yaw speed','FontSize', 10);



hold off;



% 将两个图形的纵坐标轴对齐
ax1 = gca; % 获取第一个图形的坐标轴
ax1.YAxis(1).Color = 'k'; % 设置第一个图形的左侧纵坐标轴颜色为黑色

ax2 = ax1; % 将第一个图形的坐标轴赋给第二个图形
ax2.YAxis(2).Color = 'r'; % 设置第二个图形的右侧纵坐标轴颜色为红色

%% Dissertation Stand still 1

% Create the plot
figure;
hold on;

% Normal standing
plot(out.AnglePID, 'b-', 'LineWidth', 1.5); % Reference line

% Set labels and title
xlabel('Time (s)');
ylabel('Robot pitch angle (rad)');
title('Robot pitch angle when standing still');

% Add a legend
legend('Robot pitch angle');
% legend('Reference', 'w=1 (rad/s)', 'w=5 (rad/s)', 'w=10 (rad/s)', 'w=20 (rad/s)', 'Location', 'Southeast');

% Set grid
grid on;

% Adjust axes limits if necessary
xlim([0 10]);
ylim([-0.01 0.01]);
% ylim('auto');

hold off;

%% External disturbance
% Create the figure
figure;
grid on;
% Plot the left y-axis data (Pitch angle)
yyaxis left
plot(out.AnglePID, 'b-', 'LineWidth', 1.5);
ylabel('Angle (rad)'); % Label for the left y-axis
ylim([-0.2 0.15]); % Adjust y-axis limits for clarity
% ylim([-0.15 0.1]); % Adjust y-axis limits for clarity

% Plot the right y-axis data (Disturbance)
yyaxis right
plot(out.DisturbancePID, 'r-', 'LineWidth', 1.5);
ylabel('Disturbance(N)'); % Label for the right y-axis
ylim([-0.5 5]); % Adjust y-axis limits for clarity
% ylim([-0.5 4]); % Adjust y-axis limits for clarity

% Common x-axis label and title
xlabel('Time (Seconds)');
title('Robot pitch angle under external disturbance');

% Add a legend
legend('Robot pitch angle', 'External disturbance');

% Optional: Adjust the y-axis colors to match the data
yyaxis left
ax = gca;
ax.YColor = 'b'; % Set left y-axis color to blue

yyaxis right
ax.YColor = 'r'; % Set right y-axis color to red

% Optional: Adjust x-axis limits if needed
xlim([0 10]);

%% Movement
% Create the figure
figure;
grid on;
% Plot the left y-axis data (Pitch angle)
yyaxis left
plot(out.ActualMovement, 'b-', 'LineWidth', 1.5);
ylabel('Movement (M)'); % Label for the left y-axis
ylim(['auto']); % Adjust y-axis limits for clarity
% ylim([-0.15 0.1]); % Adjust y-axis limits for clarity

% Plot the right y-axis data (Disturbance)
yyaxis right
plot(out.ReferenceMovement, 'r-', 'LineWidth', 1.5);
ylabel('Reference movement (M)'); % Label for the right y-axis
ylim(['auto']); % Adjust y-axis limits for clarity
% ylim([-0.5 4]); % Adjust y-axis limits for clarity

% Common x-axis label and title
xlabel('Time (Seconds)');
title('Movement over time');

% Add a legend
legend('Actual movement', 'Reference movement');

% Optional: Adjust the y-axis colors to match the data
yyaxis left
ax = gca;
ax.YColor = 'b'; % Set left y-axis color to blue

yyaxis right
ax.YColor = 'r'; % Set right y-axis color to red

% Optional: Adjust x-axis limits if needed
xlim([0 10]);

%% Extra load

% Load the data if needed (uncomment if necessary)
% load('your_simulink_data.mat');

% Create the plot
figure;
hold on;

% Normal standing
plot(out.AnglePID, 'b-', 'LineWidth', 1.5); % Reference line
% plot(out.DisturbancePID, 'r-', 'LineWidth', 1.5); % w=1 rad/s
% plot(time, angle_w5, 'g-', 'LineWidth', 1.5); % w=5 rad/s
% plot(time, angle_w10, 'y-', 'LineWidth', 1.5); % w=10 rad/s
% plot(time, angle_w20, 'm-', 'LineWidth', 1.5); % w=20 rad/s

% Set labels and title
xlabel('Time (s)');
ylabel('Robot pitch angle (rad)');
title('Robot pitch angle under extra load');

% Add a legend
legend('Robot pitch angle');
% legend('Reference', 'w=1 (rad/s)', 'w=5 (rad/s)', 'w=10 (rad/s)', 'w=20 (rad/s)', 'Location', 'Southeast');

% Set grid
grid on;

% Adjust axes limits if necessary
xlim([0 10]);
ylim([-0.5 0.5]);
% ylim('auto');

hold off;

%% Fall form the high

% Load the data if needed (uncomment if necessary)
% load('your_simulink_data.mat');

% Create the plot
figure;
hold on;

% Normal standing
plot(out.AnglePID, 'b-', 'LineWidth', 1.5); % Reference line
% plot(out.DisturbancePID, 'r-', 'LineWidth', 1.5); % w=1 rad/s
% plot(time, angle_w5, 'g-', 'LineWidth', 1.5); % w=5 rad/s
% plot(time, angle_w10, 'y-', 'LineWidth', 1.5); % w=10 rad/s
% plot(time, angle_w20, 'm-', 'LineWidth', 1.5); % w=20 rad/s

% Set labels and title
xlabel('Time (s)');
ylabel('Robot pitch angle (rad)');
title('Robot pitch angle when falling from high');

% Add a legend
legend('Robot pitch angle');
% legend('Reference', 'w=1 (rad/s)', 'w=5 (rad/s)', 'w=10 (rad/s)', 'w=20 (rad/s)', 'Location', 'Southeast');

% Set grid
grid on;

% Adjust axes limits if necessary
xlim([0 10]);
ylim([-0.0000001 0.0000001]);
% ylim('auto');

hold off;

%% Change the height
% Create the figure
figure;
grid on;
% Plot the left y-axis data (Pitch angle)
yyaxis left
plot(out.Height, 'b-', 'LineWidth', 1.5);
ylabel('Height (mm)'); % Label for the left y-axis
ylim(['auto']); % Adjust y-axis limits for clarity
% ylim([-0.15 0.1]); % Adjust y-axis limits for clarity

% Plot the right y-axis data (Disturbance)
yyaxis right
plot(out.AnglePID, 'r-', 'LineWidth', 1.5);
ylabel('Robot pitch angle(rad)'); % Label for the right y-axis
ylim(['auto']); % Adjust y-axis limits for clarity
% ylim([-0.5 4]); % Adjust y-axis limits for clarity

% Common x-axis label and title
xlabel('Time (Seconds)');
title('Changing the height');

% Add a legend
legend('Robot height', 'Robot pitch angle');

% Optional: Adjust the y-axis colors to match the data
yyaxis left
ax = gca;
ax.YColor = 'b'; % Set left y-axis color to blue

yyaxis right
ax.YColor = 'r'; % Set right y-axis color to red

% Optional: Adjust x-axis limits if needed
xlim([0 10]);


%% Three figure
% Create the figure
figure;
grid on;
% Plot the left y-axis data (Pitch angle)
yyaxis left
hold on;
% plot(out.AnglePID, 'b-', 'LineWidth', 1.5);
% ylabel('Angle(rad)'); % Label for the left y-axis
plot(out.ActualMovement, 'b-', 'LineWidth', 1.5);
ylabel('Movement (M)'); % Label for the left y-axis
ylim(['auto']); % Adjust y-axis limits for clarity
% ylim([-0.15 0.1]); % Adjust y-axis limits for clarity

% plot(out.DisturbancePID, 'r-', 'LineWidth', 1.5);
% ylabel('DIsturbance(N)'); % Label for the right y-axis
plot(out.ReferenceMovement, 'g-', 'LineWidth', 1.5);
% ylabel('Reference movement(M)'); % Label for the right y-axis
ylim(['auto']); % Adjust y-axis limits for clarity
% ylim([-0.5 4]); % Adjust y-axis limits for clarity
hold off;

% Plot the right y-axis data (Disturbance)
yyaxis right
% plot(out.DisturbancePID, 'r-', 'LineWidth', 1.5);
% ylabel('DIsturbance(N)'); % Label for the right y-axis
plot(out.AnglePID, 'r-', 'LineWidth', 1.5);
ylabel('Robot pitch angle(rad)'); % Label for the right y-axis
ylim(['auto']); % Adjust y-axis limits for clarity
% ylim([-0.5 4]); % Adjust y-axis limits for clarity

% Common x-axis label and title
xlabel('Time (Seconds)');
% title('Robot pitch angle under external disturbance');
title('Movement over time');

% Add a legend
legend('Actual movement', 'Reference movement');
% legend('Actual movement', 'Reference movement');

% Optional: Adjust the y-axis colors to match the data
yyaxis left
ax = gca;
ax.YColor = 'b'; % Set left y-axis color to blue

yyaxis right
ax.YColor = 'r'; % Set right y-axis color to red

% Optional: Adjust x-axis limits if needed
xlim([0 10]);