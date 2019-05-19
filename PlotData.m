function PlotData(output,folder,runIdentifier)
rad2deg = 180/pi;
if ~exist(folder,'dir')
    mkdir(folder);
end
plotDimensions = [0 0 210*3 297*3];

%% plot Euler angle estimates
figure('Units','Pixels','Position',plotDimensions,'PaperOrientation','portrait');
h=gcf;
set(h,'PaperOrientation','portrait');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
margin = 5;
set(gca, 'Fontname', 'Times newman', 'Fontsize', 24);

% subplot(3,1,1);
plot(output.time_lapsed,output.euler_angles(:,1)*rad2deg,'r','LineWidth',2);
hold on;
plot(output.time_lapsed, output.euler_angles(:,1)*rad2deg+2*sqrt(output.euler_variances(:,1)*rad2deg),':k','LineWidth',4);

% minVal = rad2deg*min(output.euler_angles(:,1))-margin;
% maxVal = rad2deg*max(output.euler_angles(:,1))+margin;
% ylim([minVal maxVal]);
grid on;
titleText=strcat({'基于三重优化方法的EKF姿态融合算法'},runIdentifier);
title(titleText);
ylabel('横滚角 (度)');
xlabel('时间 (秒)');
legend('滤波角度','实时角度');

fileName='roll.png';
fullFileName = fullfile(folder, fileName);
saveas(h,fullFileName);

% subplot(3,1,2);
figure('Units','Pixels','Position',plotDimensions,'PaperOrientation','portrait');
h=gcf;
set(h,'PaperOrientation','portrait');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);

set(gca, 'Fontname', 'Times newman', 'Fontsize', 24);

plot(output.time_lapsed,output.euler_angles(:,2)*rad2deg,'r','LineWidth',2);
hold on;
plot(output.time_lapsed,output.euler_angles(:,2)*rad2deg+2*sqrt(output.euler_variances(:,2)*rad2deg),':k','LineWidth',4);

minVal = rad2deg*min(output.euler_angles(:,2))-margin;
maxVal = rad2deg*max(output.euler_angles(:,2))+margin;
ylim([minVal maxVal]);
grid on;
titleText=strcat({'基于三重优化方法的EKF姿态融合算法'},runIdentifier);
title(titleText);

ylabel('俯仰角 (度)');
xlabel('时间 (秒)');
legend('滤波角度','实时角度');

fileName='Pitch.png';
fullFileName = fullfile(folder, fileName);
saveas(h,fullFileName);

% subplot(3,1,3);
figure('Units','Pixels','Position',plotDimensions,'PaperOrientation','portrait');
h=gcf;
set(h,'PaperOrientation','portrait');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
set(gca, 'Fontname', 'Times newman', 'Fontsize', 24);

plot(output.time_lapsed,output.euler_angles(:,3)*rad2deg,'r','LineWidth',2);
hold on;
plot(output.time_lapsed,output.euler_angles(:,3)*rad2deg+2*sqrt(output.euler_variances(:,3)*rad2deg),':k','LineWidth',4);

minVal = rad2deg*min(output.euler_angles(:,3))-margin;
maxVal = rad2deg*max(output.euler_angles(:,3))+margin;
ylim([minVal maxVal]);
grid on;
titleText=strcat({'基于三重优化方法的EKF姿态融合算法'},runIdentifier);
title(titleText);
ylabel('航向角 (度)');
xlabel('时间 (秒)');
legend('滤波角度','实时角度');

fileName='yaw.png';
fullFileName = fullfile(folder, fileName);
saveas(h,fullFileName);
% fileName='euler_angle_estimates.png';
% fullFileName = fullfile(folder, fileName);
% saveas(h,fullFileName);

%% plot NED velocity estimates
figure('Units','Pixels','Position',plotDimensions,'PaperOrientation','portrait');
h=gcf;
set(h,'PaperOrientation','portrait');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
set(gca, 'Fontname', 'Times newman', 'Fontsize', 24);

subplot(3,1,1);
set(gca, 'Fontname', 'Times newman', 'Fontsize', 24);
plot(output.time_lapsed,output.velocity_NED(:,1));
grid on;
titleText=strcat({'基于东北天的速度估计'},runIdentifier);
title(titleText);
ylabel('北 (m/s)');
xlabel('时间 (s)');
legend('以北为正向的速度');

subplot(3,1,2);
set(gca, 'Fontname', 'Times newman', 'Fontsize', 24);
plot(output.time_lapsed,output.velocity_NED(:,2));
grid on;
ylabel('东 (m/s)');
xlabel('时间 (s)');
legend('以东为正向的速度');

subplot(3,1,3);
set(gca, 'Fontname', 'Times newman', 'Fontsize', 24);
plot(output.time_lapsed,output.velocity_NED(:,3));
grid on;
ylabel('上 (m/s)');
xlabel('时间 (s)');
legend('以上为正向的速度');

fileName='velocity_estimates.png';
fullFileName = fullfile(folder, fileName);
saveas(h,fullFileName);

%% plot NED position estimates
figure('Units','Pixels','Position',plotDimensions,'PaperOrientation','portrait');
h=gcf;
set(h,'PaperOrientation','portrait');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
set(gca, 'Fontname', 'Times newman', 'Fontsize', 24);

subplot(3,1,1);
set(gca, 'Fontname', 'Times newman', 'Fontsize', 24);
plot(output.time_lapsed,output.position_NED(:,1));
grid on;
titleText=strcat({'基于东北天的位置估计'},runIdentifier);
title(titleText);
ylabel('北 (m)');
xlabel('时间 (s)');
legend('以北为正向的位置估计');

subplot(3,1,2);
set(gca, 'Fontname', 'Times newman', 'Fontsize', 24);
plot(output.time_lapsed,output.position_NED(:,2));
grid on;
ylabel('东 (m)');
xlabel('时间 (sec)');
legend('以东为正向的位置估计');

subplot(3,1,3);
set(gca, 'Fontname', 'Times newman', 'Fontsize', 24);
plot(output.time_lapsed,output.position_NED(:,3));
grid on;
ylabel('上 (m)');
xlabel('时间 (s)');
legend('以上为正向的距离估计');

fileName='position_estimates.png';
fullFileName = fullfile(folder, fileName);
saveas(h,fullFileName);

%% plot IMU gyro bias estimates
figure('Units','Pixels','Position',plotDimensions,'PaperOrientation','portrait');
h=gcf;
set(h,'PaperOrientation','portrait');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);

margin = 0.1;

subplot(3,1,1);
plot(output.time_lapsed,(1/output.dt)*[output.gyro_bias(:,1),output.gyro_bias(:,1)+2*sqrt(output.state_variances(:,11)),output.gyro_bias(:,1)-2*sqrt(output.state_variances(:,11))]*rad2deg);%%output.gyro_bias(:,1)*rad2deg);
minVal = (1/output.dt)*rad2deg*min(output.gyro_bias(:,1))-margin;
maxVal = (1/output.dt)*rad2deg*max(output.gyro_bias(:,1))+margin;
ylim([minVal maxVal]);
grid on;
titleText=strcat({'IMU Gyro Bias Estimates'},runIdentifier);
title(titleText);
ylabel('X gyro (deg/s)');
xlabel('time (sec)');
legend('estimate','upper 95% bound','lower 95% bound');

subplot(3,1,2);
plot(output.time_lapsed,(1/output.dt)*[output.gyro_bias(:,2),output.gyro_bias(:,2)+2*sqrt(output.state_variances(:,12)),output.gyro_bias(:,2)-2*sqrt(output.state_variances(:,12))]*rad2deg);
minVal = (1/output.dt)*rad2deg*min(output.gyro_bias(:,2))-margin;
maxVal = (1/output.dt)*rad2deg*max(output.gyro_bias(:,2))+margin;
ylim([minVal maxVal]);
grid on;
ylabel('Y gyro (deg/s)');
xlabel('time (sec)');
legend('estimate','upper 95% bound','lower 95% bound');

subplot(3,1,3);
plot(output.time_lapsed,(1/output.dt)*[output.gyro_bias(:,3),output.gyro_bias(:,3)+2*sqrt(output.state_variances(:,13)),output.gyro_bias(:,3)-2*sqrt(output.state_variances(:,13))]*rad2deg);
minVal = (1/output.dt)*rad2deg*min(output.gyro_bias(:,3))-margin;
maxVal = (1/output.dt)*rad2deg*max(output.gyro_bias(:,3))+margin;
ylim([minVal maxVal]);
grid on;
ylabel('Z gyro (deg/s)');
xlabel('time (sec)');
legend('estimate','upper 95% bound','lower 95% bound');

fileName='imu_gyro_bias_estimates.png';
fullFileName = fullfile(folder, fileName);
saveas(h,fullFileName);

%% plot IMU accel bias estimates
figure('Units','Pixels','Position',plotDimensions,'PaperOrientation','portrait');
h=gcf;
set(h,'PaperOrientation','portrait');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);

margin = 0.1;

subplot(3,1,1);
plot(output.time_lapsed,(1/output.dt)*[output.accel_bias(:,1),output.accel_bias(:,1)+2*sqrt(output.state_variances(:,14)),output.accel_bias(:,1)-2*sqrt(output.state_variances(:,14))]);
titleText=strcat({'IMU Accel Bias Estimates'},runIdentifier);
title(titleText);
minVal = (1/output.dt)*min(output.accel_bias(:,1))-margin;
maxVal = (1/output.dt)*max(output.accel_bias(:,1))+margin;
ylim([minVal maxVal]);
grid on;
ylabel('X accel (m/s/s)');
xlabel('time (sec)');
legend('estimate','upper 95% bound','lower 95% bound');

subplot(3,1,2);
plot(output.time_lapsed,(1/output.dt)*[output.accel_bias(:,2),output.accel_bias(:,2)+2*sqrt(output.state_variances(:,15)),output.accel_bias(:,2)-2*sqrt(output.state_variances(:,15))]);
minVal = (1/output.dt)*min(output.accel_bias(:,1))-margin;
maxVal = (1/output.dt)*max(output.accel_bias(:,1))+margin;
ylim([minVal maxVal]);
grid on;
ylabel('Y accel (m/s/s)');
xlabel('time (sec)');
legend('estimate','upper 95% bound','lower 95% bound');

subplot(3,1,3);
plot(output.time_lapsed,(1/output.dt)*[output.accel_bias(:,3),output.accel_bias(:,3)+2*sqrt(output.state_variances(:,16)),output.accel_bias(:,3)-2*sqrt(output.state_variances(:,16))]);
minVal = (1/output.dt)*min(output.accel_bias(:,1))-margin;
maxVal = (1/output.dt)*max(output.accel_bias(:,1))+margin;
ylim([minVal maxVal]);
grid on;
ylabel('Z accel (m/s/s)');
xlabel('time (sec)');
legend('estimate','upper 95% bound','lower 95% bound');

fileName='imu_accel_bias_estimates.png';
fullFileName = fullfile(folder, fileName);
saveas(h,fullFileName);

%% plot magnetometer bias estimates
if (output.magFuseMethod <= 1)
    figure('Units','Pixels','Position',plotDimensions,'PaperOrientation','portrait');
    h=gcf;
    set(h,'PaperOrientation','portrait');
    set(h,'PaperUnits','normalized');
    set(h,'PaperPosition', [0 0 1 1]);
    
    subplot(3,1,1);
    plot(output.time_lapsed',[output.mag_XYZ(:,1),output.mag_XYZ(:,1)+2*sqrt(output.state_variances(:,20)),output.mag_XYZ(:,1)-2*sqrt(output.state_variances(:,20))]);
    grid on;
    titleText=strcat({'Magnetometer Bias Estimates'},runIdentifier);
    title(titleText);
    ylabel('X bias (gauss)');
    xlabel('time (sec)');
    legend('estimate','upper 95% bound','lower 95% bound');
    
    subplot(3,1,2);
    plot(output.time_lapsed',[output.mag_XYZ(:,2),output.mag_XYZ(:,2)+2*sqrt(output.state_variances(:,21)),output.mag_XYZ(:,2)-2*sqrt(output.state_variances(:,21))]);
    grid on;
    ylabel('Y bias (gauss)');
    xlabel('time (sec)');
    legend('estimate','upper 95% bound','lower 95% bound');
    
    subplot(3,1,3);
    plot(output.time_lapsed',[output.mag_XYZ(:,3),output.mag_XYZ(:,3)+2*sqrt(output.state_variances(:,22)),output.mag_XYZ(:,3)-2*sqrt(output.state_variances(:,22))]);
    grid on;
    ylabel('Z bias (gauss)');
    xlabel('time (sec)');
    legend('estimate','upper 95% bound','lower 95% bound');
    
    fileName='body_field_estimates.png';
    fullFileName = fullfile(folder, fileName);
    saveas(h,fullFileName);
end

%% plot earth field estimates
if (output.magFuseMethod <= 1)
    figure('Units','Pixels','Position',plotDimensions,'PaperOrientation','portrait');
    h=gcf;
    set(h,'PaperOrientation','portrait');
    set(h,'PaperUnits','normalized');
    set(h,'PaperPosition', [0 0 1 1]);
    
    margin = 0.1;
    
    subplot(4,1,1);
    plot(output.time_lapsed',[output.mag_NED(:,1),output.mag_NED(:,1)+2*sqrt(output.state_variances(:,17)),output.mag_NED(:,1)-2*sqrt(output.state_variances(:,17))]);
    minVal = min(output.mag_NED(:,1))-margin;
    maxVal = max(output.mag_NED(:,1))+margin;
    ylim([minVal maxVal]);
    grid on;
    titleText=strcat({'Earth Magnetic Field Estimates'},runIdentifier);
    title(titleText);
    ylabel('North (gauss)');
    xlabel('time (sec)');
    legend('estimate','upper 95% bound','lower 95% bound');
    
    subplot(4,1,2);
    plot(output.time_lapsed',[output.mag_NED(:,2),output.mag_NED(:,2)+2*sqrt(output.state_variances(:,18)),output.mag_NED(:,2)-2*sqrt(output.state_variances(:,18))]);
    minVal = min(output.mag_NED(:,2))-margin;
    maxVal = max(output.mag_NED(:,2))+margin;
    ylim([minVal maxVal]);
    grid on;
    ylabel('East (gauss)');
    xlabel('time (sec)');
    legend('estimate','upper 95% bound','lower 95% bound');
    
    subplot(4,1,3);
    plot(output.time_lapsed',[output.mag_NED(:,3),output.mag_NED(:,3)+2*sqrt(output.state_variances(:,19)),output.mag_NED(:,3)-2*sqrt(output.state_variances(:,19))]);
    grid on;
    minVal = min(output.mag_NED(:,3))-margin;
    maxVal = max(output.mag_NED(:,3))+margin;
    ylim([minVal maxVal]);
    ylabel('Down (gauss)');
    xlabel('time (sec)');
    legend('estimate','upper 95% bound','lower 95% bound');
    
    subplot(4,1,4);
    plot(output.time_lapsed',rad2deg*atan2(output.mag_NED(:,2),output.mag_NED(:,1)));
    grid on;
    titleText=strcat({'Magnetic Declination Estimate'},runIdentifier);
    title(titleText);
    ylabel('declination (deg)');
    xlabel('time (sec)');
    
    fileName='earth_field_estimates.png';
    fullFileName = fullfile(folder, fileName);
    saveas(h,fullFileName);
end

%% plot velocity innovations
if isfield(output.innovations,'vel_innov')
    
    figure('Units','Pixels','Position',plotDimensions,'PaperOrientation','portrait');
    h=gcf;
    set(h,'PaperOrientation','portrait');
    set(h,'PaperUnits','normalized');
    set(h,'PaperPosition', [0 0 1 1]);
    
    subplot(3,1,1);
    plot(output.innovations.vel_time_lapsed',[output.innovations.vel_innov(:,1),sqrt(output.innovations.vel_innov_var(:,1)),-sqrt(output.innovations.vel_innov_var(:,1))]);
    grid on;
    titleText=strcat({'Velocity Innovations and Variances'},runIdentifier);
    title(titleText);
    ylabel('North (m/s)');
    xlabel('time (sec)');
    legend('innovation','variance sqrt','variance sqrt');
    
    subplot(3,1,2);
    plot(output.innovations.vel_time_lapsed',[output.innovations.vel_innov(:,2),sqrt(output.innovations.vel_innov_var(:,2)),-sqrt(output.innovations.vel_innov_var(:,2))]);
    grid on;
    ylabel('East (m/s)');
    xlabel('time (sec)');
    legend('innovation','variance sqrt','variance sqrt');
    
    subplot(3,1,3);
    plot(output.innovations.vel_time_lapsed',[output.innovations.vel_innov(:,3),sqrt(output.innovations.vel_innov_var(:,3)),-sqrt(output.innovations.vel_innov_var(:,3))]);
    grid on;
    ylabel('Down (m/s)');
    xlabel('time (sec)');
    legend('innovation','variance sqrt','variance sqrt');
    
    fileName='velocity_fusion.png';
    fullFileName = fullfile(folder, fileName);
    saveas(h,fullFileName);
end

%% plot position innovations
if isfield(output.innovations,'posInnov')
    figure('Units','Pixels','Position',plotDimensions,'PaperOrientation','portrait');
    h=gcf;
    set(h,'PaperOrientation','portrait');
    set(h,'PaperUnits','normalized');
    set(h,'PaperPosition', [0 0 1 1]);
    
    subplot(3,1,1);
    plot(output.innovations.vel_time_lapsed',[output.innovations.posInnov(:,1),sqrt(output.innovations.posInnovVar(:,1)),-sqrt(output.innovations.posInnovVar(:,1))]);
    grid on;
    titleText=strcat({'Position Innovations and Variances'},runIdentifier);
    title(titleText);
    ylabel('North (m)');
    xlabel('time (sec)');
    legend('innovation','variance sqrt','variance sqrt');
    
    subplot(3,1,2);
    plot(output.innovations.vel_time_lapsed',[output.innovations.posInnov(:,2),sqrt(output.innovations.posInnovVar(:,2)),-sqrt(output.innovations.posInnovVar(:,2))]);
    grid on;
    ylabel('East (m)');
    xlabel('time (sec)');
    legend('innovation','variance sqrt','variance sqrt');
    
    subplot(3,1,3);
    plot(output.innovations.hgt_time_lapsed',[output.innovations.hgtInnov(:),sqrt(output.innovations.hgtInnovVar(:)),-sqrt(output.innovations.hgtInnovVar(:))]);
    grid on;
    ylabel('Up (m)');
    xlabel('time (sec)');
    legend('innovation','variance sqrt','variance sqrt');
    
    fileName='position_fusion.png';
    fullFileName = fullfile(folder, fileName);
    saveas(h,fullFileName);
end

%% plot magnetometer innovations
if isfield(output.innovations,'magInnov')
    
    figure('Units','Pixels','Position',plotDimensions,'PaperOrientation','portrait');
    h=gcf;
    set(h,'PaperOrientation','portrait');
    set(h,'PaperUnits','normalized');
    set(h,'PaperPosition', [0 0 1 1]);
    subplot(4,1,1);
    plot(output.innovations.mag_time_lapsed,[output.innovations.magInnov(:,1)';sqrt(output.innovations.magInnovVar(:,1))';-sqrt(output.innovations.magInnovVar(:,1))']);
    ylim([-0.15 0.15]);
    grid on;
    title(strcat({'Magnetometer Innovations and Variances'},runIdentifier));
    ylabel('X (gauss)');
    xlabel('time (sec)');
    legend('innovation','innovation variance sqrt','innovation variance sqrt');
    subplot(4,1,2);
    plot(output.innovations.mag_time_lapsed,[output.innovations.magInnov(:,2)';sqrt(output.innovations.magInnovVar(:,2))';-sqrt(output.innovations.magInnovVar(:,2))']);
    ylim([-0.15 0.15]);
    grid on;
    ylabel('Y (gauss)');
    xlabel('time (sec)');
    legend('innovation','innovation variance sqrt','innovation variance sqrt');
    subplot(4,1,3);
    plot(output.innovations.mag_time_lapsed,[output.innovations.magInnov(:,3)';sqrt(output.innovations.magInnovVar(:,3))';-sqrt(output.innovations.magInnovVar(:,3))']);
    ylim([-0.15 0.15]);
    grid on;
    ylabel('Z (gauss)');
    xlabel('time (sec)');
    legend('innovation','innovation variance sqrt','innovation variance sqrt');
    subplot(4,1,4);
    plot(output.innovations.mag_time_lapsed,output.innovations.magLength);
    ylim([0 0.7]);
    grid on;
    title(strcat({'Magnetic Flux'},runIdentifier));
    ylabel('Flux (Gauss)');
    xlabel('time (sec)');
    fileName='magnetometer_fusion.png';
    fullFileName = fullfile(folder, fileName);
    saveas(h,fullFileName);
    
end

%% plot magnetic yaw innovations
if isfield(output.innovations,'hdgInnov')
    
    figure('Units','Pixels','Position',plotDimensions,'PaperOrientation','portrait');
    h=gcf;
    set(h,'PaperOrientation','portrait');
    set(h,'PaperUnits','normalized');
    set(h,'PaperPosition', [0 0 1 1]);
    
    subplot(2,1,1);
    plot(output.innovations.mag_time_lapsed,[output.innovations.hdgInnov*rad2deg;sqrt(output.innovations.hdgInnovVar)*rad2deg;-sqrt(output.innovations.hdgInnovVar)*rad2deg]);
    ylim([-30 30]);
    grid on;
    title(strcat({'Magnetic Heading Innovations and Variances'},runIdentifier));
    ylabel('yaw innovation (deg)');
    xlabel('time (sec)');
    legend('innovation','innovation variance sqrt','innovation variance sqrt');
    subplot(2,1,2);
    plot(output.innovations.mag_time_lapsed,output.innovations.magLength);
    ylim([0 0.7]);
    grid on;
    title(strcat({'Magnetic Flux'},runIdentifier));
    ylabel('Flux (Gauss)');
    xlabel('time (sec)');
    fileName='magnetometer_fusion.png';
    fullFileName = fullfile(folder, fileName);
    saveas(h,fullFileName);
    
end

%% plot optical flow innovations
if isfield(output.innovations,'flowInnov')
    
    figure('Units','Pixels','Position',plotDimensions,'PaperOrientation','portrait');
    h=gcf;
    set(h,'PaperOrientation','portrait');
    set(h,'PaperUnits','normalized');
    set(h,'PaperPosition', [0 0 1 1]);
    subplot(2,1,1);
    plot(output.innovations.flow_time_lapsed,[output.innovations.flowInnov(:,1)';sqrt(output.innovations.flowInnovVar(:,1))';-sqrt(output.innovations.flowInnovVar(:,1))']);
    ylim([-1.0 1.0]);
    grid on;
    title(strcat({'Optical Flow Innovations and Variances'},runIdentifier));
    ylabel('X (rad/sec)');
    xlabel('time (sec)');
    legend('innovation','innovation variance sqrt','innovation variance sqrt');
    subplot(2,1,2);
    plot(output.innovations.flow_time_lapsed,[output.innovations.flowInnov(:,2)';sqrt(output.innovations.flowInnovVar(:,2))';-sqrt(output.innovations.flowInnovVar(:,2))']);
    ylim([-1.0 1.0]);
    grid on;
    ylabel('Y (rad/sec)');
    xlabel('time (sec)');
    legend('innovation','innovation variance sqrt','innovation variance sqrt');
    fileName='optical_flow_fusion.png';
    fullFileName = fullfile(folder, fileName);
    saveas(h,fullFileName);
    
end
%% plot ZED camera innovations
if isfield(output.innovations,'bodyVelInnov')
    
    figure('Units','Pixels','Position',plotDimensions,'PaperOrientation','portrait');
    h=gcf;
    set(h,'PaperOrientation','portrait');
    set(h,'PaperUnits','normalized');
    set(h,'PaperPosition', [0 0 1 1]);
    
    subplot(3,1,1);
    plot(output.innovations.bodyVel_time_lapsed,[output.innovations.bodyVelInnov(:,1)';sqrt(output.innovations.bodyVelInnovVar(:,1))';-sqrt(output.innovations.bodyVelInnovVar(:,1))']);
    grid on;
    title(strcat({'ZED Camera Innovations and Variances'},runIdentifier));
    ylabel('X (m/sec)');
    xlabel('time (sec)');
    legend('innovation','innovation variance sqrt','innovation variance sqrt');
    
    subplot(3,1,2);
    plot(output.innovations.bodyVel_time_lapsed,[output.innovations.bodyVelInnov(:,2)';sqrt(output.innovations.bodyVelInnovVar(:,2))';-sqrt(output.innovations.bodyVelInnovVar(:,2))']);
    grid on;
    ylabel('Y (m/sec)');
    xlabel('time (sec)');
    legend('innovation','innovation variance sqrt','innovation variance sqrt');
    
    subplot(3,1,3);
    plot(output.innovations.bodyVel_time_lapsed,[output.innovations.bodyVelInnov(:,3)';sqrt(output.innovations.bodyVelInnovVar(:,3))';-sqrt(output.innovations.bodyVelInnovVar(:,3))']);
    grid on;
    ylabel('Z (m/sec)');
    xlabel('time (sec)');
    legend('innovation','innovation variance sqrt','innovation variance sqrt');
    
    fileName='zed_camera_fusion.png';
    fullFileName = fullfile(folder, fileName);
    saveas(h,fullFileName);
    
end
