clear;
close all;
% 四元數運算的函式
addpath('Quaternions');
SensorNum = [2, 6, 7, 10, 11]; %Sensor各部位編號 [右手腕, 左手腕, 腰, 右腳, 左腳]
%讀取sensor資料，並對齊時間
[FileName, AccMatrix, GyrMatrix] = ReadSensorRawData('Data\1024SE\1024-8\', 0);  % ReadSensorRawData(資料夾名稱, 磁力計) 0:無磁力計;1:有磁力計
% 計算四元數
QuatMatrix = Quat(AccMatrix, GyrMatrix); % Quat(加速度, 陀螺儀)
%%%%%%
% LabelForModel(AccMatrix, GyrMatrix, QuatMatrix, FileName); % 四元數加上Label，Label表示第幾個校準動作。
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[acc, vel] = SensorVel(AccMatrix, GyrMatrix, QuatMatrix); %單位(m/s/s, m/s)
[RightFistOP, LeftFistOP, WaistOP, RightFootOP, LeftFootOP] = ReadOpticalRawData('Data\1024OP', '8');  % (資料夾名稱, 動作編號) 單位(mm)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 右手 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ManualShift = 1;
OPVel = OpticalVel(RightFistOP);
SEVel = vel(:,SensorNum(1)*3-2:SensorNum(1)*3)*1000; %單位換成(mm/s)
ShifT = SE_OP_Shift(OPVel, SEVel, ManualShift);
AlignSEVel = SEVel(ShifT:length(SEVel), :);

OPV1D = sqrt(OPVel(:,1).^2 + OPVel(:,2).^2 + OPVel(:,3).^2);
AlignSEV1D = sqrt(AlignSEVel(:,1).^2 + AlignSEVel(:,2).^2 + AlignSEVel(:,3).^2);

figure('Position', [9 39 900 600], 'NumberTitle', 'off', 'Name', 'AllVel');
    hold on;
    plot(1:length(OPV1D), OPV1D, 'r');
    plot(1:length(AlignSEV1D), AlignSEV1D, 'b');
    title('AllVel');
    xlabel('Time (s)');
    ylabel('AllVel');
    hold off;
