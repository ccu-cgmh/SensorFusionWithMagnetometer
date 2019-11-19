clear;
close all;
% �|���ƹB�⪺�禡
addpath('Quaternions');
SensorNum = [2, 6, 7, 10, 11]; %Sensor�U����s�� [�k���, �����, �y, �k�}, ���}]
%Ū��sensor��ơA�ù���ɶ�
[FileName, AccMatrix, GyrMatrix] = ReadSensorRawData('Data\1024SE\1024-8\', 0);  % ReadSensorRawData(��Ƨ��W��, �ϤO�p) 0:�L�ϤO�p;1:���ϤO�p
% �p��|����
QuatMatrix = Quat(AccMatrix, GyrMatrix); % Quat(�[�t��, ������)
%%%%%%
% LabelForModel(AccMatrix, GyrMatrix, QuatMatrix, FileName); % �|���ƥ[�WLabel�ALabel��ܲĴX�Ӯշǰʧ@�C
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[acc, vel] = SensorVel(AccMatrix, GyrMatrix, QuatMatrix); %���(m/s/s, m/s)
[RightFistOP, LeftFistOP, WaistOP, RightFootOP, LeftFootOP] = ReadOpticalRawData('Data\1024OP', '8');  % (��Ƨ��W��, �ʧ@�s��) ���(mm)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% �k�� %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ManualShift = 1;
OPVel = OpticalVel(RightFistOP);
SEVel = vel(:,SensorNum(1)*3-2:SensorNum(1)*3)*1000; %��촫��(mm/s)
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
