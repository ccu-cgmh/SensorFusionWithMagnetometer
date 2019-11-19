clear;
close all;
addpath('Quaternions');
jump_mode = 1;
% quat = [0.7071 0.7071 0 0];
% rotm = quat2rotm(quat);
% rotm = rotm';
% 
% V1 = [0, 1, 0];
% V2 = [1, 0, 0];
% 
% R = vrrotvec(V1, V2);
% M = vrrotvec2mat(R);
S = readmatrix('1024SE/SensorData-1024-8.csv');
TimeStamp = S(:,1);
TimeStampDiff = diff(TimeStamp);
T(1) = 0;
for i=1:length(TimeStampDiff)
    T(i+1) = T(i) + TimeStampDiff(i)*0.001;
end
%•™∏} (11)
GyrX = S(:,62);
GyrY = S(:,63);
GyrZ = S(:,64);
AccX = S(:,65);
AccY = S(:,66);
AccZ = S(:,67);
% %S1
% GyrX = S(:,2);
% GyrY = S(:,3);
% GyrZ = S(:,4);
% AccX = S(:,5);
% AccY = S(:,6);
% AccZ = S(:,7);
% %S2
% GyrX = S(:,8);
% GyrY = S(:,9);
% GyrZ = S(:,10);
% AccX = S(:,11);
% AccY = S(:,12);
% AccZ = S(:,13);
% %S3
% GyrX = S(:,14);
% GyrY = S(:,15);
% GyrZ = S(:,16);
% AccX = S(:,17);
% AccY = S(:,18);
% AccZ = S(:,19);
% %S4
% GyrX = S(:,20);
% GyrY = S(:,21);
% GyrZ = S(:,22);
% AccX = S(:,23);
% AccY = S(:,24);
% AccZ = S(:,25);
% %S5
% GyrX = S(:,26);
% GyrY = S(:,27);
% GyrZ = S(:,28);
% AccX = S(:,29);
% AccY = S(:,30);
% AccZ = S(:,31);
Accelerometer = [AccX AccY AccZ]*9.8;
Gyroscope = [GyrX GyrY GyrZ]*pi/180;
ifilt = imufilter();
for ii=1:size(Accelerometer,1)
    qimu(ii) = ifilt(Accelerometer(ii,:), Gyroscope(ii,:));
end
[a,b,c,d] = parts(qimu);
quat = [a' b' c' d'];
%%%%%%%%%%%%%%%%%%%%%%% ACCELERATIONS
acc = quaternRotate([AccX AccY AccZ], quat);
acc = acc * 9.8;
acc(:,3) = acc(:,3) - mean(acc(1:200,3));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
acc_mag = sqrt(acc(:,1).^2 + acc(:,2).^2 + acc(:,3).^2);
gyr_magFilt = sqrt(GyrX.^2 + GyrY.^2 + GyrZ.^2);
acc_threshold = 2;
mag_threshold = 40;
stationary = zeros(size(gyr_magFilt));
stationary = 1 - stationary;
for t = 2:length(stationary)
    if acc_mag(t) > acc_threshold || gyr_magFilt(t) > mag_threshold
        stationary(t) = 0;
    end
end
vel = zeros(size(acc));
for t = 2:length(vel)
    vel(t,:) = vel(t-1,:) + acc(t-1,:) * (TimeStampDiff(t-1)/1000);
    if(stationary(t) == 1)
        vel(t,:) = [0 0 0];     % force zero velocity when foot stationary
    end
end
velDrift = zeros(size(vel));
stationaryStart = find([0; diff(stationary)] == -1);
stationaryEnd = find([0; diff(stationary)] == 1);
for i = 1:numel(stationaryEnd)
    driftRate = vel(stationaryEnd(i)-1, :) / (stationaryEnd(i) - stationaryStart(i));
    enum = 1:(stationaryEnd(i) - stationaryStart(i));
    drift = [enum'*driftRate(1) enum'*driftRate(2) enum'*driftRate(3)];
    velDrift(stationaryStart(i):stationaryEnd(i)-1, :) = drift;
end
% Remove integral drift
vel = vel - velDrift;
V = sqrt(vel(:,1).*vel(:,1)+vel(:,2).*vel(:,2)+vel(:,3).*vel(:,3));
Move = 0;
check1s = 0;
Timing = 0;
for t = 2:length(V)
    if V(t) ~= 0
        Move = 1;
    end
    if V(t) == 0 && Move == 1
        check1s = 1;
    end
    if V(t) == 0 && Move == 1 && check1s == 1
        Timing = Timing + 1;
    end
    if V(t) ~= 0 && Move == 1 && check1s == 1
        Timing = 0;
    end
    if Timing >= 60
        SampleTime = t;
        break;
    end
end
figure('Position', [9 39 900 600], 'NumberTitle', 'off', 'Name', '1D Velocity');
    hold on;
    plot(T(1:length(vel(:,1))), sqrt(vel(:,1).*vel(:,1)+vel(:,2).*vel(:,2)+vel(:,3).*vel(:,3)), 'r');
    plot(T(1:length(stationary)), stationary, 'k', 'LineWidth', 2);
    title('¨Ô¿πORI_Velocity');
    xlabel('Time (s)');
    ylabel('Velocity(m/s)');
    hold off;
pos = zeros(size(vel));
tempPos = 0;
PosShiftRange = 0.05;
if jump_mode == 1
    PosShiftRange = 0.8;
end
for t = 2:length(pos)
    pos(t,:) = pos(t-1,:) + vel(t-1,:) * (TimeStampDiff(t-1)/1000);
    if(stationary(t) == 1)
        if pos(t,3) >= tempPos-PosShiftRange && pos(t,3) <= tempPos+PosShiftRange
            pos(t,3) = tempPos;     % force zero velocity when foot stationary
        end
        tempPos = pos(t,3);
    end
end
figure('Position', [9 39 900 600], 'NumberTitle', 'off', 'Name', 'Position');
    hold on;
    plot(T(1:length(pos(:,1))), pos(:,1), 'r');
    plot(T(1:length(pos(:,2))), pos(:,2), 'g');
    plot(T(1:length(pos(:,3))), pos(:,3), 'b');
    title('¨Ô¿πPosition');
    xlabel('Time (s)');
    ylabel('Position(m)');
    legend('X', 'Y', 'Z');
    hold off;
% % % posPlot = pos;
% % % quatPlot = quat;
% % % samplePeriod = 1/100;
% % % % Extend final sample to delay end of animation
% % % extraTime = 0;
% % % onesVector = ones(extraTime*(1/samplePeriod), 1);
% % % posPlot = [posPlot; [posPlot(end, 1)*onesVector, posPlot(end, 2)*onesVector, posPlot(end, 3)*onesVector]];
% % % quatPlot = [quatPlot; [quatPlot(end, 1)*onesVector, quatPlot(end, 2)*onesVector, quatPlot(end, 3)*onesVector, quatPlot(end, 4)*onesVector]];
% % % 
% % % % Create 6 DOF animation
% % % SamplePlotFreq = 4;
% % % Spin = 120;
% % % SixDofAnimation(posPlot, quatern2rotMat(quatPlot), ...
% % %                 'SamplePlotFreq', SamplePlotFreq, 'Trail', 'All', ...
% % %                 'Position', [9 39 1280 768], 'View', [(100:(Spin/(length(posPlot)-1)):(100+Spin))', 10*ones(length(posPlot), 1)], ...
% % %                 'AxisLength', 0.1, 'ShowArrowHead', false, ...
% % %                 'Xlabel', 'X (m)', 'Ylabel', 'Y (m)', 'Zlabel', 'Z (m)', 'ShowLegend', false, ...
% % %                 'CreateAVI', false, 'AVIfileNameEnum', false, 'AVIfps', ((1/samplePeriod) / SamplePlotFreq));
YShift = sqrt(pos(SampleTime,1).^2 + pos(SampleTime,2).^2 + pos(SampleTime,3).^2);
V1 = [0, YShift, 0];
V2 = [pos(SampleTime, 1), pos(SampleTime, 2), pos(SampleTime, 3)];

R = vrrotvec(V1, V2);
M = vrrotvec2mat(R);
NewPosition = pos * M;

figure('Position', [9 39 900 600], 'NumberTitle', 'off', 'Name', 'Position');
    hold on;
    plot(T(1:length(NewPosition(:,1))), NewPosition(:,1), 'r');
    plot(T(1:length(NewPosition(:,2))), NewPosition(:,2), 'g');
    plot(T(1:length(NewPosition(:,3))), NewPosition(:,3), 'b');
    title('¨Ô¿πPosition');
    xlabel('Time (s)');
    ylabel('Position(m)');
    legend('X', 'Y', 'Z');
    hold off;

ZPosDrift = zeros(size(pos));
for i = 1:numel(stationaryEnd)
    ZdriftRate = (NewPosition(stationaryEnd(i)-1, 3) - NewPosition(stationaryEnd(i), 3))/ (stationaryEnd(i) - stationaryStart(i));
    Zenum = 1:(stationaryEnd(i) - stationaryStart(i));
    Zdrift = Zenum'*ZdriftRate;
    ZPosDrift(stationaryStart(i):stationaryEnd(i)-1,3) = Zdrift;
end
NewPosition = NewPosition - ZPosDrift;
% 
% Move = 0;
% check1s = 0;
% Timing = 0;
% for t = SampleTime:length(V)
%     if V(t) ~= 0
%         Move = 1;
%     end
%     if V(t) == 0 && Move == 1
%         check1s = 1;
%     end
%     if V(t) == 0 && Move == 1 && check1s == 1
%         Timing = Timing + 1;
%     end
%     if V(t) ~= 0 && Move == 1 && check1s == 1
%         Timing = 0;
%     end
%     if Timing >= 100
%         SampleTime2 = t;
%         break;
%     end
% end

% YShift = sqrt(pos(SampleTime2,1).^2 + pos(SampleTime2,2).^2 + pos(SampleTime2,3).^2);
% V1 = [XShift, YShift, 0];
% V2 = [NewPosition(SampleTime2,1), NewPosition(SampleTime2,2), NewPosition(SampleTime2,3)];
% 
% R = vrrotvec(V1, V2);
% M = vrrotvec2mat(R);
% NewPosition = NewPosition * M;
% 
figure('Position', [9 39 900 600], 'NumberTitle', 'off', 'Name', 'Position');
    hold on;
    plot(T(1:length(NewPosition(:,1))), NewPosition(:,1), 'r');
    plot(T(1:length(NewPosition(:,2))), NewPosition(:,2), 'g');
    plot(T(1:length(NewPosition(:,3))), NewPosition(:,3), 'b');
    title('¨Ô¿πPosition');
    xlabel('Time (s)');
    ylabel('Position(m)');
    legend('X', 'Y', 'Z');
    hold off;
    
PosDiff = diff(NewPosition);
csvwrite('OrientationOutput\LeftFootPositionDiff.csv', PosDiff);

posPlot = NewPosition;
quatPlot = quat;
samplePeriod = 1/100;
% Extend final sample to delay end of animation
extraTime = 3;
onesVector = ones(extraTime*(1/samplePeriod), 1);
posPlot = [posPlot; [posPlot(end, 1)*onesVector, posPlot(end, 2)*onesVector, posPlot(end, 3)*onesVector]];
quatPlot = [quatPlot; [quatPlot(end, 1)*onesVector, quatPlot(end, 2)*onesVector, quatPlot(end, 3)*onesVector, quatPlot(end, 4)*onesVector]];

% Create 6 DOF animation
SamplePlotFreq = 4;
Spin = 120;
% SixDofAnimation(posPlot, quatern2rotMat(quatPlot), ...
%                 'SamplePlotFreq', SamplePlotFreq, 'Trail', 'All', ...
%                 'Position', [9 39 1280 768], 'View', [(100:(Spin/(length(posPlot)-1)):(100+Spin))', 10*ones(length(posPlot), 1)], ...
%                 'AxisLength', 0.1, 'ShowArrowHead', false, ...
%                 'Xlabel', 'X (m)', 'Ylabel', 'Y (m)', 'Zlabel', 'Z (m)', 'ShowLegend', false, ...
%                 'CreateAVI', false, 'AVIfileNameEnum', false, 'AVIfps', ((1/samplePeriod) / SamplePlotFreq));