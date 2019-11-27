clear;
close all;
addpath('Quaternions');
% SensorNum = [2, 6, 7, 10, 11]; %Sensor场絪腹 [も得, オも得, 竬, 竲, オ竲]
ManualShift = 1;
edShift = 4;
OPFolder = 'Data\1115OP';
ACTNum = 2; % <-------------------------------------------------------
%合璸非戈Ж
MagFolder=fullfile('Data\1115SE\1115MAG');
dirOutput=dir(fullfile(MagFolder,'*.csv'));
MagFileNames={dirOutput.name}';
SensorPos = string(MagFileNames);
SensorPos = extractBefore(SensorPos,5);
DataFolder=fullfile('Data\1115SE\1115-'+string(ACTNum)); % <-------------------------------------------------------
dirOutput=dir(fullfile(DataFolder,'*.csv'));
DataNames={dirOutput.name}';
% т癸霍丁繷Ю
STList = [];
EDList = [];
for i=1:length(DataNames)
    Matrix = readmatrix(string(DataFolder)+'/'+string(DataNames(i)));
    STList = [STList Matrix(1, 1)];
    EDList = [EDList Matrix(length(Matrix), 1)];
end
ST = max(STList);
ED = min(EDList);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% も %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
MDPos = readmatrix('Data\1115MD\1115-'+string(ACTNum)+'\ㄢ竲キАLocationData+籐\も得location.csv'); % <-------------------------------------------------------
% MDPos = readmatrix('Data\1115MD\1115-'+string(ACTNum)+'\竲LocationData\も得location.csv');
% MDPos = readmatrix('Data\1115MD\1115-'+string(ACTNum)+'\オ竲LocationData\も得location.csv');
MDPos = readmatrix('Data\1115MDini\1115-'+string(ACTNum)+'\LocationData\も得location.csv');
SensorNum = 2;
% Calibrate Mag
MagCalMatrix = readmatrix(string(MagFolder)+'/'+string(MagFileNames(SensorNum)));
MagCal = MagCalMatrix(:, 4:6);
%     scatter3(MagCal(:,1),MagCal(:,2),MagCal(:,3));
%     axis equal;
[Abest,bbest,mfsbest] = magcal(MagCal,'auto');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calculate quat
AccMatrix = readmatrix(string(DataFolder)+'/'+string(DataNames(SensorNum*3-2)));
GyrMatrix = readmatrix(string(DataFolder)+'/'+string(DataNames(SensorNum*3-1)));
MagMatrix = readmatrix(string(DataFolder)+'/'+string(DataNames(SensorNum*3)));
Acc = AccMatrix(:, 4:6);
AlignAcc = interp1(AccMatrix(:, 1), Acc, ST:10:ED);
% Calibrate Acc
load('Data\1118SeCal\'+SensorPos(SensorNum)+'.mat');
AlignAccCorrected = (AlignAcc-bbestAcc)*AbestAcc;

Gyr = GyrMatrix(:, 4:6);
AlignGyr = interp1(GyrMatrix(:, 1), Gyr, ST:10:ED);
Mag = MagMatrix(:, 4:6);
AlignMag = interp1(MagMatrix(:, 1), Mag, ST:10:ED);
% タ合璸
MagCorrected = (AlignMag-bbest)*Abest;
% 虫
AlignAccNew = AlignAccCorrected*9.8; % (m/s/s)
AlignGyrNew = AlignGyr*pi/180; % (rad/s)
MagCorrected = MagCorrected*10^6; % (?T)

qimu = quaternion(zeros(size(AlignAccNew, 1), 4));
ifilt = ahrsfilter('ExpectedMagneticFieldStrength', mfsbest*10^6);
for ii=1:size(AlignAccNew,1)
    qimu(ii) = ifilt(AlignAccNew(ii,:), AlignGyrNew(ii,:), MagCorrected(ii,:));
end
[a,b,c,d] = parts(qimu);
quat = [a b c d];
%%%%%%%%%%%%%%%%%%%%%%% ACCELERATIONS
acc = quaternRotate(AlignAcc, quat);
temp = zeros(size(a));
quat = [a b c d temp];
acc = acc * 9.8;
acc(:,3) = acc(:,3) - mean(acc(1:200,3));
%%%%%%%%%%%%%%%%%%%%%%% Determine Stationary
acc_mag = sqrt(acc(:,1).^2 + acc(:,2).^2 + acc(:,3).^2);
gyr_mag = sqrt(AlignGyr(:,1).^2 + AlignGyr(:,2).^2 + AlignGyr(:,3).^2);

acc_threshold = 2;
mag_threshold = 40;
stationary = zeros(size(gyr_mag));
stationary = 1 - stationary;
for t = 2:length(stationary)
    if acc_mag(t) > acc_threshold || gyr_mag(t) > mag_threshold
        stationary(t) = 0;
    end
end
stationaryStart = find([0; diff(stationary)] == 1);
stationaryEnd = find([0; diff(stationary)] == -1);
vel = zeros(size(acc));
for t = 2:length(vel)
    vel(t,:) = vel(t-1,:) + acc(t-1,:) * 0.01;
    if(stationary(t) == 1)
        vel(t,:) = [0 0 0];     % force zero velocity when foot stationary
    end
end


velDrift = zeros(size(vel));
for ii = 1:min([numel(stationaryStart) numel(stationaryStart)])
    driftRate = vel(stationaryStart(ii)-1, :) / (stationaryStart(ii) - stationaryEnd(ii));
    enum = 1:(stationaryStart(ii) - stationaryEnd(ii));
    drift = [enum'*driftRate(1) enum'*driftRate(2) enum'*driftRate(3)];
    velDrift(stationaryEnd(ii):stationaryStart(ii)-1, :) = drift;
end
% Remove integral drift
vel = vel - velDrift;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Read OP data
%1115厩オも夹は
RWRA  = readmatrix(string(OPFolder) + '\LWRA_20191115-' + string(ACTNum) + '.csv');
RWRA = RWRA(:,1:3);
RWRB  = readmatrix(string(OPFolder) + '\LWRB_20191115-' + string(ACTNum) + '.csv');
RWRB = RWRB(:,1:3);
RightHandOP = (RWRA + RWRB) / 2;
% RightHandOP = RWRB;
RightHandOPV = zeros(length(RightHandOP)-1,3);
for i=2:length(RightHandOP)
    RightHandOPV(i, :) = (RightHandOP(i, :) - RightHandOP(i-1, :))/0.01;
end
RightHandOPV = RightHandOPV/1000;
RightHandOPV1D = sqrt(RightHandOPV(:,1).^2 + RightHandOPV(:,2).^2 + RightHandOPV(:,3).^2);
ShifT = SE_OP_Shift(RightHandOPV, vel, ManualShift);
AlignSEVel = vel(ShifT:length(vel), :);
AlignSEV1D = sqrt(AlignSEVel(:,1).^2 + AlignSEVel(:,2).^2 + AlignSEVel(:,3).^2);
figure('Position', [9 39 900 600], 'NumberTitle', 'off', 'Name', '1D Velocity');
hold on;
plot(1:length(AlignSEV1D), AlignSEV1D, 'b');
plot(1:length(RightHandOPV1D), RightHandOPV1D, 'r');
title('1D Velocity');
xlabel('Time (s)');
ylabel('Velocity(m/s)');
hold off;
% Automatically detect punches. Velocity is higher than 2.2, and the interval is greater than 0.5 seconds.
[pks,locs,w,p] = findpeaks(AlignSEV1D, 'MinPeakHeight', 2.2, 'MinPeakDistance',50);
[PunchStart,PunchEnd] = ForeBackTrough(AlignSEV1D, locs, -1); % -1ボт猧é, 0ボт箂翴
% 縩だ繷竚
PunchPos = zeros(size(AlignSEVel));
for t=2:length(PunchPos)
    PunchPos(t,:) = PunchPos(t-1,:) + AlignSEVel(t-1,:) * 0.01;
end
PunchPos = PunchPos*1000;
MDPos = MDPos(ShifT:length(MDPos), :)*10;
Draw(RightHandOP, RightHandOPV1D, PunchPos, AlignSEV1D, MDPos, PunchStart, PunchEnd+edShift);
MotionStart = PunchStart;
MotionEnd = PunchEnd;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% オも %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
MDPos = readmatrix('Data\1115MD\1115-'+string(ACTNum)+'\ㄢ竲キАLocationData+籐\オも得location.csv'); % <-------------------------------------------------------
% MDPos = readmatrix('Data\1115MD\1115-'+string(ACTNum)+'\竲LocationData\オも得location.csv');
% MDPos = readmatrix('Data\1115MD\1115-'+string(ACTNum)+'\オ竲LocationData\オも得location.csv');
MDPos = readmatrix('Data\1115MDini\1115-'+string(ACTNum)+'\LocationData\オも得location.csv');
SensorNum = 6;
% Calibrate Mag
MagCalMatrix = readmatrix(string(MagFolder)+'/'+string(MagFileNames(SensorNum)));
MagCal = MagCalMatrix(:, 4:6);
%     scatter3(MagCal(:,1),MagCal(:,2),MagCal(:,3));
%     axis equal;
[Abest,bbest,mfsbest] = magcal(MagCal,'auto');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calculate quat
AccMatrix = readmatrix(string(DataFolder)+'/'+string(DataNames(SensorNum*3-2)));
GyrMatrix = readmatrix(string(DataFolder)+'/'+string(DataNames(SensorNum*3-1)));
MagMatrix = readmatrix(string(DataFolder)+'/'+string(DataNames(SensorNum*3)));
Acc = AccMatrix(:, 4:6);
AlignAcc = interp1(AccMatrix(:, 1), Acc, ST:10:ED);
% Calibrate Acc
load('Data\1118SeCal\'+SensorPos(SensorNum)+'.mat');
AlignAccCorrected = (AlignAcc-bbestAcc)*AbestAcc;

Gyr = GyrMatrix(:, 4:6);
AlignGyr = interp1(GyrMatrix(:, 1), Gyr, ST:10:ED);
Mag = MagMatrix(:, 4:6);
AlignMag = interp1(MagMatrix(:, 1), Mag, ST:10:ED);
% タ合璸
MagCorrected = (AlignMag-bbest)*Abest;
% 虫
AlignAccNew = AlignAccCorrected*9.8; % (m/s/s)
AlignGyrNew = AlignGyr*pi/180; % (rad/s)
MagCorrected = MagCorrected*10^6; % (?T)

qimu = quaternion(zeros(size(AlignAccNew, 1), 4));
ifilt = ahrsfilter('ExpectedMagneticFieldStrength', mfsbest*10^6);
for ii=1:size(AlignAccNew,1)
    qimu(ii) = ifilt(AlignAccNew(ii,:), AlignGyrNew(ii,:), MagCorrected(ii,:));
end
[a,b,c,d] = parts(qimu);
quat = [a b c d];
%%%%%%%%%%%%%%%%%%%%%%% ACCELERATIONS
acc = quaternRotate(AlignAcc, quat);
temp = zeros(size(a));
quat = [a b c d temp];
acc = acc * 9.8;
acc(:,3) = acc(:,3) - mean(acc(1:200,3));
%%%%%%%%%%%%%%%%%%%%%%% Determine Stationary
acc_mag = sqrt(acc(:,1).^2 + acc(:,2).^2 + acc(:,3).^2);
gyr_mag = sqrt(AlignGyr(:,1).^2 + AlignGyr(:,2).^2 + AlignGyr(:,3).^2);

acc_threshold = 2;
mag_threshold = 40;
stationary = zeros(size(gyr_mag));
stationary = 1 - stationary;
for t = 2:length(stationary)
    if acc_mag(t) > acc_threshold || gyr_mag(t) > mag_threshold
        stationary(t) = 0;
    end
end
stationaryStart = find([0; diff(stationary)] == 1);
stationaryEnd = find([0; diff(stationary)] == -1);
vel = zeros(size(acc));
for t = 2:length(vel)
    vel(t,:) = vel(t-1,:) + acc(t-1,:) * 0.01;
    if(stationary(t) == 1)
        vel(t,:) = [0 0 0];     % force zero velocity when foot stationary
    end
end


velDrift = zeros(size(vel));
for ii = 1:min([numel(stationaryStart) numel(stationaryStart)])
    driftRate = vel(stationaryStart(ii)-1, :) / (stationaryStart(ii) - stationaryEnd(ii));
    enum = 1:(stationaryStart(ii) - stationaryEnd(ii));
    drift = [enum'*driftRate(1) enum'*driftRate(2) enum'*driftRate(3)];
    velDrift(stationaryEnd(ii):stationaryStart(ii)-1, :) = drift;
end
% Remove integral drift
vel = vel - velDrift;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Read OP data
%1115厩オも夹は
LWRA  = readmatrix(string(OPFolder) + '\RWRA_20191115-' + string(ACTNum) + '.csv');
LWRA = LWRA(:,1:3);
LWRB  = readmatrix(string(OPFolder) + '\RWRB_20191115-' + string(ACTNum) + '.csv');
LWRB = LWRB(:,1:3);
% LeftHandOP = (LWRA + LWRB) / 2;
LeftHandOP = LWRB;
LeftHandOPV = zeros(length(LeftHandOP)-1,3);
for i=2:length(LeftHandOP)
    LeftHandOPV(i, :) = (LeftHandOP(i, :) - LeftHandOP(i-1, :))/0.01;
end
LeftHandOPV = LeftHandOPV/1000;
LeftHandOPV1D = sqrt(LeftHandOPV(:,1).^2+LeftHandOPV(:,2).^2+LeftHandOPV(:,3).^2);
% ShifT = SE_OP_Shift(LeftHandOPV, vel, ManualShift);
AlignSEVel = vel(ShifT:length(vel), :);
AlignSEV1D = sqrt(AlignSEVel(:,1).^2 + AlignSEVel(:,2).^2 + AlignSEVel(:,3).^2);
figure('Position', [9 39 900 600], 'NumberTitle', 'off', 'Name', '1D Velocity');
hold on;
plot(1:length(AlignSEV1D), AlignSEV1D, 'b');
plot(1:length(LeftHandOPV1D), LeftHandOPV1D, 'r');
title('1D Velocity');
xlabel('Time (s)');
ylabel('Velocity(m/s)');
hold off;
% Automatically detect punches. Velocity is higher than 2.2, and the interval is greater than 0.5 seconds.
[pks,locs,w,p] = findpeaks(AlignSEV1D, 'MinPeakHeight', 2.2, 'MinPeakDistance',50);
[PunchStart,PunchEnd] = ForeBackTrough(AlignSEV1D, locs, -1); % -1ボт猧é, 0ボт箂翴
% 縩だ繷竚
PunchPos = zeros(size(AlignSEVel));
for t=2:length(PunchPos)
    PunchPos(t,:) = PunchPos(t-1,:) + AlignSEVel(t-1,:) * 0.01;
end
PunchPos = PunchPos*1000;
MDPos = MDPos(ShifT:length(MDPos), :)*10;
Draw(LeftHandOP, LeftHandOPV1D, PunchPos, AlignSEV1D, MDPos, PunchStart, PunchEnd+edShift);

MotionStart = [MotionStart PunchStart];
% MotionStart = sort(MotionStart);
MotionEnd = [MotionEnd PunchEnd];
% MotionEnd = sort(MotionEnd);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 竬 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MDWaistPosR = readmatrix('Data\1115MD\1115-'+string(ACTNum)+'\竲LocationData\竬location.csv');
% MDWaistPosL = readmatrix('Data\1115MD\1115-'+string(ACTNum)+'\オ竲LocationData\竬location.csv');% <-------------------------------------------------------
% Wvar = 0.01*ones(size(MDWaistPosR(:,1)));
% WRx = [MDWaistPosR(:,1) Wvar];
% WLx = [MDWaistPosL(:,1) Wvar];
% WRy = [MDWaistPosR(:,2) Wvar];
% WLy = [MDWaistPosL(:,2) Wvar];
% WRz = [MDWaistPosR(:,3) Wvar];
% WLz = [MDWaistPosL(:,3) Wvar];
% 
% dt = 0.01;
% n = length(WRx);
% % state matrix
% X = zeros(2,1);
% % covariance matrix
% P = zeros(2,2);
% % kalman filter output through the whole time
% X_arr = zeros(n, 2);
% % system noise
% Q = [0.04 0;
%     0 1];
% % transition matrix
% F = [1 dt;
%     0 1];
% % observation matrix
% H = [1 0];
% % fusion
% for i = 1:n
%     if (i == 1)
%         [X, P] = init_kalman(X, (WRx(i, 1) + WLx(i, 1))/2); % initialize the state using the 1st sensor
%     else
%         [X, P] = prediction(X, P, Q, F);
%         
%         [X, P] = update(X, P, WRx(i, 1), WRx(i, 2), H);
%         [X, P] = update(X, P, WLx(i, 1), WLx(i, 2), H);
%     end
%     
%     X_arr(i, :) = X;
% end
% NewWaistX = X_arr(:,1);
% for i = 1:n
%     if (i == 1)
%         [X, P] = init_kalman(X, (WRy(i, 1) + WLy(i, 1))/2); % initialize the state using the 1st sensor
%     else
%         [X, P] = prediction(X, P, Q, F);
%         
%         [X, P] = update(X, P, WRy(i, 1), WRy(i, 2), H);
%         [X, P] = update(X, P, WLy(i, 1), WLy(i, 2), H);
%     end
%     
%     X_arr(i, :) = X;
% end
% NewWaistY = X_arr(:,1);
% for i = 1:n
%     if (i == 1)
%         [X, P] = init_kalman(X, (WRz(i, 1) + WLz(i, 1))/2); % initialize the state using the 1st sensor
%     else
%         [X, P] = prediction(X, P, Q, F);
%         
%         [X, P] = update(X, P, WRz(i, 1), WRz(i, 2), H);
%         [X, P] = update(X, P, WLz(i, 1), WLz(i, 2), H);
%     end
%     
%     X_arr(i, :) = X;
% end
% NewWaistZ = X_arr(:,1);
% MDWaistPos = [NewWaistX NewWaistY NewWaistZ];

MDWaistPos = readmatrix('Data\1115MD\1115-'+string(ACTNum)+'\ㄢ竲キАLocationData+籐\竬location.csv');
MDWaistPos = readmatrix('Data\1115MDini\1115-'+string(ACTNum)+'\LocationData\竬location.csv');




SensorNum = 7;
% Calibrate Mag
MagCalMatrix = readmatrix(string(MagFolder)+'/'+string(MagFileNames(SensorNum)));
MagCal = MagCalMatrix(:, 4:6);
%     scatter3(MagCal(:,1),MagCal(:,2),MagCal(:,3));
%     axis equal;
[Abest,bbest,mfsbest] = magcal(MagCal,'auto');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calculate quat
AccMatrix = readmatrix(string(DataFolder)+'/'+string(DataNames(SensorNum*3-2)));
GyrMatrix = readmatrix(string(DataFolder)+'/'+string(DataNames(SensorNum*3-1)));
MagMatrix = readmatrix(string(DataFolder)+'/'+string(DataNames(SensorNum*3)));
Acc = AccMatrix(:, 4:6);
AlignAcc = interp1(AccMatrix(:, 1), Acc, ST:10:ED);
% Calibrate Acc
load('Data\1118SeCal\'+SensorPos(SensorNum)+'.mat');
AlignAccCorrected = (AlignAcc-bbestAcc)*AbestAcc;

Gyr = GyrMatrix(:, 4:6);
AlignGyr = interp1(GyrMatrix(:, 1), Gyr, ST:10:ED);
Mag = MagMatrix(:, 4:6);
AlignMag = interp1(MagMatrix(:, 1), Mag, ST:10:ED);
% タ合璸
MagCorrected = (AlignMag-bbest)*Abest;
% 虫
AlignAccNew = AlignAccCorrected*9.8; % (m/s/s)
AlignGyrNew = AlignGyr*pi/180; % (rad/s)
MagCorrected = MagCorrected*10^6; % (?T)

qimu = quaternion(zeros(size(AlignAccNew, 1), 4));
ifilt = ahrsfilter('ExpectedMagneticFieldStrength', mfsbest*10^6);
for ii=1:size(AlignAccNew,1)
    qimu(ii) = ifilt(AlignAccNew(ii,:), AlignGyrNew(ii,:), MagCorrected(ii,:));
end
[a,b,c,d] = parts(qimu);
quat = [a b c d];
%%%%%%%%%%%%%%%%%%%%%%% ACCELERATIONS
acc = quaternRotate(AlignAcc, quat);
temp = zeros(size(a));
quat = [a b c d temp];
acc = acc * 9.8;
acc(:,3) = acc(:,3) - mean(acc(1:200,3));
%%%%%%%%%%%%%%%%%%%%%%% Determine Stationary
acc_mag = sqrt(acc(:,1).^2 + acc(:,2).^2 + acc(:,3).^2);
gyr_mag = sqrt(AlignGyr(:,1).^2 + AlignGyr(:,2).^2 + AlignGyr(:,3).^2);

acc_threshold = 1;
mag_threshold = 20;
stationary = zeros(size(gyr_mag));
stationary = 1 - stationary;
for t = 2:length(stationary)
    if acc_mag(t) > acc_threshold || gyr_mag(t) > mag_threshold
        stationary(t) = 0;
    end
end
stationaryStart = find([0; diff(stationary)] == 1);
stationaryEnd = find([0; diff(stationary)] == -1);
vel = zeros(size(acc));
for t = 2:length(vel)
    vel(t,:) = vel(t-1,:) + acc(t-1,:) * 0.01;
    if(stationary(t) == 1)
        vel(t,:) = [0 0 0];     % force zero velocity when foot stationary
    end
end


velDrift = zeros(size(vel));
for ii = 1:min([numel(stationaryStart) numel(stationaryStart)])
    driftRate = vel(stationaryStart(ii)-1, :) / (stationaryStart(ii) - stationaryEnd(ii));
    enum = 1:(stationaryStart(ii) - stationaryEnd(ii));
    drift = [enum'*driftRate(1) enum'*driftRate(2) enum'*driftRate(3)];
    velDrift(stationaryEnd(ii):stationaryStart(ii)-1, :) = drift;
end
% Remove integral drift
vel = vel - velDrift;
AlignSEVel = vel(ShifT:length(vel), :);
AlignSEV1D = sqrt(AlignSEVel(:,1).^2 + AlignSEVel(:,2).^2 + AlignSEVel(:,3).^2);

LPSI  = readmatrix(string(OPFolder) + '\LPSI_20191115-' + string(ACTNum) + '.csv');
LPSI = LPSI(:,1:3);
RPSI  = readmatrix(string(OPFolder) + '\RPSI_20191115-' + string(ACTNum) + '.csv');
RPSI = RPSI(:,1:3);
WaistOP = (RPSI + LPSI) / 2;
WaistOPV = zeros(length(WaistOP)-1,3);
for i=2:length(WaistOP)
    WaistOPV(i, :) = (WaistOP(i, :) - WaistOP(i-1, :))/0.01;
end
WaistOPV = WaistOPV/1000;
WaistOPV1D = sqrt(WaistOPV(:,1).^2+WaistOPV(:,2).^2+WaistOPV(:,3).^2);
figure('Position', [9 39 900 600], 'NumberTitle', 'off', 'Name', '1D Velocity');
hold on;
plot(1:length(AlignSEV1D), AlignSEV1D, 'b');
plot(1:length(WaistOPV1D), WaistOPV1D, 'r');
title('1D Velocity');
xlabel('Time (s)');
ylabel('Velocity(m/s)');
hold off;

% 縩だ竬场竚
WaistPos = zeros(size(AlignSEVel));
for t=2:length(WaistPos)
    WaistPos(t,:) = WaistPos(t-1,:) + AlignSEVel(t-1,:) * 0.01;
end
WaistPos = WaistPos*1000;
MDWaistPos = MDWaistPos(ShifT:length(MDWaistPos), :)*10;
Draw(WaistOP, WaistOPV1D, WaistPos, AlignSEV1D, MDWaistPos, MotionStart, MotionEnd+edShift);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% WR = readmatrix('Data\1115MD\1115-'+string(ACTNum)+'\竲LocationData\竬location.csv');
% WL = readmatrix('Data\1115MD\1115-'+string(ACTNum)+'\オ竲LocationData\竬location.csv');
% % WR = readmatrix('Data\1115MDini\1115-'+string(ACTNum)+'\LocationData\竲︳衡竬location.csv');
% % WL = readmatrix('Data\1115MDini\1115-'+string(ACTNum)+'\LocationData\オ竲︳衡竬location.csv');
% WR = WR(ShifT:length(WR), :)*10;
% WL = WL(ShifT:length(WL), :)*10;
% Wvar = 0.06*ones(size(WR(:,1)));
% WRx = [WR(:,1) Wvar];
% WLx = [WL(:,1) Wvar];
% WRy = [WR(:,2) Wvar];
% WLy = [WL(:,2) Wvar];
% WRz = [WR(:,3) Wvar];
% WLz = [WL(:,3) Wvar];
% 
% dt = 0.01;
% n = length(WRx);
% % state matrix
% X = zeros(2,1);
% % covariance matrix
% P = zeros(2,2);
% % kalman filter output through the whole time
% X_arr = zeros(n, 2);
% % system noise
% Q = [0.04 0;
%     0 1];
% % transition matrix
% F = [1 dt;
%     0 1];
% % observation matrix
% H = [1 0];
% 
% % fusion
% for i = 1:n
%     if (i == 1)
%         [X, P] = init_kalman(X, (WRx(i, 1) + WLx(i, 1))/2); % initialize the state using the 1st sensor
%     else
%         [X, P] = prediction(X, P, Q, F);
%         
%         [X, P] = update(X, P, WRx(i, 1), WRx(i, 2), H);
%         [X, P] = update(X, P, WLx(i, 1), WLx(i, 2), H);
%     end
%     
%     X_arr(i, :) = X;
% end
% X_arr(:, 1) = X_arr(:, 1) - (X_arr(1, 1) - WaistOP(1, 1));
% figure('Position', [9 39 900 600], 'NumberTitle', 'off', 'Name', 'POS X');
% hold on;
% plot(1:length(WRx(:, 1)), WRx(:, 1), '--', 'LineWidth', 1);
% plot(1:length(WLx(:, 1)), WLx(:, 1), '--', 'LineWidth', 1);
% plot(1:length(X_arr(:, 1)), X_arr(:, 1), 'LineWidth', 4);
% plot(1:length(WaistOP(:, 1)), WaistOP(:, 1), 'LineWidth', 8);
% hold off;
% legend('Sensor Input 1', 'Sensor Input 2', 'Fused Output');
% for i = 1:n
%     if (i == 1)
%         [X, P] = init_kalman(X, (WRy(i, 1) + WLy(i, 1))/2); % initialize the state using the 1st sensor
%     else
%         [X, P] = prediction(X, P, Q, F);
%         
%         [X, P] = update(X, P, WRy(i, 1), WRy(i, 2), H);
%         [X, P] = update(X, P, WLy(i, 1), WLy(i, 2), H);
%     end
%     
%     X_arr(i, :) = X;
% end
% X_arr(:, 1) = X_arr(:, 1) - (X_arr(1, 1) - WaistOP(1, 2));
% figure('Position', [9 39 900 600], 'NumberTitle', 'off', 'Name', 'POS Y');
% hold on;
% plot(1:length(WRy(:, 1)), WRy(:, 1), '--', 'LineWidth', 1);
% plot(1:length(WLy(:, 1)), WLy(:, 1), '--', 'LineWidth', 1);
% plot(1:length(X_arr(:, 1)), X_arr(:, 1), 'LineWidth', 4);
% plot(1:length(WaistOP(:, 2)), WaistOP(:, 2), 'LineWidth', 8);
% hold off;
% legend('Sensor Input 1', 'Sensor Input 2', 'Fused Output');
% for i = 1:n
%     if (i == 1)
%         [X, P] = init_kalman(X, (WRz(i, 1) + WLz(i, 1))/2); % initialize the state using the 1st sensor
%     else
%         [X, P] = prediction(X, P, Q, F);
%         
%         [X, P] = update(X, P, WRz(i, 1), WRz(i, 2), H);
%         [X, P] = update(X, P, WLz(i, 1), WLz(i, 2), H);
%     end
%     
%     X_arr(i, :) = X;
% end
% X_arr(:, 1) = X_arr(:, 1) - (X_arr(1, 1) - WaistOP(1, 3));
% figure('Position', [9 39 900 600], 'NumberTitle', 'off', 'Name', 'POS Z');
% hold on;
% plot(1:length(WRz(:, 1)), WRz(:, 1), '--', 'LineWidth', 1);
% plot(1:length(WLz(:, 1)), WLz(:, 1), '--', 'LineWidth', 1);
% plot(1:length(X_arr(:, 1)), X_arr(:, 1), 'LineWidth', 4);
% plot(1:length(WaistOP(:, 3)), WaistOP(:, 3), 'LineWidth', 8);
% hold off;
% legend('Sensor Input 1', 'Sensor Input 2', 'Fused Output');
% 
% 
% function [X, P] = init_kalman(X, y)
%     X(1,1) = y;
%     X(2,1) = 0;
% 
%     P = [100 0;
%          0   300];
% end
% 
% function [X, P] = prediction(X, P, Q, F)
%     X = F*X;
%     P = F*P*F' + Q;
% end
% 
% function [X, P] = update(X, P, y, R, H)
%     Inn = y - H*X;
%     S = H*P*H' + R;
%     K = P*H'/S;
% 
%     X = X + K*Inn;
%     P = P - K*H*P;
% end