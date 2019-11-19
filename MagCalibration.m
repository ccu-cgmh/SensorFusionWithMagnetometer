clear;
close all;
addpath('Quaternions');
calibrate = 0;
jump_mode = 1;

MagFolder=fullfile('Data\1115SE\1115MAG');
dirOutput=dir(fullfile(MagFolder,'*.csv'));
MagFileNames={dirOutput.name}';
SensorPos = string(MagFileNames);
SensorPos = extractBefore(SensorPos,5);
DataFolder=fullfile('Data\1115SE\1115-21');
dirOutput=dir(fullfile(DataFolder,'*.csv'));
DataNames={dirOutput.name}';
% 找對齊時間的頭尾
STList = [];
EDList = [];
for i=1:length(DataNames)
    Matrix = readmatrix(string(DataFolder)+'/'+string(DataNames(i)));
    STList = [STList Matrix(1, 1)];
    EDList = [EDList Matrix(length(Matrix), 1)];
end
ST = max(STList);
ED = min(EDList);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i=1:length(MagFileNames)
    % Calibrate Mag
    MagCalMatrix = readmatrix(string(MagFolder)+'/'+string(MagFileNames(i)));
    MagCal = MagCalMatrix(:, 4:6);
%     scatter3(MagCal(:,1),MagCal(:,2),MagCal(:,3));
%     axis equal;
    [Abest,bbest,mfsbest] = magcal(MagCal,'auto');
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Calculate quat
    AccMatrix = readmatrix(string(DataFolder)+'/'+string(DataNames(i*3-2)));
    GyrMatrix = readmatrix(string(DataFolder)+'/'+string(DataNames(i*3-1)));
    MagMatrix = readmatrix(string(DataFolder)+'/'+string(DataNames(i*3)));
    Acc = AccMatrix(:, 4:6);
    AlignAcc = interp1(AccMatrix(:, 1), Acc, ST:10:ED);
    % Calibrate Acc
    load('Data\1118SeCal\'+SensorPos(i)+'.mat');
    AlignAccCorrected = (AlignAcc-bbestAcc)*AbestAcc;
    
    
    Gyr = GyrMatrix(:, 4:6);
    AlignGyr = interp1(GyrMatrix(:, 1), Gyr, ST:10:ED);
    Mag = MagMatrix(:, 4:6);
    AlignMag = interp1(MagMatrix(:, 1), Mag, ST:10:ED);
    % 校正磁力計值
    MagCorrected = (AlignMag-bbest)*Abest;
    % 單位
    AlignAccNew = AlignAccCorrected*9.8; % (m/s/s)
    AlignGyrNew = AlignGyr*pi/180; % (rad/s)
    MagCorrected = MagCorrected*10^6; % (?T)
    
    qimu = quaternion(zeros(size(AlignAccNew, 1), 4));
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     ifilt = imufilter();
%     for ii=1:size(AlignAccNew,1)
%         qimu(ii) = ifilt(AlignAccNew(ii,:), AlignGyrNew(ii,:));
%     end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    ifilt = ahrsfilter('ExpectedMagneticFieldStrength', mfsbest*10^6);
    for ii=1:size(AlignAccNew,1)
        qimu(ii) = ifilt(AlignAccNew(ii,:), AlignGyrNew(ii,:), MagCorrected(ii,:));
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
    
%     figure('Position', [9 39 900 600], 'NumberTitle', 'off', 'Name', 'TwoVelocity');
%     hold on;
%     plot([0 (1:length(acc_mag)-1)./100], acc_mag, 'r');
%     plot([0 (1:length(gyr_mag)-1)./100], gyr_mag, 'b');
%     title('Two Velocity');
%     xlabel('Time (s)');
%     ylabel('Velocity(m/s)');
%     hold off;
    
    acc_threshold = 3;
    mag_threshold = 60;
    stationary = zeros(size(gyr_mag));
    stationary = 1 - stationary;
    for t = 2:length(stationary)
        if acc_mag(t) > acc_threshold || gyr_mag(t) > mag_threshold
            stationary(t) = 0;
        end
    end
    stationaryStart = find([0; diff(stationary)] == 1);
    stationaryEnd = find([0; diff(stationary)] == -1);
    
    if (i == 10 ||i == 11) && calibrate == 0
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
            if Timing >= 40
                SampleTime = t;
                break;
            end
        end
%         figure('Position', [9 39 900 600], 'NumberTitle', 'off', 'Name', '1D Velocity');
%         hold on;
%         plot(1:length(vel(:,1)), sqrt(vel(:,1).*vel(:,1)+vel(:,2).*vel(:,2)+vel(:,3).*vel(:,3)), 'r');
%         plot(1:length(stationary), stationary, 'k');
%         %     plot(1:length(acc_mag), acc_mag, 'g');
%         %     plot(1:length(gyr_magFilt), gyr_magFilt, 'b');
%         title('1D Velocity');
%         xlabel('Time (s)');
%         ylabel('Velocity(m/s)');
%         hold off;
        
        pos = zeros(size(vel));
        tempPos = 0;
        PosShiftRange = 0.05;
        if jump_mode == 1
            PosShiftRange = 0.8;
        end
        for t = 2:length(pos)
            pos(t,:) = pos(t-1,:) + vel(t-1,:) * 0.01;
            if(stationary(t) == 1)
                if pos(t,3) >= tempPos-PosShiftRange && pos(t,3) <= tempPos+PosShiftRange
                    pos(t,3) = tempPos;     % force zero velocity when foot stationary
                end
                tempPos = pos(t,3);
            end
        end
            
        YShift = sqrt(pos(SampleTime,1).^2 + pos(SampleTime,2).^2 + pos(SampleTime,3).^2);
        V1 = [0, YShift, 0];
        V2 = [pos(SampleTime, 1), pos(SampleTime, 2), pos(SampleTime, 3)];

        R = vrrotvec(V1, V2);
        M = vrrotvec2mat(R);
        NewPosition = pos * M;

        ZPosDrift = zeros(size(pos));
        for ii = 1:min([numel(stationaryStart) numel(stationaryStart)])
            ZdriftRate = (NewPosition(stationaryStart(ii)-1, 3) - NewPosition(stationaryStart(ii), 3))/ (stationaryStart(ii) - stationaryEnd(ii));
            Zenum = 1:(stationaryStart(ii) - stationaryEnd(ii));
            Zdrift = Zenum'*ZdriftRate;
            ZPosDrift(stationaryEnd(ii):stationaryStart(ii)-1,3) = Zdrift;
        end
        NewPosition = NewPosition - ZPosDrift;
        figure('Position', [9 39 900 600], 'NumberTitle', 'off', 'Name', 'Position');
            hold on;
            plot(1:length(NewPosition(:,1)), NewPosition(:,1), 'r');
            plot(1:length(NewPosition(:,2)), NewPosition(:,2), 'g');
            plot(1:length(NewPosition(:,3)), NewPosition(:,3), 'b');
            title('穿戴Position');
            xlabel('Time (s)');
            ylabel('Position(m)');
            legend('X', 'Y', 'Z');
            hold off;

        PosDiff = diff(NewPosition);
        if i ==10
            csvwrite('OrientationOutput\RightFootPositionDiff.csv', PosDiff);
        elseif i == 11
            csvwrite('OrientationOutput\LeftFootPositionDiff.csv', PosDiff);
        end
    end
    
    
    
    stationaryStart = [1; stationaryStart];
    label = 1;
    for ii = 1:numel(stationaryEnd)
        if stationaryEnd(ii) - stationaryStart(ii) >= 100
            quat(round((stationaryStart(ii) + stationaryEnd(ii))/2), 5) = label;
            label = label + 1;
        end
    end

    if calibrate == 1
        figure('Position', [9 39 900 600], 'NumberTitle', 'off', 'Name', 'stationary');
        hold on;
        plot([0 (1:length(stationary)-1)./100], stationary, 'k');
        title('stationary');
        xlabel('Time (s)');
        ylabel('stationary');
        hold off;
    end
    csvwrite(strcat('OrientationOutput\', strcat(SensorPos(i), 'quaternions.csv')), quat);
end