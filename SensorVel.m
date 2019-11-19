function [ForceAccMatrix, velMatrix] = SensorVel(AccMatrix, GyrMatrix, QuatMatrix)
    [row, col] = size(AccMatrix);
    ForceAccMatrix = [];
    velMatrix = [];
    for i=1:col/3
        %單位換算，sensor原始的單位是(g)和(deg/s)
        %換算為(m/s/s)和(rad/s)
        Accelerometer = AccMatrix(:, i*3-2:i*3)*9.8;
        Gyroscope = GyrMatrix(:, i*3-2:i*3)*pi/180;
        quat = [QuatMatrix(:,i*4-3) QuatMatrix(:,i*4-2) QuatMatrix(:,i*4-1) QuatMatrix(:,i*4)];
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ACCELERATIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        acc = quaternRotate(Accelerometer, quat);
        acc(:,3) = acc(:,3) - mean(acc(1:200,3));
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 偵測靜止 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        acc_mag = sqrt(acc(:,1).^2 + acc(:,2).^2 + acc(:,3).^2);
        gyr_magFilt = sqrt(Gyroscope(:,1).^2 + Gyroscope(:,2).^2 + Gyroscope(:,3).^2);
        acc_threshold = 2;
        mag_threshold = 0.7;
        stationary = zeros(size(gyr_magFilt));
        stationary = 1 - stationary;
        for t = 2:length(stationary)
            if acc_mag(t) > acc_threshold || gyr_magFilt(t) > mag_threshold
                stationary(t) = 0;
            end
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% VELICITY %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        vel = zeros(size(acc));
        for t = 2:length(vel)
            vel(t,:) = vel(t-1,:) + acc(t-1,:) * 0.01;
            if(stationary(t) == 1)
                vel(t,:) = [0 0 0];     % force zero velocity when stationary
            end
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 扣除速度飄移 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
        ForceAccMatrix = [ForceAccMatrix acc];
        velMatrix = [velMatrix vel];
    end
end