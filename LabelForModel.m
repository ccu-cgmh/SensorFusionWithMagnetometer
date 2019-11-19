function LabelForModel(AccMatrix, GyrMatrix, QuatMatrix, FileName)
    [row, col] = size(AccMatrix);
    for i=1:col/3
        %單位換算，sensor原始的單位是(g)和(deg/s)
        %換算為(m/s/s)和(rad/s)
        Accelerometer = AccMatrix(:, i*3-2:i*3)*9.8;
        Gyroscope = GyrMatrix(:, i*3-2:i*3)*pi/180;
        quat = [QuatMatrix(:,i*4-3) QuatMatrix(:,i*4-2) QuatMatrix(:,i*4-1) QuatMatrix(:,i*4)];
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ACCELERATIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        acc = quaternRotate(Accelerometer, quat);
        acc(:,3) = acc(:,3) - mean(acc(1:200,3));
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        temp = zeros(size(QuatMatrix(:,i*4-3)));
        quat = [QuatMatrix(:,i*4-3) QuatMatrix(:,i*4-2) QuatMatrix(:,i*4-1) QuatMatrix(:,i*4) temp];
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        acc_mag = sqrt(acc(:,1).^2 + acc(:,2).^2 + acc(:,3).^2);
        gyr_magFilt = sqrt(Gyroscope(:,1).^2 + Gyroscope(:,2).^2 + Gyroscope(:,3).^2);
        acc_threshold = 3; %(m/s/s)
        mag_threshold = 1; %(rad)
        stationary = zeros(size(gyr_magFilt));
        stationary = 1 - stationary;
        for t = 2:length(stationary)
            if acc_mag(t) > acc_threshold || gyr_magFilt(t) > mag_threshold
                stationary(t) = 0;
            end
        end
        stationaryStart = find([0; diff(stationary)] == 1);
        stationaryStart = [1; stationaryStart];
        stationaryEnd = find([0; diff(stationary)] == -1);
        label = 1;
        for ii = 1:numel(stationaryEnd)
            if stationaryEnd(ii) - stationaryStart(ii) >= 100
                quat(round((stationaryStart(ii) + stationaryEnd(ii))/2), 5) = label;
                label = label + 1;
            end
        end
        figure('Position', [9 39 900 600], 'NumberTitle', 'off', 'Name', 'stationary');
        hold on;
        plot(1:length(stationary), stationary, 'r');
        title('stationary');
        xlabel('Time (s)');
        ylabel('stationary');
        hold off;
        csvwrite(strcat('OrientationOutput\', strcat(FileName(i), 'quaternions.csv')), quat);
    end
end