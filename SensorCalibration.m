clear;
close all;

Folder=fullfile('Data\1118SeCal');
dirOutput=dir(fullfile(Folder,'*.csv'));
FileNames={dirOutput.name}';
for i=1:length(FileNames)/2
    SensorPos = string(FileNames(i*2));
    SensorPos = extractBefore(SensorPos,5);
    % Calibrate Acc
    AccCalMatrix = readmatrix(string(Folder)+'/'+string(FileNames(i*2-1)));
    AccCal = AccCalMatrix(:, 4:6);
    AccCal1D = sqrt(AccCal(:,1).^2 + AccCal(:,2).^2 + AccCal(:,3).^2);
    AccCalDiff = abs(diff(AccCal1D));
    NewAccCal = [];
    st = 1;
    for ii = 1:length(AccCalDiff)
        if AccCalDiff(ii) > 0.01
            if ii - st > 100
                NewAccCal = [NewAccCal;AccCal(st+10:ii-10,:)];
            end
            st = ii;
        end
    end
    NewAccCal = [NewAccCal;AccCal(st+5:ii-5,:)];
    NewAccCal1D = sqrt(NewAccCal(:,1).^2 + NewAccCal(:,2).^2 + NewAccCal(:,3).^2);
%     figure('Position', [9 39 900 600], 'NumberTitle', 'off', 'Name', 'AccCal1D');
%     hold on;
% %     plot(1:length(AccCal1D), AccCal1D, 'r');
% %     plot(1:length(AccCalDiff), AccCalDiff, 'b');
%     plot(1:length(NewAccCal1D), NewAccCal1D, 'k');
%     title('AccCal1D');
%     xlabel('Time');
%     ylabel('Acc(g)');
%     hold off;
%     scatter3(NewAccCal(:,1),NewAccCal(:,2),NewAccCal(:,3));
%     axis equal;
    [AbestAcc,bbestAcc,mfsbest] = magcal(NewAccCal,'auto');
    AccCorrected = (NewAccCal-bbestAcc)*AbestAcc;
    save(string(Folder) + '\' + string(SensorPos)+'.mat', 'AbestAcc', 'bbestAcc');
    
%     scatter3(AccCorrected(:,1),AccCorrected(:,2),AccCorrected(:,3));
%     axis equal;
    
    AccCorrected1D = sqrt(AccCorrected(:,1).^2 + AccCorrected(:,2).^2 + AccCorrected(:,3).^2);
    figure('Position', [9 39 900 600], 'NumberTitle', 'off', 'Name', 'AccCal1D');
    hold on;
    plot(1:length(NewAccCal1D), NewAccCal1D, 'k');
    plot(1:length(AccCorrected1D), AccCorrected1D, 'r');
    title('AccCal1D');
    xlabel('Time');
    ylabel('Acc(g)');
    hold off;
    
    
    
% % %     
% % %     % Calibrate Gyr
% % %     GyrCalMatrix = readmatrix(string(Folder)+'/'+string(FileNames(i*2)));
% % %     GyrCal = GyrCalMatrix(:, 4:6);
% % %     GyrCal1D = sqrt(GyrCal(:,1).^2 + GyrCal(:,2).^2 + GyrCal(:,3).^2);
% % %     GyrCalDiff = abs(diff(GyrCal1D));
% % %     NewGyrCal = [];
% % %     st = 1;
% % %     for ii = 1:length(GyrCalDiff)
% % %         if GyrCalDiff(ii) > 0.5
% % %             if ii - st > 100
% % %                 NewGyrCal = [NewGyrCal;GyrCal(st+5:ii-5,:)];
% % %             end
% % %             st = ii;
% % %         end
% % %     end
% % %     NewGyrCal = [NewGyrCal;GyrCal(st+5:ii-5,:)];
% % %     NewGyrCal1D = sqrt(NewGyrCal(:,1).^2 + NewGyrCal(:,2).^2 + NewGyrCal(:,3).^2);
% % %     figure('Position', [9 39 900 600], 'NumberTitle', 'off', 'Name', 'GyrCal1D');
% % %     hold on;
% % % %     plot(1:length(GyrCal1D), GyrCal1D, 'r');
% % % %     plot(1:length(GyrCalDiff), GyrCalDiff, 'b');
% % % %     plot(1:length(NewGyrCal1D), NewGyrCal1D, 'k');
% % %     plot(1:length(NewGyrCal(:,1)), NewGyrCal(:,1), 'r');
% % %     plot(1:length(NewGyrCal(:,2)), NewGyrCal(:,2), 'g');
% % %     plot(1:length(NewGyrCal(:,3)), NewGyrCal(:,3), 'b');
% % %     title('GyrCal1D');
% % %     xlabel('Time (s)');
% % %     ylabel('Gyr(deg/s)');
% % %     hold off;
    
end
