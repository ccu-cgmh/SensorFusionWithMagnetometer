function [FileNameFirst4, Acc, Gyr, Mag] = ReadSensorRawData(Folder, label)
    %
    % 資料夾內所有CSV檔的檔名
    dirOutput=dir(fullfile(Folder,'*.csv'));
    FileName={dirOutput.name}';
    % 讀所有檔案時間標記的頭跟尾
    TimeStart = zeros(length(FileName), 1);
    TimeEnd = zeros(length(FileName), 1);
    for i=1:length(FileName)
        Matrix = readmatrix(append(Folder, string(FileName(i))));
        TimeStart(i) = Matrix(1,1);
        TimeEnd(i) = Matrix(length(Matrix),1);
    end
    % 建立新的時間軸起終點
    TimestampStart = max(TimeStart);
    TimestampEnd = min(TimeEnd);
    Acc = [];
    Gyr = [];
    FileNameFirst4 = [];
    if label == 0
        for i=1:length(FileName)/2
            FileNameStr = string(FileName(i*2-1));
            FileNameStr = extractBefore(FileNameStr, 5);
            FileNameFirst4 = [FileNameFirst4 FileNameStr];
            AccMatrix = readmatrix(append(Folder, string(FileName(i*2-1))));
            % 內插出時間軸起點到終點，間格10毫秒的資料
            Acc = [Acc interp1(AccMatrix(:, 1), AccMatrix(:,4:6), TimestampStart:10:TimestampEnd)];
            
            GyrMatrix = readmatrix(append(Folder, string(FileName(i*2))));
            % 內插出時間軸起點到終點，間格10毫秒的資料
            Gyr = [Gyr interp1(GyrMatrix(:, 1), GyrMatrix(:,4:6), TimestampStart:10:TimestampEnd)];
        end
    end
end