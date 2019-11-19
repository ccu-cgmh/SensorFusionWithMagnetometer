function [FileNameFirst4, Acc, Gyr, Mag] = ReadSensorRawData(Folder, label)
    %
    % ��Ƨ����Ҧ�CSV�ɪ��ɦW
    dirOutput=dir(fullfile(Folder,'*.csv'));
    FileName={dirOutput.name}';
    % Ū�Ҧ��ɮ׮ɶ��аO���Y���
    TimeStart = zeros(length(FileName), 1);
    TimeEnd = zeros(length(FileName), 1);
    for i=1:length(FileName)
        Matrix = readmatrix(append(Folder, string(FileName(i))));
        TimeStart(i) = Matrix(1,1);
        TimeEnd(i) = Matrix(length(Matrix),1);
    end
    % �إ߷s���ɶ��b�_���I
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
            % �����X�ɶ��b�_�I����I�A����10�@�����
            Acc = [Acc interp1(AccMatrix(:, 1), AccMatrix(:,4:6), TimestampStart:10:TimestampEnd)];
            
            GyrMatrix = readmatrix(append(Folder, string(FileName(i*2))));
            % �����X�ɶ��b�_�I����I�A����10�@�����
            Gyr = [Gyr interp1(GyrMatrix(:, 1), GyrMatrix(:,4:6), TimestampStart:10:TimestampEnd)];
        end
    end
end