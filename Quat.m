function quat = Quat(AccMatrix, GyrMatrix)
    quat = [];
    %��촫��Asensor��l�����O(g)�M(deg/s)
    %���⬰(m/s/s)�M(rad/s)
    AccMatrix = AccMatrix*9.8;
    GyrMatrix = GyrMatrix*pi/180;
    [row, col] = size(AccMatrix);
    for i=1:col/3
        qimu = quaternion(zeros(size(AccMatrix, 1), 4));
        ifilt = imufilter();
        for ii=1:row
            qimu(ii) = ifilt(AccMatrix(ii,i*3-2:i*3), GyrMatrix(ii,i*3-2:i*3));
        end
        [a,b,c,d] = parts(qimu);
        Tempquat = [a b c d];
        quat = [quat Tempquat];
    end
end