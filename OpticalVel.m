function OPV = OpticalVel(OP)
    OPV = zeros(length(OP)-1,3);
    for i=2:length(OP)
        OPV(i, :) = (OP(i, :) - OP(i-1, :))/0.01;
        if abs(OPV(i, 1)) > 10000
            OPV(i, 1) = 0;
        end
        if abs(OPV(i, 2)) > 10000
            OPV(i, 2) = 0;
        end
        if abs(OPV(i, 3)) > 10000
            OPV(i, 3) = 0;
        end
    end
end