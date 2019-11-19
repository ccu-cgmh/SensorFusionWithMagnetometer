function ShifT = SE_OP_Shift(OPVel, SEVel, ManualShift)
    OPV1D = sqrt(OPVel(:,1).^2 + OPVel(:,2).^2 + OPVel(:,3).^2);
    SEV1D = sqrt(SEVel(:,1).^2 + SEVel(:,2).^2 + SEVel(:,3).^2);
    ListOP = [OPV1D' zeros(1, length(SEV1D)-length(OPV1D))];
    OP_SE_Diff = zeros(1, length(SEV1D));
    for i = 1:length(SEV1D)
        ListSE = [SEV1D(i:length(SEV1D))' zeros(1, i-1)];
        TwoListDiff = ListSE - ListOP;
        OP_SE_Diff(i) = sum(TwoListDiff.^2);
    end
    ShifT = find(OP_SE_Diff==min(min(OP_SE_Diff))) - 1;
    ShifT = ShifT + ManualShift;
end