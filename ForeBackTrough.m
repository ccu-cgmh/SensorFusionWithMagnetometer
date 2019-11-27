function [st,ed] = ForeBackTrough(Signal, PeakLoc, L)
    st = [];
    ed = [];
    for i=1:length(PeakLoc)
        % Fore Trough
        if L == 0 && i == 1
            [minV, minI] = min(Signal(PeakLoc(i)-40:PeakLoc(i)));
            st = [st PeakLoc(i)-39+minI];
        end
        if L == 0 && i ~= 1
            [minV, minI] = min(Signal(PeakLoc(i-1):PeakLoc(i)));
            st = [st PeakLoc(i-1)-1+minI];
        end
        if L == -1
            for t=1:PeakLoc(i)
                if Signal(PeakLoc(i)-t) < Signal(PeakLoc(i)-t-1) || Signal(PeakLoc(i)-t) == 0
                    st = [st PeakLoc(i)-t];
                    break;
                end
            end
        end
        
        % back Trough
        if L == 0 && i == length(PeakLoc)
            [minV, minI] = min(Signal(PeakLoc(i):PeakLoc(i)+40));
            ed = [ed PeakLoc(i)-1+minI];
        end
        if L == 0 && i ~= length(PeakLoc)
            [minV, minI] = min(Signal(PeakLoc(i):PeakLoc(i+1)));
            ed = [ed PeakLoc(i)-1+minI];
        end
        if L == -1
            for t=PeakLoc(i):length(Signal)
                if L == -1 && (Signal(t) < Signal(t+1) || Signal(t) == 0)
                    ed = [ed t];
                    break;
                end
                if L == 0 && Signal(t) == 0
                    ed = [ed t];
                    break;
                end
            end
        end
    end
end