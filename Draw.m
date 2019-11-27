function fig = Draw(OP, OPV, SE, SEV, MD, MotionStart, MotionEnd)
    LineListOP = [];
    PathListOP = [];
    LineListSE = [];
    PathListSE = [];
    LineListMD = [];
    PathListMD = [];
    for i=1:length(MotionStart)
        st = MotionStart(i);
        ed = MotionEnd(i);
        disp(['²Ä' num2str(i) '®±']);
        disp(['«ùÄò®É¶¡: ' num2str((ed - st)*0.01) ' (s)']);
        LineOP = sqrt((OP(ed, 1) - OP(st, 1))^2 + (OP(ed, 2) - OP(st, 2))^2 + (OP(ed, 3) - OP(st, 3))^2);
        disp(['(¥ú¾Ç)ª½½u¶ZÂ÷: ' num2str(LineOP) ' (mm)']);
        LineListOP = [LineListOP LineOP]; %mm
        LineSE = sqrt((SE(ed, 1) - SE(st, 1))^2 + (SE(ed, 2) - SE(st, 2))^2 + (SE(ed, 3) - SE(st, 3))^2);
        disp(['(Sensor)ª½½u¶ZÂ÷: ' num2str(LineSE) ' (mm)']);
        LineListSE = [LineListSE LineSE]; %mm
        LineMD = sqrt((MD(ed, 1) - MD(st, 1))^2 + (MD(ed, 2) - MD(st, 2))^2 + (MD(ed, 3) - MD(st, 3))^2);
        disp(['(¼Ò«¬)ª½½u¶ZÂ÷: ' num2str(LineMD) ' (mm)']);
        LineListMD = [LineListMD LineMD]; %mm
        disp(['(Sensor)ª½½u¶ZÂ÷ »~®t: ' num2str((LineSE - LineOP)/LineOP*100) ' (%)']);
        disp([' (¼Ò«¬) ª½½u¶ZÂ÷ »~®t: ' num2str((LineMD - LineOP)/LineOP*100) ' (%)']);
        PathOP = 0;
        for ii = st:ed
            PathOP = PathOP + sqrt((OP(ii,1) - OP(ii+1,1)).^2 + (OP(ii,2) - OP(ii+1,2)).^2 + (OP(ii,3) - OP(ii+1,3)).^2);
        end
        disp(['(¥ú¾Ç)­y¸ñ¶ZÂ÷: ' num2str(PathOP) ' (mm)']);
        PathListOP = [PathListOP PathOP]; %mm
        PathSE = 0;
        for ii = st:ed
            PathSE = PathSE + sqrt((SE(ii,1) - SE(ii+1,1)).^2 + (SE(ii,2) - SE(ii+1,2)).^2 + (SE(ii,3) - SE(ii+1,3)).^2);
        end
        disp(['(Sensor)­y¸ñ¶ZÂ÷: ' num2str(PathSE) ' (mm)']);
        PathListSE = [PathListSE PathSE]; %mm
        PathMD = 0;
        for ii = st:ed
            PathMD = PathMD + sqrt((MD(ii,1) - MD(ii+1,1)).^2 + (MD(ii,2) - MD(ii+1,2)).^2 + (MD(ii,3) - MD(ii+1,3)).^2);
        end
        disp(['(¼Ò«¬)­y¸ñ¶ZÂ÷: ' num2str(PathMD) ' (mm)']);
        PathListMD = [PathListMD PathMD]; %mm
        disp(['(Sensor)­y¸ñ¶ZÂ÷ »~®t: ' num2str((PathSE - PathOP)/PathOP*100) ' (%)']);
        disp([' (¼Ò«¬) ­y¸ñ¶ZÂ÷ »~®t: ' num2str((PathMD - PathOP)/PathOP*100) ' (%)']);
        
        fig = figure('NumberTitle', 'off', 'Name', '6DOF Animation');
        title(['²Ä' num2str(i) '®±']);
        screenSize = get(0, 'ScreenSize');
        set(fig, 'Position', [0 0 screenSize(3) screenSize(4)]);
        hold on;
        axis equal;
        View = [30 20];
        view(View(1, :));
        xlabel('X (m)');
        ylabel('Y (m)');
        zlabel('Z (m)');
        if LineOP == 0 || LineSE == 0 || LineMD == 0
            continue;
        end
        %%%%%Âà§¤¼Ð¶b OP
        YShift = LineOP;
        V1 = [0, YShift, 0];
        V2 = [OP(ed, 1)-OP(st, 1), OP(ed, 2)-OP(st, 2), OP(ed, 3)-OP(st, 3)];
        R = vrrotvec(V1, V2);
        M = vrrotvec2mat(R);
        OPTemp = OP * M;
%         OPTemp = OP;
        %%%%%Âà§¤¼Ð¶b SE
        YShift = LineSE;
        V1 = [0, YShift, 0];
        V2 = [SE(ed, 1)-SE(st, 1), SE(ed, 2)-SE(st, 2), SE(ed, 3)-SE(st, 3)];
        R = vrrotvec(V1, V2);
        M = vrrotvec2mat(R);
        SETemp = SE * M;
%         SETemp = SE;
        %%%%%Âà§¤¼Ð¶b MD
        YShift = LineMD;
        V1 = [0, YShift, 0];
        V2 = [MD(ed, 1)-MD(st, 1), MD(ed, 2)-MD(st, 2), MD(ed, 3)-MD(st, 3)];
        R = vrrotvec(V1, V2);
        M = vrrotvec2mat(R);
        MDTemp = MD * M;
%         MDTemp = MD;
        %
        SEPath = SETemp(st:ed, :)-SETemp(st,:);
        MDPath = MDTemp(st:ed, :)-MDTemp(st,:);
        EndDrift = SEPath(length(SEPath),:) - MDPath(length(MDPath),:);
        driftRate = EndDrift / (ed - st);
        enum = 1:(ed - st)+1;
        drift = [enum'*driftRate(1) enum'*driftRate(2) enum'*driftRate(3)];
        SEPath = SEPath - drift;
        SENewPath = zeros(size(MDTemp));
        SENewPath(st:ed,:) = SEPath;
        PathSE = 0;
        TempV1DList = [];
        for ii = 1:length(SEPath)-1
            TempV1D = sqrt((SEPath(ii,1) - SEPath(ii+1,1)).^2 + (SEPath(ii,2) - SEPath(ii+1,2)).^2 + (SEPath(ii,3) - SEPath(ii+1,3)).^2);
            TempV1DList = [TempV1DList TempV1D];
            PathSE = PathSE + TempV1D;
        end
        
        disp(['NEW(Sensor)­y¸ñ¶ZÂ÷: ' num2str(PathSE) ' (mm)']);
        disp(['NEW(Sensor)­y¸ñ¶ZÂ÷ »~®t: ' num2str((PathSE - PathOP)/PathOP*100) ' (%)']);
        
        OPVMAX = max(OPV(st:ed))*1000;
        disp(['(¥ú¾Ç)§À«l: ' num2str(OPVMAX) ' (mm/s)']);
        SEVMAX = max(SEV(st:ed))*1000;
        disp(['(Sensor)§À«l: ' num2str(SEVMAX) ' (mm/s)']);
        disp(['(Sensor)§À«l »~®t: ' num2str((SEVMAX-OPVMAX)/OPVMAX*100) ' (%)']);
        TempVMAX = max(TempV1DList)*100;
        disp(['NEW(Sensor)§À«l: ' num2str(TempVMAX) ' (mm/s)']);
        disp(['NEW(Sensor)§À«l »~®t: ' num2str((TempVMAX-OPVMAX)/OPVMAX*100) ' (%)']);
        
        
        disp(['(Sensor)³t«×: ' num2str(PathSE/((ed - st)*0.01)) ' (mm/s)']);
        
        
        for t = st:ed
            OPx = OPTemp(st:t,1) - OPTemp(st,1);
            OPy = OPTemp(st:t,2) - OPTemp(st,2);
            OPz = OPTemp(st:t,3) - OPTemp(st,3);
            orgHandle = plot3(OPx, OPy, OPz, '-r');
            set(orgHandle, 'xdata', OPx, 'ydata', OPy, 'zdata', OPz);
            SEx = SETemp(st:t,1) - SETemp(st,1);
            SEy = SETemp(st:t,2) - SETemp(st,2);
            SEz = SETemp(st:t,3) - SETemp(st,3);
            orgHandle = plot3(SEx, SEy, SEz, '-g');
            set(orgHandle, 'xdata', SEx, 'ydata', SEy, 'zdata', SEz);
            MDx = MDTemp(st:t,1) - MDTemp(st,1);
            MDy = MDTemp(st:t,2) - MDTemp(st,2);
            MDz = MDTemp(st:t,3) - MDTemp(st,3);
            orgHandle = plot3(MDx, MDy, MDz, '-b');
            set(orgHandle, 'xdata', MDx, 'ydata', MDy, 'zdata', MDz);
            SENEWx = SENewPath(st:t,1) - SENewPath(st,1);
            SENEWy = SENewPath(st:t,2) - SENewPath(st,2);
            SENEWz = SENewPath(st:t,3) - SENewPath(st,3);
            orgHandle = plot3(SENEWx, SENEWy, SENEWz, '-.k');
            set(orgHandle, 'xdata', SENEWx, 'ydata', SENEWy, 'zdata', SENEWz);
            drawnow;
        end
        hold off;
    end
end