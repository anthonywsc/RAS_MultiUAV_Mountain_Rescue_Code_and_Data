function runZhashuiSitlViewer(varargin)
%RUNZHASHUISITLVIEWER Live MATLAB visualization for the PX4 SITL telemetry.

p = inputParser;
packageRoot = fileparts(fileparts(mfilename('fullpath')));
addParameter(p,'LogFile',fullfile(packageRoot,'results','tracking.csv'),@(x)ischar(x)||isstring(x));
addParameter(p,'SummaryFile',fullfile(packageRoot,'results','summary.json'),@(x)ischar(x)||isstring(x));
addParameter(p,'HorizontalScale',0.25,@(x)isnumeric(x)&&isscalar(x)&&x>0);
addParameter(p,'VerticalScale',0.25,@(x)isnumeric(x)&&isscalar(x)&&x>0);
addParameter(p,'PollPeriodS',0.20,@(x)isnumeric(x)&&isscalar(x)&&x>0);
parse(p,varargin{:});
opt = p.Results;

terrainDir = fullfile(packageRoot,'assets','terrain');
D = load(fullfile(terrainDir,'zhashui.mat'),'map','X','Y');
S = load(fullfile(packageRoot,'assets','mission','mission_run.mat'),'run');
R = S.run.results;
texture = flipud(imresize(imread(fullfile(terrainDir,'z_bing.jpg')),size(D.map),'bilinear'));
base = R.Target(1,1:3);

fig = figure('Name','Zhashui PX4 SITL - live telemetry','Color','w','Position',[60 60 1320 880]);
h = surf(D.X,D.Y,D.map,texture,'EdgeColor','none');
set(h,'FaceColor','texturemap','CDataMapping','direct'); hold on;
colors = lines(3);
plot3(D.X(base(1),base(2)),D.Y(base(1),base(2)),base(3),'kp', ...
    'MarkerFaceColor','y','MarkerSize',12);
targetX = interp1(1:size(D.map,2),D.X(1,:),R.Target(2:end,2));
targetY = interp1(1:size(D.map,1),D.Y(:,1),R.Target(2:end,1));
targetHandle = plot3(targetX,targetY,R.Target(2:end,3),'ro', ...
    'MarkerFaceColor',[0.95 0.2 0.2],'MarkerSize',7,'LineWidth',1.0);
paths = gobjects(3,1); vehicles = gobjects(3,1);
for u = 1:3
    paths(u) = plot3(nan,nan,nan,'Color',colors(u,:),'LineWidth',2.2);
    vehicles(u) = plot3(nan,nan,nan,'o','Color',colors(u,:), ...
        'MarkerFaceColor',colors(u,:),'MarkerSize',8);
end
xlabel('East (m)'); ylabel('North (m)'); zlabel('Elevation (m)');
title('Zhashui three-UAV PX4 SITL: live position and wind-disturbed tracks');
grid on; box on; axis tight; view(-48,31); daspect([1 1 1/3]); axis vis3d;
legend([targetHandle;paths;vehicles],{'Tasks','UAV 1 track','UAV 2 track', ...
    'UAV 3 track','UAV 1','UAV 2','UAV 3'},'Location','eastoutside');
drawnow;

lastRows = 0;
while isvalid(fig)
    if exist(opt.LogFile,'file') == 2
        try
            T = readtable(opt.LogFile);
            if height(T) > lastRows
                lastRows = height(T);
                for u = 1:3
                    U = T(T.uav == u,:);
                    if isempty(U), continue; end
                    northM = U.localN / opt.HorizontalScale;
                    eastM = U.localE / opt.HorizontalScale;
                    msl = base(3) - U.localD / opt.VerticalScale;
                    baseX = D.X(base(1),base(2));
                    baseY = D.Y(base(1),base(2));
                    set(paths(u),'XData',baseX+eastM, ...
                        'YData',baseY+northM,'ZData',msl);
                    set(vehicles(u),'XData',baseX+eastM(end), ...
                        'YData',baseY+northM(end),'ZData',msl(end));
                end
                title(sprintf('Zhashui PX4 SITL: %.1f s, %d telemetry samples',T.t(end),height(T)));
                drawnow limitrate;
            end
        catch
            % The logger can be between CSV writes; read it again next poll.
        end
    end
    if exist(opt.SummaryFile,'file') == 2
        break;
    end
    pause(opt.PollPeriodS);
end
if isvalid(fig)
    resultDir = fileparts(char(opt.LogFile));
    savefig(fig,fullfile(resultDir,'trajectory_3d.fig'));
    exportgraphics(fig,fullfile(resultDir,'trajectory_3d.png'),'Resolution',200);
end
end
