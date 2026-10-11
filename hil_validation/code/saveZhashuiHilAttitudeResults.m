function saveZhashuiHilAttitudeResults(varargin)
%SAVEZHASHUIHILATTITUDERESULTS Save PX4 feedback attitude for one HIL run.

p = inputParser;
addParameter(p,'LogFile','',@(x)ischar(x)||isstring(x));
parse(p,varargin{:});
logFile = char(p.Results.LogFile);
assert(exist(logFile,'file') == 2,'Telemetry log was not found: %s',logFile);

T = readtable(logFile);
required = {'rollRad','pitchRad','yawRad','rollRateRadS','pitchRateRadS','yawRateRadS'};
assert(all(ismember(required,T.Properties.VariableNames)), ...
    'Telemetry log does not contain PX4 attitude feedback.');

fig = figure('Name','Zhashui HIL - PX4 attitude feedback','Color','w', ...
    'Position',[100 100 1280 780]);
tl = tiledlayout(2,1,'TileSpacing','compact','Padding','compact');
colors = lines(3);
styles = {'-','--',':'};
columns = {'rollRad','pitchRad','yawRad'};
nexttile; hold on; grid on; box on;
for a = 1:3
    for u = 1:3
        U = T(T.uav == u,:);
        plot(U.t,rad2deg(U.(columns{a})),'Color',colors(u,:), ...
            'LineStyle',styles{a},'LineWidth',1.05);
    end
end
xlabel('Time (s)'); ylabel('Euler angle (deg)');
title('PX4 feedback attitude');
legend({'UAV 1 roll','UAV 2 roll','UAV 3 roll','UAV 1 pitch','UAV 2 pitch','UAV 3 pitch', ...
    'UAV 1 yaw','UAV 2 yaw','UAV 3 yaw'},'Location','eastoutside');

rateColumns = {'rollRateRadS','pitchRateRadS','yawRateRadS'};
nexttile; hold on; grid on; box on;
for a = 1:3
    for u = 1:3
        U = T(T.uav == u,:);
        plot(U.t,rad2deg(U.(rateColumns{a})),'Color',colors(u,:), ...
            'LineStyle',styles{a},'LineWidth',1.05);
    end
end
xlabel('Time (s)'); ylabel('Angular rate (deg/s)');
title('PX4 feedback angular rate');
sgtitle(tl,'Zhashui three-UAV HIL: flight attitude feedback');

resultDir = fileparts(logFile);
savefig(fig,fullfile(resultDir,'attitude.fig'));
exportgraphics(fig,fullfile(resultDir,'attitude.png'),'Resolution',200);

positionFig = figure('Name','Zhashui HIL - XYZ position','Color','w', ...
    'Position',[120 120 1280 780]);
positionLayout = tiledlayout(3,1,'TileSpacing','compact','Padding','compact');
positionColumns = {'localN','localE','localD'};
positionTitles = {'X: North','Y: East','Z: Up'};
positionLabels = {'North (m)','East (m)','Up (m)'};
for axisId = 1:3
    nexttile; hold on; grid on; box on;
    for u = 1:3
        U = T(T.uav == u,:);
        value = U.(positionColumns{axisId});
        if axisId == 3
            value = -value;
        end
        plot(U.t,value,'Color',colors(u,:),'LineWidth',1.15);
    end
    xlabel('Time (s)'); ylabel(positionLabels{axisId});
    title(positionTitles{axisId});
end
legend({'UAV 1','UAV 2','UAV 3'},'Location','eastoutside');
sgtitle(positionLayout,'Zhashui three-UAV HIL: local XYZ position');
savefig(positionFig,fullfile(resultDir,'xyz_position.fig'));
exportgraphics(positionFig,fullfile(resultDir,'xyz_position.png'),'Resolution',200);
end
