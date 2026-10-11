function mission = exportZhashuiSitlMission(varargin)
%EXPORTZHASHUISITLMISSION Export the latest Zhashui mission for PX4 SITL.
% The mission geometry stays tied to the real DEM.  Horizontal and vertical
% scales only define the physical size used by the local CopterSim instance;
% the MATLAB viewer maps reported NED states back onto the unscaled DEM.

p = inputParser;
packageRoot = fileparts(fileparts(mfilename('fullpath')));
addParameter(p,'ResultFile',fullfile(packageRoot,'assets','mission','mission_run.mat'),@(x)ischar(x)||isstring(x));
addParameter(p,'OutputFile',fullfile(packageRoot,'results','mission.json'),@(x)ischar(x)||isstring(x));
addParameter(p,'HorizontalScale',0.25,@(x)isnumeric(x)&&isscalar(x)&&x>0);
addParameter(p,'VerticalScale',0.25,@(x)isnumeric(x)&&isscalar(x)&&x>0);
addParameter(p,'WaypointSpacingM',35,@(x)isnumeric(x)&&isscalar(x)&&x>0);
parse(p,varargin{:});
opt = p.Results;

terrainFile = fullfile(packageRoot,'assets','terrain','zhashui.mat');
assert(exist(opt.ResultFile,'file') == 2,'Missing mission result: %s',opt.ResultFile);
assert(exist(terrainFile,'file') == 2,'Missing Zhashui terrain: %s',terrainFile);
S = load(opt.ResultFile,'run');
D = load(terrainFile,'map','X','Y');
R = S.run.results;
assert(numel(R.trajectories) == 3,'This SITL experiment requires exactly three UAVs.');

base = R.Target(1,1:3); % [row col MSL altitude]
resolutionM = mean(diff(D.X(1,:)));
routes = cell(1,3);
for u = 1:3
    raw = R.trajectories{u}; % [row col MSL altitude]
    raw = resampleRoute(raw,resolutionM,opt.WaypointSpacingM);
    ned = [(raw(:,1)-base(1))*resolutionM*opt.HorizontalScale, ...
           (raw(:,2)-base(2))*resolutionM*opt.HorizontalScale, ...
           -(raw(:,3)-base(3))*opt.VerticalScale];
    routes{u} = struct('uavId',u,'taskSequence',R.sequences{u}, ...
        'waypointsNED',ned,'mapWaypoints',raw);
end

mission = struct('schemaVersion',2,'terrainName','Zhashui DEM / Bing Aerial', ...
    'sourceResultFile',char(opt.ResultFile),'physicalSITL',true, ...
    'horizontalScale',opt.HorizontalScale,'verticalScale',opt.VerticalScale, ...
    'resolutionM',resolutionM,'baseRowColMsl',base,'routes',{routes});
outDir = fileparts(opt.OutputFile);
if ~exist(outDir,'dir'), mkdir(outDir); end
fid = fopen(opt.OutputFile,'w');
assert(fid >= 0,'Cannot write: %s',opt.OutputFile);
cleaner = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s\n',jsonencode(mission));
fprintf('Exported Zhashui three-UAV PX4 SITL mission to %s\n',opt.OutputFile);
end

function out = resampleRoute(path,resolutionM,spacingM)
keep = [true; sqrt(sum(diff(path(:,1:2),1,1).^2,2))*resolutionM > 0.1];
path = path(keep,:);
if size(path,1) < 2
    out = path;
    return;
end
d = [0;cumsum(sqrt(sum(diff(path(:,1:2),1,1).^2,2))*resolutionM)];
sampleD = (0:spacingM:d(end)).';
if sampleD(end) < d(end), sampleD(end+1,1) = d(end); end
out = interp1(d,path,sampleD,'linear');
end
