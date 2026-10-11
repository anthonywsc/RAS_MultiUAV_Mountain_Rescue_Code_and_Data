function results = main_UAVMission(mode)
%MAIN_UAVMISSION Unified entry point for the multi-UAV mission workflow.
%
%   results = main_UAVMission('pathdemo') % one DEMA* path demonstration
%   results = main_UAVMission('costdemo') % small directed cost-library test
%   results = main_UAVMission('costlib')  % full library for packaged Target set
%   results = main_UAVMission('initial')       % generate targets + library + terrain-aware ACO
%   results = main_UAVMission('initialstatic') % packaged targets + completed static library + ACO
%   results = main_UAVMission('acotest')       % alias of initialstatic for ACO testing
%   results = main_UAVMission('dynamic')     % dynamic simulation using packaged legacy data
%   results = main_UAVMission('dynamicaco')  % dynamic simulation using latest ACO result
%   results = main_UAVMission('commtest')    % alias of dynamicaco for communication validation
%   results = main_UAVMission('realloctest') % alias of dynamicaco for local reallocation validation
%   results = main_UAVMission('all')         % initial allocation then dynamic simulation
%
% v2.5 keeps the v2.4.1 ACO and introduces the paper-aligned terrain-dependent
% communication graph with separate direct adjacency and multi-hop reachability.

if nargin < 1 || isempty(mode)
    mode = 'dynamic';
end

root = fileparts(mfilename('fullpath'));
cfg = uavConfig(root);
if ~exist(cfg.paths.outputDir, 'dir')
    mkdir(cfg.paths.outputDir);
end
rng(cfg.randomSeed, 'twister');

assert(exist(cfg.paths.mapTxt, 'file') == 2, 'Map file not found: %s', cfg.paths.mapTxt);
map = load(cfg.paths.mapTxt);
validateMap(map, cfg);

switch lower(mode)
    case 'pathdemo'
        startXY = [316, 78];
        goalXY  = [851, 886];
        results = planTerrainPath(startXY, goalXY, map, cfg, true);

    case 'costdemo'
        data = loadPackagedTargets(cfg);
        n = min(cfg.cost.demoNodes,size(data.Target,1));
        TargetDemo = data.Target(1:n,:);
        results = buildTerrainCostLibrary(TargetDemo,map,cfg, ...
            'CheckpointFile',cfg.paths.costDemoCheckpoint, ...
            'Resume',cfg.cost.resume, ...
            'Verbose',cfg.cost.verbose, ...
            'FailFast',cfg.cost.failFast);

    case 'costlib'
        data = loadPackagedTargets(cfg);
        results = buildTerrainCostLibrary(data.Target,map,cfg, ...
            'CheckpointFile',cfg.paths.staticCostCheckpoint, ...
            'FinalFile',cfg.paths.staticCostLibrary, ...
            'Resume',cfg.cost.resume, ...
            'Verbose',cfg.cost.verbose, ...
            'FailFast',cfg.cost.failFast);

    case 'initial'
        data = initialTaskAllocation(map, cfg, true);
        save(cfg.paths.generatedData, '-struct', 'data');
        results = data;

    case {'initialstatic','acotest'}
        packaged = loadPackagedTargets(cfg);
        assert(exist(cfg.paths.staticCostLibrary,'file')==2, ...
            'Static terrain-cost library not found: %s',cfg.paths.staticCostLibrary);
        S = load(cfg.paths.staticCostLibrary,'Cost');
        assert(isfield(S,'Cost'),'Static cost-library file does not contain Cost.');
        data = initialTaskAllocation(map,cfg,true,'Target',packaged.Target,'Cost',S.Cost);
        save(cfg.paths.generatedData, '-struct', 'data');
        results = data;

    case 'dynamic'
        assert(exist(cfg.paths.staticData, 'file') == 2, ...
            'Precomputed StaticData file not found: %s', cfg.paths.staticData);
        data = load(cfg.paths.staticData);
        data.map = map; % use the packaged map explicitly
        results = simulateDynamicMission(data, cfg);

    case {'dynamicaco','commtest','realloctest'}
        assert(exist(cfg.paths.generatedData,'file')==2, ...
            ['Latest ACO result not found: %s. Run main_UAVMission(''acotest'') ', ...
             'first to generate it.'],cfg.paths.generatedData);
        data = load(cfg.paths.generatedData);
        data.map = map;
        results = simulateDynamicMission(data,cfg);

    case 'all'
        data = initialTaskAllocation(map, cfg, true);
        save(cfg.paths.generatedData, '-struct', 'data');
        results = simulateDynamicMission(data, cfg);

    otherwise
        error(['Unknown mode "%s". Use pathdemo, costdemo, costlib, ', ...
               'initial, initialstatic, acotest, dynamic, dynamicaco, ', ...
               'commtest, realloctest, or all.'], mode);
end

results = saveRunResultsIfEnabled(results, mode, cfg);
end

function data = loadPackagedTargets(cfg)
assert(exist(cfg.paths.staticData,'file')==2, ...
    'Precomputed StaticData file not found: %s',cfg.paths.staticData);
data=load(cfg.paths.staticData,'Target');
assert(isfield(data,'Target') && size(data.Target,2)>=3, ...
    'StaticData does not contain a valid Target array.');
end

function validateMap(map, cfg)
assert(isnumeric(map) && ismatrix(map) && ~isempty(map), 'DEM map must be a nonempty numeric matrix.');
assert(all(isfinite(map(:))), 'DEM map contains NaN or Inf values.');
base = cfg.uav.baseIndex;
assert(base(1) >= 1 && base(1) <= size(map,1) && base(2) >= 1 && base(2) <= size(map,2), ...
    'Configured base index lies outside DEM map.');
end

function results = saveRunResultsIfEnabled(results, mode, cfg)
if ~isfield(cfg,'results') || ~isfield(cfg.results,'autoSave') || ~cfg.results.autoSave
    return;
end
if ~isfield(cfg,'paths') || ~isfield(cfg.paths,'runResultsDir')
    return;
end

if ~exist(cfg.paths.runResultsDir,'dir')
    mkdir(cfg.paths.runResultsDir);
end

modeTag = lower(char(mode));
modeTag = regexprep(modeTag,'[^a-z0-9_]+','_');
modeTag = regexprep(modeTag,'_+','_');
modeTag = regexprep(modeTag,'^_|_$','');
if isempty(modeTag)
    modeTag = 'default';
end

runTime = datetime('now','Format','yyyyMMdd_HHmmss_SSS');
timestamp = char(runTime);
savedAtTime = runTime;
savedAtTime.Format = 'yyyy-MM-dd HH:mm:ss.SSS';
runRecord = struct();
runRecord.mode = mode;
runRecord.timestamp = timestamp;
runRecord.savedAt = char(savedAtTime);
runRecord.projectRoot = cfg.projectRoot;
runRecord.randomSeed = cfg.randomSeed;
runRecord.matlabVersion = version;
runRecord.resultClass = class(results);
runRecord.file = fullfile(cfg.paths.runResultsDir, ...
    sprintf('%s_%s.mat',timestamp,modeTag));
runRecord.latestModeFile = fullfile(cfg.paths.runResultsDir, ...
    sprintf('latest_%s.mat',modeTag));
runRecord.latestFile = fullfile(cfg.paths.runResultsDir,'latest.mat');

if isstruct(results) && isscalar(results)
    results.runRecord = runRecord;
end

save(runRecord.file,'results','cfg','mode','runRecord','-v7.3');
if isfield(cfg.results,'saveLatest') && cfg.results.saveLatest
    copyfile(runRecord.file,runRecord.latestModeFile,'f');
    copyfile(runRecord.file,runRecord.latestFile,'f');
end

fprintf('Saved run results: %s\n',runRecord.file);
end
