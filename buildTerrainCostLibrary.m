function Cost = buildTerrainCostLibrary(Target, map, cfg, varargin)
%BUILDTERRAINCOSTLIBRARY Build/resume the directed terrain-cost library.
%
%   Cost = buildTerrainCostLibrary(Target,map,cfg)
%
%   Cost = buildTerrainCostLibrary(...,'CheckpointFile',file,...
%       'FinalFile',file,'Resume',true,'ExistingCost',Cost0)
%
% The routine evaluates every ORDERED task-node pair (j,k), j~=k, with
% planTerrainPath.m and stores the paper quantities
%
%       P_jk : Cost.Paths{j,k}
%       D_jk : Cost.D(j,k)       [m]
%       E_jk : Cost.E(j,k)       [J]
%       T_jk : Cost.T(j,k)       [s]
%
% together with diagnostic path/search metrics.  The library is directed
% by default because the DEM-informed altitude assignment is history- and
% direction-dependent; therefore P_jk need not equal reverse(P_kj).
%
% Target must contain at least [row col z] in its first three columns.
% Extra columns (demand, deadline, etc.) are ignored by this layer.
%
% Incremental update:
%   If one or more targets are appended during mission execution, pass the
%   previous Cost through 'ExistingCost'. The matrices are expanded and
%   only missing ordered pairs are computed. This provides the later
%   online row/column update required by the dynamic reallocation module.

if size(Target,2) < 3
    error('buildTerrainCostLibrary:InvalidTarget', ...
        'Target must have at least three columns [row col z].');
end
validateattributes(Target(:,1:3),{'numeric'},{'2d','real','finite'});
validateattributes(map,{'numeric'},{'2d','nonempty','real','finite'});

p = inputParser;
p.FunctionName = mfilename;
addParameter(p,'CheckpointFile','',@(x)ischar(x) || (isstring(x) && isscalar(x)));
addParameter(p,'FinalFile','',@(x)ischar(x) || (isstring(x) && isscalar(x)));
addParameter(p,'Resume',true,@(x)islogical(x) || isnumeric(x));
addParameter(p,'ExistingCost',[],@(x)isstruct(x) || isempty(x));
addParameter(p,'Verbose',cfg.plot.verbose,@(x)islogical(x) || isnumeric(x));
addParameter(p,'FailFast',true,@(x)islogical(x) || isnumeric(x));
parse(p,varargin{:});
opt = p.Results;
opt.CheckpointFile = char(opt.CheckpointFile);
opt.FinalFile = char(opt.FinalFile);
opt.Resume = logical(opt.Resume);
opt.Verbose = logical(opt.Verbose);
opt.FailFast = logical(opt.FailFast);

n = size(Target,1);
positions = Target(:,1:3);
flightNodes = resolveFlightNodes(positions,map,cfg);
signature = makeSignature(positions,map,cfg);

% Priority: explicitly supplied in-memory library > compatible checkpoint
% > fresh library.
if ~isempty(opt.ExistingCost)
    Cost = expandExistingCost(opt.ExistingCost, positions, flightNodes, map, cfg);
elseif opt.Resume && ~isempty(opt.CheckpointFile) && exist(opt.CheckpointFile,'file') == 2
    S = load(opt.CheckpointFile,'Cost','checkpointSignature');
    if isfield(S,'Cost') && isfield(S,'checkpointSignature') && ...
            isequaln(S.checkpointSignature,signature)
        Cost = S.Cost;
        Cost = ensureCurrentSchema(Cost,positions,flightNodes,map,cfg);
        if opt.Verbose
            fprintf('Resuming terrain-cost library from checkpoint:\n  %s\n',opt.CheckpointFile);
        end
    else
        warning('buildTerrainCostLibrary:CheckpointMismatch', ...
            'Checkpoint does not match the current targets/map/path settings; rebuilding it.');
        Cost = initializeCost(positions,flightNodes,map,cfg);
    end
else
    Cost = initializeCost(positions,flightNodes,map,cfg);
end

Cost.Signature = signature;
Cost.LastBuildStarted = datestr(now,30);
runTimer = tic;
completedAtStart = countCompletedPairs(Cost.Success);
computedSinceCheckpoint = 0;

if opt.Verbose
    totalPairs = n*(n-1);
    donePairs = countCompletedPairs(Cost.Success);
    fprintf('\nTerrain-cost library: %d nodes, %d directed pairs.\n',n,totalPairs);
    fprintf('Already available: %d/%d directed pairs.\n',donePairs,totalPairs);
end

for j = 1:n
    for k = 1:n
        if j == k || Cost.Success(j,k)
            continue;
        end

        pairTimer = tic;
        pairNumber = countCompletedPairs(Cost.Success) + 1;
        totalPairs = n*(n-1);
        if opt.Verbose
            fprintf('[%d/%d] planning %d -> %d ... ', ...
                pairNumber,totalPairs,j-1,k-1);
        end

        try
            % Pass the full task-node position. planTerrainPath projects an
            % unsafe endpoint altitude into the DEM clearance corridor.
            R = planTerrainPath(positions(j,:),positions(k,:),map,cfg,false);

            Cost.D(j,k) = R.D;
            Cost.E(j,k) = R.E;
            Cost.T(j,k) = R.D/cfg.uav.speed;
            Cost.Paths{j,k} = R.path;
            Cost.Success(j,k) = logical(R.success);
            Cost.Failed(j,k) = ~logical(R.success);
            Cost.FailureMessage{j,k} = '';

            m = R.metrics;
            Cost.HorizontalDistance(j,k) = m.horizontalLength;
            Cost.VerticalDistance(j,k) = m.verticalDistance;
            Cost.ClimbDistance(j,k) = m.climbDistance;
            Cost.DescentDistance(j,k) = m.descentDistance;
            Cost.MeanAbsSlopeDeg(j,k) = m.meanAbsSlopeDeg;
            Cost.MeanClimbAngleDeg(j,k) = m.meanClimbAngleDeg;
            Cost.MaxAbsSlopeDeg(j,k) = m.maxAbsSlopeDeg;
            Cost.NumWaypoints(j,k) = m.numWaypoints;

            s = R.searchInfo;
            Cost.ExpandedNodes(j,k) = s.expandedNodes;
            Cost.GeneratedNodes(j,k) = s.generatedNodes;
            Cost.ReopenedNodes(j,k) = s.reopenedNodes;
            if isfield(s,'queuePushes'), Cost.QueuePushes(j,k) = s.queuePushes; end
            if isfield(s,'iterations'),  Cost.SearchIterations(j,k) = s.iterations; end
            Cost.SearchDistance(j,k) = s.finalSearchDistance;
            Cost.SearchScore(j,k) = s.finalSearchScore;
            Cost.PairRuntime(j,k) = R.runtime;

            if opt.Verbose
                fprintf('D=%.3f km, E=%.1f kJ, %.2f s\n', ...
                    R.D/1000,R.E/1000,toc(pairTimer));
            end
        catch ME
            Cost.Success(j,k) = false;
            Cost.Failed(j,k) = true;
            Cost.FailureMessage{j,k} = ME.message;
            Cost.PairRuntime(j,k) = toc(pairTimer);
            if opt.Verbose
                fprintf('FAILED (%.2f s): %s\n',Cost.PairRuntime(j,k),ME.message);
            end
            saveCheckpointIfRequested(Cost,signature,opt.CheckpointFile);
            if opt.FailFast
                rethrow(ME);
            end
        end

        computedSinceCheckpoint = computedSinceCheckpoint + 1;
        if computedSinceCheckpoint >= cfg.cost.checkpointEvery
            saveCheckpointIfRequested(Cost,signature,opt.CheckpointFile);
            computedSinceCheckpoint = 0;
        end
    end
end

Cost.LastBuildRuntime = toc(runTimer);
Cost.BuildRuntimeAccumulated = Cost.BuildRuntimeAccumulated + Cost.LastBuildRuntime;
Cost.LastBuildFinished = datestr(now,30);
Cost.CompletedDirectedPairs = countCompletedPairs(Cost.Success);
Cost.TotalDirectedPairs = n*(n-1);
Cost.NewPairsThisRun = Cost.CompletedDirectedPairs - completedAtStart;
Cost.Summary = summarizeCost(Cost);

saveCheckpointIfRequested(Cost,signature,opt.CheckpointFile);
if ~isempty(opt.FinalFile)
    ensureParentDirectory(opt.FinalFile);
    save(opt.FinalFile,'Cost','-v7.3');
end

if opt.Verbose
    fprintf('Terrain-cost library complete: %d/%d pairs, %d failed.\n', ...
        Cost.CompletedDirectedPairs,Cost.TotalDirectedPairs,nnz(Cost.Failed));
    fprintf('This run: %.1f s; accumulated build time: %.1f s.\n\n', ...
        Cost.LastBuildRuntime,Cost.BuildRuntimeAccumulated);
end
end

%% ------------------------------------------------------------------------
function Cost = initializeCost(positions,flightNodes,map,cfg)
n = size(positions,1);
Cost = struct();
Cost.Version = '2.3';
Cost.IsDirected = true;
Cost.TargetPosition = positions;
Cost.FlightNodePosition = flightNodes;
Cost.MapSize = size(map);
Cost.MapResolution = cfg.map.resolution;
Cost.NodeCount = n;

Cost.D = nan(n);                     % final 3-D terrain path [m]
Cost.E = nan(n);                     % paper energy proxy [J]
Cost.T = nan(n);                     % travel time [s]
Cost.DirectDistance = nan(n);        % straight 3-D distance [m]
Cost.HorizontalDirectDistance = nan(n);
Cost.AltitudeDifference = nan(n);    % z_k-z_j [m]
Cost.Paths = cell(n);

Cost.HorizontalDistance = nan(n);
Cost.VerticalDistance = nan(n);
Cost.ClimbDistance = nan(n);
Cost.DescentDistance = nan(n);
Cost.MeanAbsSlopeDeg = nan(n);
Cost.MeanClimbAngleDeg = nan(n);
Cost.MaxAbsSlopeDeg = nan(n);
Cost.NumWaypoints = nan(n);

Cost.PairRuntime = nan(n);
Cost.ExpandedNodes = nan(n);
Cost.GeneratedNodes = nan(n);
Cost.ReopenedNodes = nan(n);
Cost.QueuePushes = nan(n);
Cost.SearchIterations = nan(n);
Cost.SearchDistance = nan(n);
Cost.SearchScore = nan(n);

Cost.Success = false(n);
Cost.Failed = false(n);
Cost.FailureMessage = cell(n);
Cost.BuildRuntimeAccumulated = 0;
Cost.LastBuildRuntime = 0;
Cost.CompletedDirectedPairs = 0;
Cost.TotalDirectedPairs = n*(n-1);
Cost.NewPairsThisRun = 0;
Cost.LastBuildStarted = '';
Cost.LastBuildFinished = '';

for j = 1:n
    Cost.D(j,j)=0; Cost.E(j,j)=0; Cost.T(j,j)=0;
    Cost.DirectDistance(j,j)=0;
    Cost.HorizontalDirectDistance(j,j)=0;
    Cost.AltitudeDifference(j,j)=0;
    Cost.HorizontalDistance(j,j)=0;
    Cost.VerticalDistance(j,j)=0;
    Cost.ClimbDistance(j,j)=0;
    Cost.DescentDistance(j,j)=0;
    Cost.MeanAbsSlopeDeg(j,j)=0;
    Cost.MeanClimbAngleDeg(j,j)=0;
    Cost.MaxAbsSlopeDeg(j,j)=0;
    Cost.NumWaypoints(j,j)=1;
    Cost.PairRuntime(j,j)=0;
    Cost.ExpandedNodes(j,j)=0;
    Cost.GeneratedNodes(j,j)=0;
    Cost.ReopenedNodes(j,j)=0;
    Cost.QueuePushes(j,j)=0;
    Cost.SearchIterations(j,j)=0;
    Cost.SearchDistance(j,j)=0;
    Cost.SearchScore(j,j)=0;
    Cost.Paths{j,j}=flightNodes(j,:);
    Cost.Success(j,j)=true;
end

for j=1:n
    for k=1:n
        if j==k, continue; end
        dr = (flightNodes(k,1)-flightNodes(j,1))*cfg.map.resolution;
        dc = (flightNodes(k,2)-flightNodes(j,2))*cfg.map.resolution;
        dz = flightNodes(k,3)-flightNodes(j,3);
        Cost.HorizontalDirectDistance(j,k)=hypot(dr,dc);
        Cost.DirectDistance(j,k)=hypot(Cost.HorizontalDirectDistance(j,k),dz);
        Cost.AltitudeDifference(j,k)=dz;
    end
end
end

function Cost = expandExistingCost(old,positions,flightNodes,map,cfg)
if ~isfield(old,'TargetPosition')
    error('buildTerrainCostLibrary:ExistingCostInvalid', ...
        'ExistingCost does not contain TargetPosition.');
end
nOld = size(old.TargetPosition,1);
nNew = size(positions,1);
if nNew < nOld || ~isequaln(old.TargetPosition,positions(1:nOld,:))
    error('buildTerrainCostLibrary:ExistingCostMismatch', ...
        'ExistingCost targets must be an unchanged prefix of the current Target array.');
end
fresh = initializeCost(positions,flightNodes,map,cfg);
if nOld == 0
    Cost = fresh;
    return;
end

matrixFields = {'D','E','T','DirectDistance','HorizontalDirectDistance', ...
    'AltitudeDifference','HorizontalDistance','VerticalDistance', ...
    'ClimbDistance','DescentDistance','MeanAbsSlopeDeg', ...
    'MeanClimbAngleDeg','MaxAbsSlopeDeg','NumWaypoints','PairRuntime', ...
    'ExpandedNodes','GeneratedNodes','ReopenedNodes','QueuePushes', ...
    'SearchIterations','SearchDistance','SearchScore','Success','Failed'};
for q=1:numel(matrixFields)
    f=matrixFields{q};
    if isfield(old,f)
        fresh.(f)(1:nOld,1:nOld)=old.(f)(1:nOld,1:nOld);
    end
end
if isfield(old,'Paths')
    fresh.Paths(1:nOld,1:nOld)=old.Paths(1:nOld,1:nOld);
end
if isfield(old,'FailureMessage')
    fresh.FailureMessage(1:nOld,1:nOld)=old.FailureMessage(1:nOld,1:nOld);
end
if isfield(old,'BuildRuntimeAccumulated')
    fresh.BuildRuntimeAccumulated=old.BuildRuntimeAccumulated;
end
Cost=fresh;
end

function Cost = ensureCurrentSchema(Cost,positions,flightNodes,map,cfg)
% Checkpoints produced by this version should already have the full schema.
% Re-initializing and copying also makes resume robust to small schema edits.
Cost = expandExistingCost(Cost,positions,flightNodes,map,cfg);
end

function flightNodes = resolveFlightNodes(positions,map,cfg)
n=size(positions,1);
flightNodes=zeros(n,3);
for j=1:n
    rc=round(positions(j,1:2));
    if rc(1)<1 || rc(1)>size(map,1) || rc(2)<1 || rc(2)>size(map,2)
        error('buildTerrainCostLibrary:TargetOutsideMap', ...
            'Target node %d lies outside the DEM.',j-1);
    end
    terrain=map(rc(1),rc(2));
    z=min(max(positions(j,3),terrain+cfg.map.minClearance), ...
        terrain+cfg.map.maxClearance);
    flightNodes(j,:)=[rc,z];
end
end

function signature = makeSignature(positions,map,cfg)
signature = struct();
signature.TargetPosition = positions;
signature.MapSize = size(map);
signature.MapResolution = cfg.map.resolution;
signature.MinClearance = cfg.map.minClearance;
signature.MaxClearance = cfg.map.maxClearance;
signature.cActualSlope = cfg.path.cActualSlope;
signature.cExpectedSlope = cfg.path.cExpectedSlope;
signature.HeuristicWeight = cfg.path.heuristicWeight;
signature.SmoothingEnabled = cfg.path.enableSmoothing;
signature.SmoothingSpacing = cfg.path.smoothingSpacing;
signature.MuHorizontal = cfg.path.muHorizontal;
signature.MuVertical = cfg.path.muVertical;
end

function n = countCompletedPairs(success)
n = nnz(success)-size(success,1); % exclude diagonal entries
end

function S = summarizeCost(Cost)
%SUMMARIZECOST Generate summary statistics of the terrain-cost library.
%
% Unit convention:
%   D-related distance quantities : m or km, explicitly included in names
%   E-related energy quantities   : J or kJ, explicitly included in names
%   runtime                       : s
%   relative asymmetry            : %
%   node/pair counts              : dimensionless

mask = Cost.Success & ~eye(size(Cost.Success));

S = struct();

%% Basic counts: dimensionless
S.numNodes       = Cost.NodeCount;
S.completedPairs = nnz(mask);
S.failedPairs    = nnz(Cost.Failed);

%% Mean statistics over all successfully generated directed pairs
if any(mask(:))
    S.meanDkm             = mean(Cost.D(mask)) / 1000;          % [km]
    S.meanEkJ             = mean(Cost.E(mask)) / 1000;          % [kJ]
    S.meanPairRuntimeS    = mean(Cost.PairRuntime(mask));        % [s]
    S.meanExpandedNodes   = mean(Cost.ExpandedNodes(mask));      % count
else
    S.meanDkm             = NaN;
    S.meanEkJ             = NaN;
    S.meanPairRuntimeS    = NaN;
    S.meanExpandedNodes   = NaN;
end

%% Directed-cost asymmetry
%
% For each unordered node pair {j,k}, compare the two directed costs:
%
%   Delta D_jk = |D_jk - D_kj|
%   Delta E_jk = |E_jk - E_kj|
%
% Relative asymmetry uses the symmetric mean denominator:
%
%   delta_D = |D_jk-D_kj| / ((D_jk+D_kj)/2) * 100 [%]
%
%   delta_E = |E_jk-E_kj| / ((E_jk+E_kj)/2) * 100 [%]
%
% This definition does not privilege either flight direction.

n = size(Cost.D,1);

dAsymM      = [];
eAsymJ      = [];
dRelAsymPct = [];
eRelAsymPct = [];

for j = 1:n
    for k = j+1:n

        if Cost.Success(j,k) && Cost.Success(k,j)

            Djk = Cost.D(j,k);
            Dkj = Cost.D(k,j);

            Ejk = Cost.E(j,k);
            Ekj = Cost.E(k,j);

            %% Absolute directional asymmetry
            dAbs = abs(Djk - Dkj);      % [m]
            eAbs = abs(Ejk - Ekj);      % [J]

            dAsymM(end+1) = dAbs;       %#ok<AGROW>
            eAsymJ(end+1) = eAbs;       %#ok<AGROW>

            %% Relative directional asymmetry
            dMean = 0.5 * (Djk + Dkj);
            eMean = 0.5 * (Ejk + Ekj);

            if dMean > eps
                dRelAsymPct(end+1) = 100 * dAbs / dMean; %#ok<AGROW>
            end

            if eMean > eps
                eRelAsymPct(end+1) = 100 * eAbs / eMean; %#ok<AGROW>
            end
        end
    end
end

%% Store asymmetry statistics with explicit units
if isempty(dAsymM)
    S.meanDirectionalAsymmetryDm        = NaN;   % [m]
    S.maxDirectionalAsymmetryDm         = NaN;   % [m]
    S.meanRelativeAsymmetryDPercent     = NaN;   % [%]
    S.maxRelativeAsymmetryDPercent      = NaN;   % [%]
else
    S.meanDirectionalAsymmetryDm        = mean(dAsymM);
    S.maxDirectionalAsymmetryDm         = max(dAsymM);

    S.meanRelativeAsymmetryDPercent     = mean(dRelAsymPct);
    S.maxRelativeAsymmetryDPercent      = max(dRelAsymPct);
end

if isempty(eAsymJ)
    S.meanDirectionalAsymmetryEJ        = NaN;   % [J]
    S.maxDirectionalAsymmetryEJ         = NaN;   % [J]
    S.meanRelativeAsymmetryEPercent     = NaN;   % [%]
    S.maxRelativeAsymmetryEPercent      = NaN;   % [%]
else
    S.meanDirectionalAsymmetryEJ        = mean(eAsymJ);
    S.maxDirectionalAsymmetryEJ         = max(eAsymJ);

    S.meanRelativeAsymmetryEPercent     = mean(eRelAsymPct);
    S.maxRelativeAsymmetryEPercent      = max(eRelAsymPct);
end

end

function saveCheckpointIfRequested(Cost,signature,file)
if isempty(file)
    return;
end
ensureParentDirectory(file);
checkpointSignature=signature; %#ok<NASGU>
save(file,'Cost','checkpointSignature','-v7.3');
end

function ensureParentDirectory(file)
[parent,~,~]=fileparts(file);
if ~isempty(parent) && exist(parent,'dir')~=7
    mkdir(parent);
end
end
