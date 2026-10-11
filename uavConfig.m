function cfg = uavConfig(projectRoot)
%UAVCONFIG Centralized configuration for the multi-UAV mountain-rescue code.
%   Central configuration for the refactored program. The terrain-path
%   block is aligned with Sec. III of the current IEEE Systems Journal draft;
%   the remaining ACO/dynamic modules are updated in subsequent stages.

if nargin < 1 || isempty(projectRoot)
    projectRoot = fileparts(mfilename('fullpath'));
end

cfg.projectRoot = projectRoot;
cfg.randomSeed = 20260920;

%% Paths
cfg.paths.dataDir = fullfile(projectRoot, 'data');
cfg.paths.outputDir = fullfile(projectRoot, 'output');
cfg.paths.runResultsDir = fullfile(cfg.paths.outputDir, 'run_results');
cfg.paths.mapTxt = fullfile(cfg.paths.dataDir, 'map10.txt');
cfg.paths.staticData = fullfile(cfg.paths.dataDir, 'StaticData_original.mat');
cfg.paths.generatedData = fullfile(cfg.paths.outputDir, 'InitialData_refactored.mat');
cfg.paths.initialCostCheckpoint = fullfile(cfg.paths.outputDir, 'InitialCostLibrary_checkpoint.mat');
cfg.paths.initialCostLibrary = fullfile(cfg.paths.outputDir, 'InitialCostLibrary.mat');
cfg.paths.staticCostCheckpoint = fullfile(cfg.paths.outputDir, 'StaticCostLibrary_checkpoint.mat');
cfg.paths.staticCostLibrary = fullfile(cfg.paths.outputDir, 'StaticCostLibrary.mat');
cfg.paths.costDemoCheckpoint = fullfile(cfg.paths.outputDir, 'CostDemo_checkpoint.mat');

%% Run-result persistence
cfg.results.autoSave = true;
cfg.results.saveLatest = true;

%% Map / DEM
cfg.map.resolution = 10;       % m/grid cell
cfg.map.originalResolution = 90;
cfg.map.minClearance = 10;     % m, original program
cfg.map.maxClearance = 30;     % m, original program
cfg.map.cropRows = 1001:1100;  % original DEMmap.m
cfg.map.cropCols = 1001:1100;

%% UAV / mission
cfg.uav.speed = 60 / 3.6;      % m/s
cfg.uav.maxEndurance = 30*60;  % s
cfg.uav.maxTravel = cfg.uav.speed * cfg.uav.maxEndurance; % 30000 m
cfg.uav.capacity = 20;         % kg
cfg.uav.numVehicles = 3;       % M, homogeneous UAVs used in pre-mission allocation
cfg.uav.baseIndex = [100, 50]; % [row, col] in DEM grid
cfg.uav.baseClearance = 10;    % m

%% Initial targets / ACO
cfg.initial.numTargets = 10;
cfg.initial.demandRange = [2, 8];    % kg
cfg.initial.deadlineRange = [1, 20]; % min; regenerated until individually reachable
cfg.initial.balanceThreshold = 10;   % retained only as a legacy output alias
cfg.initial.deadlineSlackMin = 5;    % min, used only if direct reachability exceeds nominal range

% Paper-aligned terrain-aware ACO (Sec. IV-A)
cfg.aco.numAnts = 50;
cfg.aco.maxIter = 500;
% Early stopping: keep maxIter as a safety upper bound, but stop when the
% global best has not improved meaningfully for a prescribed number of iterations.
cfg.aco.stallIter = 50;             % iterations without meaningful global-best improvement
cfg.aco.improvementTolM = 1.0;      % [m-equivalent], minimum improvement that resets stall counter
cfg.aco.alpha = 1;                 % pheromone exponent
cfg.aco.beta = 1;                  % terrain-distance heuristic exponent
cfg.aco.gamma = 0.5;               % altitude-compatibility exponent chi
cfg.aco.rho = 0.1;                 % pheromone evaporation rate rho_p
cfg.aco.lambdaAltitudePerM = 1e-3; % lambda_h [1/m] in xi_jk=exp(-lambda_h*|z_j-z_k|)
cfg.aco.epsilonDistanceM = 1;      % epsilon_D [m]
cfg.aco.initialPheromone = 1;
cfg.aco.minPheromone = 1e-10;

% J_ACO is implemented in equivalent-distance units [m]:
% J=sum L_i + lambda_L*sigma_L + lambda_C*sigma_C + lambda_T*sum lateness.
cfg.aco.lambdaRouteBalance = 1.0;          % [m/m]
cfg.aco.lambdaPayloadBalanceMPerKg = 1000; % [m/kg], 1 kg imbalance ~ 1 km equivalent
cfg.aco.lambdaLatenessMPerS = cfg.uav.speed; % [m/s], 1 min lateness ~ 1 km at 60 km/h

% Pheromone deposition Delta tau=Q/J. Q therefore uses the same equivalent-
% distance unit as J so the deposited pheromone remains dimensionless.
cfg.aco.QObjectiveM = 1e4;        % [m-equivalent]
cfg.aco.feasibilityTolerance = 1e-9;
cfg.aco.maxNoFeasibleIterations = 20;
cfg.aco.verboseEvery = 25;

% Legacy names retained temporarily for scripts that may still reference them.
cfg.aco.lambdaHeight = cfg.aco.lambdaAltitudePerM;
cfg.aco.Q = cfg.aco.QObjectiveM;

%% Terrain-informed path planning (paper-aligned Section III)
% DEMA*: f = g + h + c_a*s_a + c_b*s_e.  Slope terms use radians,
% consistent with the mathematical formulation in the manuscript.
cfg.path.cActualSlope = 0.5;     % c_a
cfg.path.cExpectedSlope = 1.0;   % c_b
cfg.path.epsilon = 1e-9;
cfg.path.gTolerance = 1e-9;
cfg.path.heapTieTolerance = 1e-9; % lexicographic f/h tie handling in binary heap
cfg.path.heuristicWeight = 1.0;   % 1.0 preserves the manuscript A* definition
cfg.path.maxExpansions = inf;    % internally capped at 4*numel(map)
cfg.path.failOnNoPath = true;

% Feature-preserving simplification.
cfg.path.simplifyHeightTol = 5;      % m
cfg.path.slopeChangeTolDeg = 3;      % deg
cfg.path.slopeDeadbandDeg = 0.2;     % deg
cfg.path.headingChangeTolDeg = 30;   % deg, preserves relevant horizontal turns

% Continuous terrain-clearance verification.
cfg.path.clearanceCheckSpacing = 5;  % m; <= half of the 10-m DEM grid
cfg.path.clearanceTolerance = 1e-6;
cfg.path.clearanceRepairMargin = 0.5;% m numerical safety margin
cfg.path.maxClearanceRepairs = 500;
cfg.path.duplicateTolerance = 1e-6;

% Toolbox-free cubic B-spline smoothing.
cfg.path.enableSmoothing = true;
cfg.path.smoothingSpacing = 50;      % m between sampled smoothed waypoints (approx.)

% Energy-like task-edge metric E_jk = sum(mu_h*d_h + mu_v*d_v).
cfg.path.muHorizontal = 106;         % J/m
cfg.path.muVertical = 340;           % J/m
cfg.path.plotDemo = true;


%% Terrain-cost library
% Directed pairwise library C_H={D,E,T,P}. Directed computation is kept
% because the DEMA* altitude state and smoothing can make j->k and k->j
% differ. Checkpointing allows long offline builds to be safely resumed.
cfg.cost.checkpointEvery = 5;      % save after this many newly evaluated pairs
cfg.cost.resume = true;            % resume a compatible checkpoint automatically
cfg.cost.failFast = true;          % stop on the first planning failure after checkpointing
cfg.cost.demoNodes = 3;            % base + first tasks used by main('costdemo')
cfg.cost.verbose = true;

%% Dynamic mission
cfg.dynamic.timeStep = 1;             % s
cfg.dynamic.discoveryRadius = 100;    % grid cells = 1000 m at 10 m resolution
cfg.dynamic.defaultTaskDueTime = 30;  % min, as in original CBBA.m
cfg.dynamic.newTargets = [ ...
    200, 200, 1470, 3; ...
    574, 293, 1560, 4];

%% Communication (paper-aligned terrain-dependent model)
% Direct-link model: first screen by 3-D range, then evaluate the DEM terrain
% obstruction relative to the straight communication segment. The effective
% range is d_th = d_max/(1 + kappa*h_obs^2). Direct adjacency and multi-hop
% reachability are kept as separate graph quantities.
cfg.comm.maxDistanceM = 1350;              % d_max [m], current validated dynamic-test setting
cfg.comm.blockageAlphaPerM2 = 1e-4;        % kappa [1/m^2]
cfg.comm.losSampleSpacingM = 5;            % terrain-profile sampling spacing [m]
cfg.comm.obstructionToleranceM = 1e-6;     % numerical tolerance for h_obs [m]
cfg.comm.storeHistory = true;              % store A^d(t), R(t), and hop-count history

% Temporary aliases for legacy scripts; new communication code uses the
% explicit-unit names above.
cfg.comm.maxDistance = cfg.comm.maxDistanceM;
cfg.comm.blockageAlpha = cfg.comm.blockageAlphaPerM2;

%% Dynamic reallocation: CBBA-inspired insertion auction
% At each detected dynamic task, every communication-reachable active UAV
% evaluates all feasible insertion positions in its remaining route. The
% first option is an immediate diversion from the current flight position;
% later options insert the task after the current target or a later target.
cfg.reallocation.allowImmediateDiversion = true;
cfg.reallocation.distanceRefM = 5000;       % D_ref [m], close to mean static pairwise terrain cost
cfg.reallocation.latenessRefS = 60;         % T_ref [s]
cfg.reallocation.wDistance = 0.40;          % w_D
cfg.reallocation.wCapacity = 0.20;          % w_C
cfg.reallocation.wEndurance = 0.20;         % w_T
cfg.reallocation.wLateness = 0.20;          % w_L
cfg.reallocation.feasibilityTolerance = 1e-9;
cfg.reallocation.storeCandidateDiagnostics = true;
% Winner-selection policy. 'terrain-exact' is the proposed method.
% 'cbba-euclidean' is a single-task CBBA adaptation that ranks local
% insertion bids by straight-line incremental distance before validating
% the selected candidate with the terrain planner.
cfg.reallocation.selectionPolicy = 'terrain-exact';

% Event-driven online terrain-cost update. Dynamic tasks do not rebuild a
% complete row/column. v2.9.1 ranks all local insertion candidates by a
% conservative optimistic bid upper bound based on 3-D straight-line lower
% bounds, then evaluates exact DEMA* terrain costs lazily. Once the best
% remaining upper bound falls below the exact incumbent score, all later
% candidates are safely pruned. Exact after-task candidates reuse baseline
% suffix arrival times for incremental scoring, and exact task edges remain
% persistently cached.
cfg.reallocation.onlineCostMode = 'candidate-lazy-bound-cache';
cfg.reallocation.lazyExactEvaluation = true;
cfg.reallocation.lazyBoundTolerance = 1e-10; % score tolerance for exact B&B pruning
cfg.reallocation.costUpdateVerbose = true;

% Legacy names retained only for backward compatibility with old scripts.
cfg.auction.distanceWeight = 1;
cfg.auction.loadWeight = 0.5;
cfg.auction.timeWeight = 0.1;

%% Compatibility notes
% Deprecated compatibility flag. The paper-aligned communication module now
% always uses the physical line z(lambda)=z1+lambda*(z2-z1).
cfg.compatibility.legacyLOS = false;

% Original simulator stops when the final target is served and does not
% explicitly fly the return-to-base leg, although the ACO route cost includes it.
cfg.compatibility.stopBeforeReturnToBase = true;

%% Plotting
cfg.plot.enable = true;
cfg.plot.verbose = true;
cfg.plot.view = [-30, 25];
cfg.plot.verticalAspect = 10;
end
