function data = initialTaskAllocation(map, cfg, doPlot, varargin)
%INITIALTASKALLOCATION Paper-aligned terrain-aware ACO pre-mission assignment.
%
%   data = initialTaskAllocation(map,cfg,doPlot)
%   data = initialTaskAllocation(...,'Target',Target,'Cost',Cost)
%
% The implementation follows Sec. IV-A of the current manuscript:
%   1) all mobility information comes from the directed terrain-cost library;
%   2) each ant constructs M UAV routes from resource-feasible candidate sets;
%   3) transition probability uses pheromone, terrain distance, and altitude
%      compatibility;
%   4) the solution objective combines total route length, route-length
%      balance, payload balance, and accumulated soft lateness;
%   5) energy is retained as an independent performance metric and is not
%      duplicated in the ACO objective.
%
% Target columns are [row, col, z, demand_kg, deadline_min]. Internal cost-
% library node IDs are 1...(N+1), with node 1 denoting the base. Public task
% IDs use base=0 and tasks=1...N.

if nargin < 3 || isempty(doPlot), doPlot = true; end

p = inputParser;
p.FunctionName = mfilename;
addParameter(p,'Target',[],@(x)isnumeric(x) || isempty(x));
addParameter(p,'Cost',[],@(x)isstruct(x) || isempty(x));
parse(p,varargin{:});
opt = p.Results;

if isempty(opt.Target)
    [Target, Cost] = buildEnvironment(map,cfg,doPlot);
else
    Target = opt.Target;
    validateTarget(Target,cfg);
    if isempty(opt.Cost)
        Cost = buildTerrainCostLibrary(Target,map,cfg, ...
            'CheckpointFile',cfg.paths.initialCostCheckpoint, ...
            'FinalFile',cfg.paths.initialCostLibrary, ...
            'Resume',cfg.cost.resume, ...
            'Verbose',cfg.cost.verbose, ...
            'FailFast',cfg.cost.failFast);
    else
        Cost = opt.Cost;
        validateCostForTarget(Cost,Target);
    end
    if doPlot
        plotEnvironment(Target,map,cfg);
    end
end

validateCostComplete(Cost);

[bestRoute, bestRoutesPublic, acoInfo] = runTerrainAwareACO(Target,Cost,cfg,doPlot);

if doPlot
    reportAllocation(bestRoutesPublic,acoInfo,cfg);
    drawAllocatedRoutes(acoInfo.bestRoutesInternal,Target,map,Cost.Paths,cfg);
end

data = struct();
data.map = map;
data.Target = Target;
data.Cost = Cost;
data.bestRoute = bestRoute;                 % legacy flattened public encoding
data.RouteCell = bestRoutesPublic;           % one [0 ... 0] vector per UAV
data.acoInfo = acoInfo;

% Backward-compatible aliases. These now point to the unified cost library.
data.Distance = Cost.DirectDistance;
data.PathLength = Cost.D;
data.Energy = Cost.E;
data.High = Cost.AltitudeDifference;
data.TravelTime = Cost.T;
data.WayPoints = Cost.Paths;
data.MinDis = acoInfo.bestTotalDistanceKmHistory;
data.Capacity = cfg.uav.capacity;
data.Travelcon = cfg.uav.maxTravel;
data.Stddis = cfg.initial.balanceThreshold;
data.Stdcon = cfg.initial.balanceThreshold;
data.tNum = size(Target,1)-1;
data.v = cfg.uav.speed;
end

%% ------------------------------------------------------------------------
function [Target,Cost] = buildEnvironment(map,cfg,doPlot)
% Generate a reproducible random rescue scenario. Path costs depend only on
% the first three target columns, so demand/deadline changes do not require
% rebuilding the terrain-cost library.

tNum = cfg.initial.numTargets;
base = cfg.uav.baseIndex;
startPoint = [base(1),base(2),map(base(1),base(2))+cfg.uav.baseClearance];

% Generate unique target grid cells and avoid the base cell.
numCells = numel(map);
assert(tNum < numCells,'Number of targets exceeds available DEM cells.');
baseLin = sub2ind(size(map),base(1),base(2));
perm = randperm(numCells,tNum+1);
perm(perm==baseLin) = [];
if numel(perm) < tNum
    remaining = setdiff(1:numCells,[perm,baseLin],'stable');
    perm = [perm,remaining(1:(tNum-numel(perm)))]; %#ok<AGROW>
end
perm = perm(1:tNum);
[x,y] = ind2sub(size(map),perm(:));

z = zeros(tNum,1);
for k=1:tNum
    z(k) = round(map(x(k),y(k))+1); % rescue-site elevation
end

City = [startPoint; x y z];
Demand = [0; randi(cfg.initial.demandRange,tNum,1)];
% Temporary deadlines; updated after terrain travel times are known.
EndTime = [0; randi(cfg.initial.deadlineRange,tNum,1)];
Target = [City,Demand,EndTime];

Cost = buildTerrainCostLibrary(Target,map,cfg, ...
    'CheckpointFile',cfg.paths.initialCostCheckpoint, ...
    'FinalFile',cfg.paths.initialCostLibrary, ...
    'Resume',cfg.cost.resume, ...
    'Verbose',cfg.cost.verbose, ...
    'FailFast',cfg.cost.failFast);

% Guarantee that every task has a meaningful deadline later than its direct
% terrain-informed travel time from the base. Lateness remains a soft term
% during multi-task route construction.
for node=2:size(Target,1)
    earliestMin = Cost.T(1,node)/60;
    lower = max(cfg.initial.deadlineRange(1),ceil(earliestMin)+1);
    upper = max(cfg.initial.deadlineRange(2),lower+cfg.initial.deadlineSlackMin);
    Target(node,5) = randi([lower,upper]);
end

validateTarget(Target,cfg);
if doPlot
    plotEnvironment(Target,map,cfg);
end
end

%% ------------------------------------------------------------------------
function [bestRouteFlat,bestRoutesPublic,info] = runTerrainAwareACO(Target,Cost,cfg,doPlot)
% Paper-aligned ACO using the candidate set in Eq. (candidate_set), the
% transition law in Eq. (aco_prob), and the objective in Eq. (aco_cost).

tStart = tic;
nNodes = size(Target,1);
nTargets = nNodes-1;
M = cfg.uav.numVehicles;
A = cfg.aco.numAnts;
maxIter = cfg.aco.maxIter;

assert(M>=1 && M<=nTargets, ...
    'cfg.uav.numVehicles must be between 1 and the number of known tasks.');

% Directed path availability is a hard prerequisite for an edge candidate.
validEdge = logical(Cost.Success) & isfinite(Cost.D) & isfinite(Cost.T);

% Terrain-distance heuristic eta_jk = 1/(D_jk + epsilon_D).
Eta = zeros(nNodes);
mask = validEdge & ~eye(nNodes);
Eta(mask) = 1 ./ (Cost.D(mask) + cfg.aco.epsilonDistanceM);

% Altitude compatibility xi_jk = exp(-lambda_h*|z_j-z_k|).
% Use safe flight-node altitudes from the terrain-cost library.
zFlight = Cost.FlightNodePosition(:,3);
dZ = abs(zFlight-zFlight.');
Xi = exp(-cfg.aco.lambdaAltitudePerM*dZ);
Xi(~validEdge) = 0;
Xi(1:nNodes+1:end) = 0;

Tau = cfg.aco.initialPheromone*ones(nNodes);
Tau(~validEdge) = 0;
Tau(1:nNodes+1:end) = 0;

bestJ = inf;
bestRoutes = [];
bestMetrics = struct();
bestIter = 0;
stopReason = 'maxIter';

history.bestObjective = nan(maxIter,1);
history.iterationBestObjective = nan(maxIter,1);
history.meanFeasibleObjective = nan(maxIter,1);
history.feasibleAntCount = zeros(maxIter,1);
history.bestTotalDistanceKm = nan(maxIter,1);
history.bestSigmaLKm = nan(maxIter,1);
history.bestSigmaCKg = nan(maxIter,1);
history.bestTotalLatenessMin = nan(maxIter,1);

noFeasibleStreak = 0;

for iter=1:maxIter
    deltaTau = zeros(nNodes);
    iterBestJ = inf;
    feasibleObjectives = nan(A,1);

    for ant=1:A
        [routes,constructed,assignedCount] = constructAntRoutes( ...
            Target,Cost,Tau,Eta,Xi,cfg);

        metrics = evaluateRoutes(routes,Target,Cost,cfg);
        feasible = constructed && assignedCount==nTargets && metrics.valid;

        if ~feasible
            continue;
        end

        feasibleObjectives(ant) = metrics.objectiveM;
        J = metrics.objectiveM;

        if J < iterBestJ
            iterBestJ = J;
        end

        if J < bestJ - cfg.aco.improvementTolM
            bestJ = J;
            bestRoutes = routes;
            bestMetrics = metrics;
            bestIter = iter;
        end

        % Eq. (pheromone update): every feasible ant contributes Q/J on
        % every directed edge used by its routes, including return edges.
        deposit = cfg.aco.QObjectiveM/J;
        for u=1:M
            r = routes{u};
            for e=1:numel(r)-1
                a = r(e); b = r(e+1);
                if a~=b
                    deltaTau(a,b) = deltaTau(a,b) + deposit;
                end
            end
        end
    end

    feasibleMask = isfinite(feasibleObjectives);
    history.feasibleAntCount(iter) = nnz(feasibleMask);
    if any(feasibleMask)
        history.iterationBestObjective(iter) = min(feasibleObjectives(feasibleMask));
        history.meanFeasibleObjective(iter) = mean(feasibleObjectives(feasibleMask));
        noFeasibleStreak = 0;
    else
        noFeasibleStreak = noFeasibleStreak+1;
    end

    % Pheromone evaporation and deposition.
    Tau = (1-cfg.aco.rho)*Tau + deltaTau;
    Tau(mask) = max(Tau(mask),cfg.aco.minPheromone);
    Tau(~validEdge) = 0;
    Tau(1:nNodes+1:end) = 0;

    if isfinite(bestJ)
        history.bestObjective(iter) = bestJ;
        history.bestTotalDistanceKm(iter) = bestMetrics.totalDistanceM/1000;
        history.bestSigmaLKm(iter) = bestMetrics.sigmaLM/1000;
        history.bestSigmaCKg(iter) = bestMetrics.sigmaCKg;
        history.bestTotalLatenessMin(iter) = bestMetrics.totalLatenessS/60;
    elseif iter>1
        history.bestObjective(iter) = history.bestObjective(iter-1);
        history.bestTotalDistanceKm(iter) = history.bestTotalDistanceKm(iter-1);
        history.bestSigmaLKm(iter) = history.bestSigmaLKm(iter-1);
        history.bestSigmaCKg(iter) = history.bestSigmaCKg(iter-1);
        history.bestTotalLatenessMin(iter) = history.bestTotalLatenessMin(iter-1);
    end

    if cfg.plot.verbose && (iter==1 || mod(iter,cfg.aco.verboseEvery)==0 || iter==maxIter)
        if isfinite(bestJ)
            fprintf(['ACO iteration %d/%d: feasible ants %d/%d, ', ...
                'best J=%.3f km-eq, distance=%.3f km, ', ...
                'sigma_L=%.3f km, sigma_C=%.3f kg, lateness=%.2f min\n'], ...
                iter,maxIter,history.feasibleAntCount(iter),A,bestJ/1000, ...
                bestMetrics.totalDistanceM/1000,bestMetrics.sigmaLM/1000, ...
                bestMetrics.sigmaCKg,bestMetrics.totalLatenessS/60);
        else
            fprintf('ACO iteration %d/%d: no feasible ant yet.\n',iter,maxIter);
        end
    end

    if noFeasibleStreak >= cfg.aco.maxNoFeasibleIterations
        error('initialTaskAllocation:NoFeasibleACO', ...
            ['No feasible complete ACO solution was generated for %d consecutive iterations. ', ...
             'Check UAV payload/endurance or target demands.'],noFeasibleStreak);
    end

    % Early stopping: stop after stallIter iterations without a meaningful
    % improvement of the global best objective.
    if bestIter > 0 && (iter-bestIter) >= cfg.aco.stallIter
        stopReason = 'stall';
        if cfg.plot.verbose
            fprintf(['ACO early stopping at iteration %d: global best has not ', ...
                'improved for %d iterations. Best solution first found at iteration %d.\n'], ...
                iter,cfg.aco.stallIter,bestIter);
        end
        break;
    end
end

actualIter = iter;

% Remove unused preallocated tail so convergence plots/statistics use only
% the iterations that were actually executed.
history.bestObjective = history.bestObjective(1:actualIter);
history.iterationBestObjective = history.iterationBestObjective(1:actualIter);
history.meanFeasibleObjective = history.meanFeasibleObjective(1:actualIter);
history.feasibleAntCount = history.feasibleAntCount(1:actualIter);
history.bestTotalDistanceKm = history.bestTotalDistanceKm(1:actualIter);
history.bestSigmaLKm = history.bestSigmaLKm(1:actualIter);
history.bestSigmaCKg = history.bestSigmaCKg(1:actualIter);
history.bestTotalLatenessMin = history.bestTotalLatenessMin(1:actualIter);

if ~isfinite(bestJ) || isempty(bestRoutes)
    error('initialTaskAllocation:NoFeasibleSolution', ...
        'ACO did not produce a feasible complete assignment.');
end

bestRoutesPublic = cell(1,M);
for u=1:M
    bestRoutesPublic{u} = bestRoutes{u}-1;
end
bestRouteFlat = flattenPublicRoutes(bestRoutesPublic);

info = bestMetrics;
info.bestObjectiveM = bestJ;
info.bestObjectiveKmEquivalent = bestJ/1000;
info.bestRoutesInternal = bestRoutes;
info.bestRoutesPublic = bestRoutesPublic;
info.bestRouteFlat = bestRouteFlat;
info.bestObjectiveHistory = history.bestObjective;
info.iterationBestObjectiveHistory = history.iterationBestObjective;
info.meanFeasibleObjectiveHistory = history.meanFeasibleObjective;
info.feasibleAntCountHistory = history.feasibleAntCount;
info.bestTotalDistanceKmHistory = history.bestTotalDistanceKm;
info.bestSigmaLKmHistory = history.bestSigmaLKm;
info.bestSigmaCKgHistory = history.bestSigmaCKg;
info.bestTotalLatenessMinHistory = history.bestTotalLatenessMin;
info.finalPheromone = Tau;
info.runtimeS = toc(tStart);
info.numAnts = A;
info.maxIter = maxIter;
info.actualIterations = actualIter;
info.bestIteration = bestIter;
info.stopReason = stopReason;
info.stallIter = cfg.aco.stallIter;
info.improvementTolM = cfg.aco.improvementTolM;
info.numVehicles = M;
info.weights.lambdaRouteBalance = cfg.aco.lambdaRouteBalance;
info.weights.lambdaPayloadBalanceMPerKg = cfg.aco.lambdaPayloadBalanceMPerKg;
info.weights.lambdaLatenessMPerS = cfg.aco.lambdaLatenessMPerS;
info.heuristic.alpha = cfg.aco.alpha;
info.heuristic.beta = cfg.aco.beta;
info.heuristic.gamma = cfg.aco.gamma;
info.heuristic.lambdaAltitudePerM = cfg.aco.lambdaAltitudePerM;

if doPlot
    figure('Name','Terrain-aware ACO convergence');
    plot(history.bestObjective/1000,'LineWidth',1.5); hold on;
    plot(history.iterationBestObjective/1000,'--','LineWidth',1.0);
    grid on;
    xlabel('Iteration');
    ylabel('Objective (km-equivalent)');
    legend('Global best','Iteration best','Location','best');
end
end

%% ------------------------------------------------------------------------
function [routes,success,assignedCount] = constructAntRoutes(Target,Cost,Tau,Eta,Xi,cfg)
% Joint round-robin route construction. Each UAV selects from its own
% resource-feasible candidate set A_i(j). Randomized UAV order in each sweep
% avoids a systematic preference for UAV 1 while preserving Eq. (aco_prob)
% for task selection within each UAV.

nTargets = size(Target,1)-1;
M = cfg.uav.numVehicles;
remaining = 2:(nTargets+1);

routes = cell(1,M);
current = ones(1,M);
routeLoad = zeros(1,M);       % kg
routeTime = zeros(1,M);       % s, time from base to current node
for u=1:M
    routes{u} = 1;
end

while ~isempty(remaining)
    progressed = false;
    uavOrder = randperm(M);

    for uu=1:M
        u = uavOrder(uu);
        if isempty(remaining), break; end

        candidates = feasibleCandidates(u,current(u),remaining,routeLoad(u), ...
            routeTime(u),Target,Cost,cfg);
        if isempty(candidates)
            continue;
        end

        next = selectTask(current(u),candidates,Tau,Eta,Xi,cfg);
        routes{u}(end+1) = next; %#ok<AGROW>
        routeLoad(u) = routeLoad(u) + Target(next,4);
        routeTime(u) = routeTime(u) + Cost.T(current(u),next);
        current(u) = next;
        remaining(remaining==next) = [];
        progressed = true;
    end

    if ~progressed
        break;
    end
end

for u=1:M
    routes{u}(end+1) = 1;
end

assignedCount = nTargets-numel(remaining);
success = isempty(remaining);
end

function candidates = feasibleCandidates(~,current,remaining,currentLoad,currentTime,Target,Cost,cfg)
% Candidate set:
% C_i + c_k <= Cmax,
% T_i + T_jk + T_k0 <= Tmax.

if isempty(remaining)
    candidates = remaining;
    return;
end

payloadOK = currentLoad + Target(remaining,4).' <= cfg.uav.capacity + cfg.aco.feasibilityTolerance;
returnTime = currentTime + Cost.T(current,remaining) + Cost.T(remaining,1).';
timeOK = returnTime <= cfg.uav.maxEndurance + cfg.aco.feasibilityTolerance;
edgeOK = Cost.Success(current,remaining) & Cost.Success(remaining,1).';

candidates = remaining(payloadOK & timeOK & edgeOK);
end

function next = selectTask(current,candidates,Tau,Eta,Xi,cfg)
% Numerically stable roulette-wheel implementation of
% tau^alpha * eta^beta * xi^gamma.

tau = Tau(current,candidates);
eta = Eta(current,candidates);
xi = Xi(current,candidates);

logW = cfg.aco.alpha*log(max(tau,realmin)) + ...
       cfg.aco.beta *log(max(eta,realmin)) + ...
       cfg.aco.gamma*log(max(xi,realmin));
logW = logW-max(logW);
w = exp(logW);

if ~all(isfinite(w)) || sum(w)<=0
    p = ones(size(candidates))/numel(candidates);
else
    p = w/sum(w);
end

idx = find(cumsum(p)>=rand,1,'first');
if isempty(idx), idx=numel(candidates); end
next = candidates(idx);
end

%% ------------------------------------------------------------------------
function Mtr = evaluateRoutes(routes,Target,Cost,cfg)
% Evaluate one complete multi-UAV solution. The objective has units of
% equivalent distance [m]:
% J = sum L_i + lambda_L*sigma_L + lambda_C*sigma_C
%     + lambda_T*sum lateness.
% lambda_C therefore has units [m/kg], and lambda_T has units [m/s].

M = numel(routes);
routeDistanceM = zeros(1,M);
routeEnergyJ = zeros(1,M);
routeTimeS = zeros(1,M);
routePayloadKg = zeros(1,M);
routeLatenessS = zeros(1,M);
deadlineViolations = zeros(1,M);
arrivalTimesS = cell(1,M);
valid = true;
servedTasks = [];

for u=1:M
    r = routes{u};
    if isempty(r) || r(1)~=1 || r(end)~=1
        valid=false; break;
    end

    elapsed = 0;
    arrivals = [];
    for e=1:numel(r)-1
        a=r(e); b=r(e+1);
        if ~Cost.Success(a,b) || ~isfinite(Cost.D(a,b)) || ~isfinite(Cost.T(a,b))
            valid=false; break;
        end

        routeDistanceM(u) = routeDistanceM(u)+Cost.D(a,b);
        routeEnergyJ(u) = routeEnergyJ(u)+Cost.E(a,b);
        elapsed = elapsed+Cost.T(a,b);

        if b~=1
            servedTasks(end+1)=b; %#ok<AGROW>
            routePayloadKg(u)=routePayloadKg(u)+Target(b,4);
            arrivals(end+1)=elapsed; %#ok<AGROW>
            dueS=Target(b,5)*60;
            late=max(0,elapsed-dueS);
            routeLatenessS(u)=routeLatenessS(u)+late;
            deadlineViolations(u)=deadlineViolations(u)+(late>0);
        end
    end
    if ~valid, break; end

    routeTimeS(u)=elapsed;
    arrivalTimesS{u}=arrivals;
    if routePayloadKg(u)>cfg.uav.capacity+cfg.aco.feasibilityTolerance || ...
            routeTimeS(u)>cfg.uav.maxEndurance+cfg.aco.feasibilityTolerance
        valid=false; break;
    end
end

nTargets=size(Target,1)-1;
if valid
    expected=2:(nTargets+1);
    if numel(servedTasks)~=nTargets || ~isequal(sort(servedTasks),expected)
        valid=false;
    end
end

if M>1
    sigmaLM=std(routeDistanceM);
    sigmaCKg=std(routePayloadKg);
else
    sigmaLM=0; sigmaCKg=0;
end

totalDistanceM=sum(routeDistanceM);
totalEnergyJ=sum(routeEnergyJ);
totalLatenessS=sum(routeLatenessS);

objectiveM = totalDistanceM + ...
    cfg.aco.lambdaRouteBalance*sigmaLM + ...
    cfg.aco.lambdaPayloadBalanceMPerKg*sigmaCKg + ...
    cfg.aco.lambdaLatenessMPerS*totalLatenessS;

if ~valid
    objectiveM=inf;
end

Mtr=struct();
Mtr.valid=valid;
Mtr.objectiveM=objectiveM;
Mtr.totalDistanceM=totalDistanceM;
Mtr.totalEnergyJ=totalEnergyJ;
Mtr.totalLatenessS=totalLatenessS;
Mtr.totalDeadlineViolations=sum(deadlineViolations);
Mtr.routeDistanceM=routeDistanceM;
Mtr.routeEnergyJ=routeEnergyJ;
Mtr.routeTimeS=routeTimeS;
Mtr.routePayloadKg=routePayloadKg;
Mtr.routeLatenessS=routeLatenessS;
Mtr.deadlineViolations=deadlineViolations;
Mtr.arrivalTimesS=arrivalTimesS;
Mtr.sigmaLM=sigmaLM;
Mtr.sigmaCKg=sigmaCKg;
end

%% ------------------------------------------------------------------------
function flat = flattenPublicRoutes(routesPublic)
flat = routesPublic{1};
for u=2:numel(routesPublic)
    r=routesPublic{u};
    flat=[flat,r(2:end)]; %#ok<AGROW>
end
end

function reportAllocation(routesPublic,info,cfg)
fprintf('\nInitial terrain-aware ACO allocation\n');
fprintf('Objective: %.3f km-equivalent | total distance: %.3f km | total energy: %.1f kJ\n', ...
    info.bestObjectiveKmEquivalent,info.totalDistanceM/1000,info.totalEnergyJ/1000);
fprintf('sigma_L = %.3f km | sigma_C = %.3f kg | total lateness = %.2f min | violations = %d\n', ...
    info.sigmaLM/1000,info.sigmaCKg,info.totalLatenessS/60,info.totalDeadlineViolations);
for u=1:numel(routesPublic)
    fprintf('UAV %d: route %s | D=%.2f km | E=%.1f kJ | T=%.1f min | payload=%.1f/%.1f kg | lateness=%.2f min\n', ...
        u,sprintf('%d ',routesPublic{u}),info.routeDistanceM(u)/1000, ...
        info.routeEnergyJ(u)/1000,info.routeTimeS(u)/60,info.routePayloadKg(u), ...
        cfg.uav.capacity,info.routeLatenessS(u)/60);
end
fprintf('ACO runtime: %.2f s\n\n',info.runtimeS);
end

%% ------------------------------------------------------------------------
function validateTarget(Target,cfg)
assert(size(Target,2)>=5, ...
    'Target must contain [row,col,z,demand_kg,deadline_min].');
assert(all(isfinite(Target(:))),'Target contains NaN or Inf.');
assert(Target(1,4)==0,'The first Target row must be the base with zero demand.');
assert(all(Target(2:end,4)>0),'All rescue tasks must have positive demand.');
assert(all(Target(2:end,5)>0),'All rescue tasks must have positive deadlines.');
assert(all(Target(2:end,4)<=cfg.uav.capacity), ...
    'At least one task demand exceeds the UAV payload capacity.');
end

function validateCostForTarget(Cost,Target)
assert(isfield(Cost,'TargetPosition') && isfield(Cost,'D') && ...
    isfield(Cost,'E') && isfield(Cost,'T') && isfield(Cost,'Paths') && ...
    isfield(Cost,'Success'),'Invalid terrain-cost library structure.');
assert(size(Cost.D,1)==size(Target,1), ...
    'Cost node count does not match Target node count.');
assert(isequaln(Cost.TargetPosition,Target(:,1:3)), ...
    'Cost.TargetPosition does not match Target(:,1:3).');
end

function validateCostComplete(Cost)
n=size(Cost.D,1);
offdiag=~eye(n);
assert(all(Cost.Success(offdiag)), ...
    'Terrain-cost library is incomplete: at least one directed pair is unavailable.');
assert(all(isfinite(Cost.D(offdiag))) && all(isfinite(Cost.E(offdiag))) && ...
    all(isfinite(Cost.T(offdiag))), ...
    'Terrain-cost library contains NaN/Inf on required directed pairs.');
end

%% ------------------------------------------------------------------------
function plotEnvironment(Target,map,cfg)
res=cfg.map.resolution; [X,Y]=meshgrid((0:size(map,2)-1)*res,(0:size(map,1)-1)*res);
figure('Name','Rescue targets');
surf(X,Y,map,'EdgeColor','none'); hold on; shading interp;
plot3((Target(1,2)-1)*res,(Target(1,1)-1)*res,Target(1,3),'p','MarkerSize',12,'MarkerFaceColor','auto');
plot3((Target(2:end,2)-1)*res,(Target(2:end,1)-1)*res,Target(2:end,3),'o','MarkerFaceColor','auto');
for i=1:size(Target,1)
    text((Target(i,2)-1)*res,(Target(i,1)-1)*res,Target(i,3)+80,num2str(i-1));
end
view(cfg.plot.view); axis tight; grid on; xlabel('x (m)'); ylabel('y (m)'); zlabel('Elevation (m)');
legend('Terrain','Base','Targets','Location','best');
end

function drawAllocatedRoutes(routes,Target,map,WayPoints,cfg)
res=cfg.map.resolution; [X,Y]=meshgrid((0:size(map,2)-1)*res,(0:size(map,1)-1)*res);
figure('Name','Initial UAV routes');
surf(X,Y,map,'EdgeColor','none'); hold on; shading interp;
plot3((Target(1,2)-1)*res,(Target(1,1)-1)*res,Target(1,3),'p','MarkerSize',12,'MarkerFaceColor','auto');
plot3((Target(2:end,2)-1)*res,(Target(2:end,1)-1)*res,Target(2:end,3),'o','MarkerFaceColor','auto');

h=gobjects(0);
for u=1:numel(routes)
    r=routes{u};
    for e=1:numel(r)-1
        a=r(e); b=r(e+1);
        if a==b, continue; end
        wp=WayPoints{a,b};
        if isempty(wp), continue; end
        hh=plot3((wp(:,2)-1)*res,(wp(:,1)-1)*res,wp(:,3),'LineWidth',1.5);
        if numel(h)<u, h(u)=hh; end %#ok<AGROW>
    end
end
view(cfg.plot.view); axis tight; grid on; xlabel('x (m)'); ylabel('y (m)'); zlabel('Elevation (m)');
if ~isempty(h)
    legend(h,arrayfun(@(k)sprintf('UAV %d',k),1:numel(h),'UniformOutput',false),'Location','best');
end
end
