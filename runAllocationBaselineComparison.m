function S = runAllocationBaselineComparison(varargin)
%RUNALLOCATIONBASELINECOMPARISON Compare pre-mission assignment quality.
% Each paired instance uses the same task locations, randomized demands and
% deadlines, and the same six-UAV resource limits. All final routes are
% evaluated with the directed terrain-cost library.

p = inputParser;
addParameter(p,'NumScenarios',30,@(x)isnumeric(x)&&isscalar(x)&&x>=1);
addParameter(p,'RandomSeed',20261010,@(x)isnumeric(x)&&isscalar(x));
addParameter(p,'Verbose',true,@(x)islogical(x)&&isscalar(x));
parse(p,varargin{:});
opt = p.Results;

root = fileparts(mfilename('fullpath'));
cfg = uavConfig(root);
cfg.uav.numVehicles = 6;
cfg.plot.enable = false;
cfg.plot.verbose = false;

targetData = load(cfg.paths.staticData,'Target');
costData = load(cfg.paths.staticCostLibrary,'Cost');
TargetBase = targetData.Target;
CostTerrain = costData.Cost;
CostEuclidean = makeEuclideanCost(CostTerrain,cfg);

methods = {'Terrain-aware ACO','Euclidean ACO','Terrain greedy'};
rows = repmat(emptyRow(),opt.NumScenarios*numel(methods),1);
rowIndex = 0;

for s = 1:opt.NumScenarios
    scenarioSeed = opt.RandomSeed+s-1;
    Target = randomizeRequirements(TargetBase,CostTerrain,cfg,scenarioSeed);

    for methodIndex = 1:numel(methods)
        rowIndex = rowIndex+1;
        method = methods{methodIndex};
        allocationSeed = opt.RandomSeed+10000+s;
        tStart = tic;
        try
            switch methodIndex
                case 1
                    rng(allocationSeed,'twister');
                    data = initialTaskAllocation([],cfg,false, ...
                        'Target',Target,'Cost',CostTerrain);
                    routes = data.RouteCell;
                case 2
                    rng(allocationSeed,'twister');
                    data = initialTaskAllocation([],cfg,false, ...
                        'Target',Target,'Cost',CostEuclidean);
                    routes = data.RouteCell;
                otherwise
                    routes = greedyInsertion(Target,CostTerrain,cfg);
            end
            runtimeS = toc(tStart);
            metrics = evaluatePublicRoutes(routes,Target,CostTerrain,cfg,true);
            rows(rowIndex) = makeRow(s,scenarioSeed,method,runtimeS,metrics,routes,'');
        catch ME
            runtimeS = toc(tStart);
            rows(rowIndex) = makeRow(s,scenarioSeed,method,runtimeS,struct(),{},ME.message);
        end
    end

    if opt.Verbose
        fprintf('Allocation baseline scenario %d/%d complete.\n',s,opt.NumScenarios);
    end
end

scenarioTable = struct2table(rows);
summaryTable = summarizeByMethod(scenarioTable,methods);
pairedTable = pairedImprovements(scenarioTable);

outDir = fullfile(cfg.paths.outputDir,'experiment_results');
if ~exist(outDir,'dir'), mkdir(outDir); end
csvFile = fullfile(outDir,'allocation_baseline_comparison_30.csv');
summaryCsv = fullfile(outDir,'allocation_baseline_summary.csv');
matFile = fullfile(outDir,'allocation_baseline_comparison_30.mat');
writetable(scenarioTable,csvFile);
writetable(summaryTable,summaryCsv);
save(matFile,'scenarioTable','summaryTable','pairedTable','opt','-v7');

S = struct('options',opt,'scenarioTable',scenarioTable, ...
    'summaryTable',summaryTable,'pairedTable',pairedTable, ...
    'csvFile',csvFile,'summaryCsv',summaryCsv,'matFile',matFile);

disp(summaryTable);
disp(pairedTable);
fprintf('Saved CSV: %s\n',csvFile);
fprintf('Saved MAT: %s\n',matFile);
end

function CostE = makeEuclideanCost(Cost,cfg)
CostE = Cost;
D = Cost.DirectDistance;
D(1:size(D,1)+1:end) = 0;
CostE.D = D;
CostE.T = D/cfg.uav.speed;
CostE.E = D;
CostE.Success = isfinite(D);
CostE.Success(1:size(D,1)+1:end) = true;
end

function Target = randomizeRequirements(TargetBase,Cost,cfg,seed)
rng(seed,'twister');
Target = TargetBase;
n = size(Target,1);
Target(2:end,4) = randi(cfg.initial.demandRange,n-1,1);
for node = 2:n
    earliestMin = Cost.T(1,node)/60;
    lower = max(cfg.initial.deadlineRange(1),ceil(earliestMin)+1);
    upper = min(cfg.initial.deadlineRange(2),lower+8);
    if upper < lower, upper = lower; end
    Target(node,5) = randi([lower,upper]);
end
end

function routes = greedyInsertion(Target,Cost,cfg)
M = cfg.uav.numVehicles;
routesInternal = repmat({[1 1]},1,M);
[~,order] = sortrows([Target(2:end,5),Target(2:end,4)],[1 -2]);
taskNodes = order(:).'+1;

% Match the proposed construction rule that deploys every available UAV.
% With a common base and homogeneous vehicles, the first M deadline-ordered
% tasks can be assigned to arbitrary distinct vehicle labels.
initialCount = min(M,numel(taskNodes));
for u = 1:initialCount
    routesInternal{u} = [1 taskNodes(u) 1];
end
taskNodes = taskNodes(initialCount+1:end);

for node = taskNodes
    bestObjective = inf;
    bestRoutes = {};
    for u = 1:M
        route = routesInternal{u};
        for pos = 1:numel(route)-1
            candidate = routesInternal;
            candidate{u} = [route(1:pos),node,route(pos+1:end)];
            metrics = evaluateInternalRoutes(candidate,Target,Cost,cfg,false);
            if metrics.valid && metrics.objectiveM < bestObjective
                bestObjective = metrics.objectiveM;
                bestRoutes = candidate;
            end
        end
    end
    if isempty(bestRoutes)
        error('runAllocationBaselineComparison:GreedyInfeasible', ...
            'No feasible insertion exists for task %d.',node-1);
    end
    routesInternal = bestRoutes;
end

routes = cellfun(@(r)r-1,routesInternal,'UniformOutput',false);
end

function metrics = evaluatePublicRoutes(routes,Target,Cost,cfg,requireComplete)
routesInternal = cellfun(@(r)r+1,routes,'UniformOutput',false);
metrics = evaluateInternalRoutes(routesInternal,Target,Cost,cfg,requireComplete);
end

function Mtr = evaluateInternalRoutes(routes,Target,Cost,cfg,requireComplete)
M = numel(routes);
routeDistanceM = zeros(1,M);
routeEnergyJ = zeros(1,M);
routeTimeS = zeros(1,M);
routePayloadKg = zeros(1,M);
routeLatenessS = zeros(1,M);
deadlineViolations = zeros(1,M);
servedTasks = [];
valid = true;

for u = 1:M
    r = routes{u};
    if isempty(r) || r(1)~=1 || r(end)~=1
        valid = false;
        break;
    end
    elapsed = 0;
    for e = 1:numel(r)-1
        a = r(e); b = r(e+1);
        if ~Cost.Success(a,b) || ~isfinite(Cost.D(a,b)) || ~isfinite(Cost.T(a,b))
            valid = false;
            break;
        end
        routeDistanceM(u) = routeDistanceM(u)+Cost.D(a,b);
        routeEnergyJ(u) = routeEnergyJ(u)+Cost.E(a,b);
        elapsed = elapsed+Cost.T(a,b);
        if b~=1
            servedTasks(end+1) = b; %#ok<AGROW>
            routePayloadKg(u) = routePayloadKg(u)+Target(b,4);
            late = max(0,elapsed-Target(b,5)*60);
            routeLatenessS(u) = routeLatenessS(u)+late;
            deadlineViolations(u) = deadlineViolations(u)+(late>0);
        end
    end
    routeTimeS(u) = elapsed;
    if routePayloadKg(u)>cfg.uav.capacity+cfg.aco.feasibilityTolerance || ...
            routeTimeS(u)>cfg.uav.maxEndurance+cfg.aco.feasibilityTolerance
        valid = false;
    end
end

if requireComplete
    expected = 2:size(Target,1);
    valid = valid && numel(servedTasks)==numel(expected) && ...
        isequal(sort(servedTasks),expected);
else
    valid = valid && numel(unique(servedTasks))==numel(servedTasks);
end

if M>1
    sigmaLM = std(routeDistanceM);
    sigmaCKg = std(routePayloadKg);
else
    sigmaLM = 0;
    sigmaCKg = 0;
end
totalDistanceM = sum(routeDistanceM);
totalEnergyJ = sum(routeEnergyJ);
totalLatenessS = sum(routeLatenessS);
objectiveM = totalDistanceM + cfg.aco.lambdaRouteBalance*sigmaLM + ...
    cfg.aco.lambdaPayloadBalanceMPerKg*sigmaCKg + ...
    cfg.aco.lambdaLatenessMPerS*totalLatenessS;
if ~valid, objectiveM = inf; end

Mtr = struct('valid',valid,'objectiveM',objectiveM, ...
    'totalDistanceM',totalDistanceM,'totalEnergyJ',totalEnergyJ, ...
    'totalLatenessS',totalLatenessS, ...
    'totalDeadlineViolations',sum(deadlineViolations), ...
    'maxRouteDistanceM',max(routeDistanceM), ...
    'sigmaLM',sigmaLM,'sigmaCKg',sigmaCKg);
end

function row = emptyRow()
row = struct('scenario',NaN,'seed',NaN,'method','', ...
    'feasible',false,'objectiveKmEq',NaN,'totalDistanceKm',NaN, ...
    'totalEnergyKJ',NaN,'maxRouteDistanceKm',NaN, ...
    'routeDistanceStdKm',NaN,'payloadStdKg',NaN, ...
    'totalLatenessMin',NaN,'deadlineViolations',NaN, ...
    'runtimeS',NaN,'routes',{{}},'errorMessage','');
end

function row = makeRow(scenario,seed,method,runtimeS,m,routes,errorMessage)
row = emptyRow();
row.scenario = scenario;
row.seed = seed;
row.method = method;
row.runtimeS = runtimeS;
row.routes = {routes};
row.errorMessage = errorMessage;
if isempty(fieldnames(m)), return; end
row.feasible = m.valid;
row.objectiveKmEq = m.objectiveM/1000;
row.totalDistanceKm = m.totalDistanceM/1000;
row.totalEnergyKJ = m.totalEnergyJ/1000;
row.maxRouteDistanceKm = m.maxRouteDistanceM/1000;
row.routeDistanceStdKm = m.sigmaLM/1000;
row.payloadStdKg = m.sigmaCKg;
row.totalLatenessMin = m.totalLatenessS/60;
row.deadlineViolations = m.totalDeadlineViolations;
end

function T = summarizeByMethod(A,methods)
n = numel(methods);
method = strings(n,1);
numCases = zeros(n,1);
feasibleRate = zeros(n,1);
objectiveKmEq = nan(n,1);
totalDistanceKm = nan(n,1);
maxRouteDistanceKm = nan(n,1);
routeDistanceStdKm = nan(n,1);
payloadStdKg = nan(n,1);
totalLatenessMin = nan(n,1);
deadlineViolations = nan(n,1);
runtimeS = nan(n,1);
for k = 1:n
    method(k) = methods{k};
    mask = strcmp(A.method,methods{k});
    valid = mask & A.feasible;
    numCases(k) = nnz(mask);
    feasibleRate(k) = mean(A.feasible(mask));
    objectiveKmEq(k) = mean(A.objectiveKmEq(valid));
    totalDistanceKm(k) = mean(A.totalDistanceKm(valid));
    maxRouteDistanceKm(k) = mean(A.maxRouteDistanceKm(valid));
    routeDistanceStdKm(k) = mean(A.routeDistanceStdKm(valid));
    payloadStdKg(k) = mean(A.payloadStdKg(valid));
    totalLatenessMin(k) = mean(A.totalLatenessMin(valid));
    deadlineViolations(k) = mean(A.deadlineViolations(valid));
    runtimeS(k) = mean(A.runtimeS(mask));
end
T = table(method,numCases,feasibleRate,objectiveKmEq,totalDistanceKm, ...
    maxRouteDistanceKm,routeDistanceStdKm,payloadStdKg,totalLatenessMin, ...
    deadlineViolations,runtimeS);
end

function T = pairedImprovements(A)
methods = {'Euclidean ACO','Terrain greedy'};
baseline = strings(2,1);
pairedCases = zeros(2,1);
objectiveReductionPct = nan(2,1);
distanceReductionPct = nan(2,1);
maxRouteReductionPct = nan(2,1);
latenessReductionPct = nan(2,1);
for k = 1:2
    baseline(k) = methods{k};
    P = A(strcmp(A.method,'Terrain-aware ACO'),:);
    B = A(strcmp(A.method,methods{k}),:);
    [~,ia,ib] = intersect(P.scenario,B.scenario,'stable');
    P = P(ia,:); B = B(ib,:);
    valid = P.feasible & B.feasible;
    pairedCases(k) = nnz(valid);
    objectiveReductionPct(k) = 100*mean((B.objectiveKmEq(valid)-P.objectiveKmEq(valid))./B.objectiveKmEq(valid));
    distanceReductionPct(k) = 100*mean((B.totalDistanceKm(valid)-P.totalDistanceKm(valid))./B.totalDistanceKm(valid));
    maxRouteReductionPct(k) = 100*mean((B.maxRouteDistanceKm(valid)-P.maxRouteDistanceKm(valid))./B.maxRouteDistanceKm(valid));
    denom = max(B.totalLatenessMin(valid),1/60);
    latenessReductionPct(k) = 100*mean((B.totalLatenessMin(valid)-P.totalLatenessMin(valid))./denom);
end
T = table(baseline,pairedCases,objectiveReductionPct,distanceReductionPct, ...
    maxRouteReductionPct,latenessReductionPct);
end
