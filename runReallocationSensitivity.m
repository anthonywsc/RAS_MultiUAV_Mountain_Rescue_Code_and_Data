function S = runReallocationSensitivity(varargin)
%RUNREALLOCATIONSENSITIVITY Monte Carlo/sensitivity test for Lazy B&B.
%   S = runReallocationSensitivity('NumScenarios',50, ...
%       'DynamicTaskCounts',[1 2 3], ...
%       'CommDistancesM',[900 1350 1800], ...
%       'CapacityScales',[0.75 1 1.25], ...
%       'EnduranceScales',[0.75 1 1.25]);
%
%   Each generated dynamic scenario is run twice with the same initial
%   allocation and dynamic tasks:
%       1) lazyExactEvaluation = false  (No-B&B, all potential candidates)
%       2) lazyExactEvaluation = true   (Lazy-B&B)
%
%   The harness reports winner consistency, final route consistency, exact
%   evaluation reduction, actual DEMA* query reduction, and planning-time
%   reduction. Results are saved under output/experiment_results.

p = inputParser;
addParameter(p,'NumScenarios',10,@(x)isnumeric(x)&&isscalar(x)&&x>=1);
addParameter(p,'DynamicTaskCounts',[1 2 3],@(x)isnumeric(x)&&isvector(x)&&all(x>=1));
addParameter(p,'CommDistancesM',[900 1350 1800],@(x)isnumeric(x)&&isvector(x)&&all(x>0));
addParameter(p,'CapacityScales',1,@(x)isnumeric(x)&&isvector(x)&&all(x>0));
addParameter(p,'EnduranceScales',1,@(x)isnumeric(x)&&isvector(x)&&all(x>0));
addParameter(p,'RandomSeed',20260923,@(x)isnumeric(x)&&isscalar(x));
addParameter(p,'MaxOffsetGrid',80,@(x)isnumeric(x)&&isscalar(x)&&x>=0);
addParameter(p,'Verbose',true,@(x)islogical(x)&&isscalar(x));
addParameter(p,'NumVehicles',[],@(x)isempty(x)||(isnumeric(x)&&isscalar(x)&&x>=1&&x==round(x)));
parse(p,varargin{:});
opt = p.Results;

root = fileparts(mfilename('fullpath'));
cfgBase = uavConfig(root);
cfgBase.plot.enable = false;
cfgBase.plot.verbose = opt.Verbose;
cfgBase.reallocation.storeCandidateDiagnostics = false;

map = load(cfgBase.paths.mapTxt);
if isempty(opt.NumVehicles)
    data = load(cfgBase.paths.generatedData);
    data.map = map;
else
    cfgBase.uav.numVehicles = opt.NumVehicles;
    staticData = load(cfgBase.paths.staticData,'Target');
    staticCost = load(cfgBase.paths.staticCostLibrary,'Cost');
    data = initialTaskAllocation(map,cfgBase,false, ...
        'Target',staticData.Target,'Cost',staticCost.Cost);
    data.map = map;
end

rng(opt.RandomSeed,'twister');
rows = repmat(emptyScenarioRow(),0,1);
caseIndex = 0;

for c = 1:numel(opt.CommDistancesM)
    for r = 1:numel(opt.CapacityScales)
        for e = 1:numel(opt.EnduranceScales)
            for d = 1:numel(opt.DynamicTaskCounts)
                for s = 1:opt.NumScenarios
            caseIndex = caseIndex + 1;
            commM = opt.CommDistancesM(c);
            capacityScale = opt.CapacityScales(r);
            enduranceScale = opt.EnduranceScales(e);
            numTasks = opt.DynamicTaskCounts(d);
            seed = opt.RandomSeed + caseIndex - 1;
            rng(seed,'twister');
            newTargets = generateDynamicTargets(data.Target,map,cfgBase,numTasks,opt.MaxOffsetGrid);

            if opt.Verbose
                fprintf(['\n===== MC case %d | comm %.0f m | capacity x%.2f | ', ...
                    'endurance x%.2f | dynamic tasks %d | seed %d =====\n'], ...
                    caseIndex,commM,capacityScale,enduranceScale,numTasks,seed);
            end

            cfgFull = configureCase(cfgBase,commM,newTargets,false, ...
                capacityScale,enduranceScale,opt.Verbose);
            Rfull = simulateDynamicMission(data,cfgFull);

            cfgLazy = configureCase(cfgBase,commM,newTargets,true, ...
                capacityScale,enduranceScale,opt.Verbose);
            Rlazy = simulateDynamicMission(data,cfgLazy);

            rows(end+1,1) = summarizeCase(caseIndex,commM,capacityScale, ...
                enduranceScale,numTasks,seed, ...
                newTargets,Rfull,Rlazy); %#ok<AGROW>
            if opt.Verbose, printCaseSummary(rows(end)); end

            % Preserve progress in long Monte Carlo campaigns.  The final
            % files below remain the authoritative outputs.
            saveProgress(cfgBase,rows,opt);
                end
            end
        end
    end
end

scenarioTable = struct2table(rows);
summaryTable = aggregateSummary(scenarioTable);

outDir = fullfile(cfgBase.paths.outputDir,'experiment_results');
if ~exist(outDir,'dir')
    mkdir(outDir);
end
timestamp = char(datetime('now','Format','yyyyMMdd_HHmmss_SSS'));
outFile = fullfile(outDir,sprintf('%s_reallocation_sensitivity.mat',timestamp));
latestFile = fullfile(outDir,'latest_reallocation_sensitivity.mat');
csvFile = fullfile(outDir,sprintf('%s_reallocation_sensitivity.csv',timestamp));
csvLatestFile = fullfile(outDir,'latest_reallocation_sensitivity.csv');

S = struct();
S.options = opt;
S.scenarioTable = scenarioTable;
S.summaryTable = summaryTable;
S.outputFile = outFile;
S.csvFile = csvFile;

saveResultFile(outFile,S,scenarioTable,summaryTable,opt);
copyfile(outFile,latestFile,'f');
writetable(scenarioTable,csvFile);
copyfile(csvFile,csvLatestFile,'f');

fprintf('\n===== MONTE CARLO / SENSITIVITY SUMMARY =====\n');
disp(summaryTable);
fprintf('Saved MAT: %s\n',outFile);
fprintf('Saved CSV: %s\n',csvFile);
end

function saveResultFile(fileName,S,scenarioTable,summaryTable,opt)
% Prefer v7.3 for large campaigns, but keep the experiment deliverable on
% MATLAB installations/filesystems where HDF5-backed saves fail.
try
    save(fileName,'S','scenarioTable','summaryTable','opt','-v7.3');
catch ME
    warning('runReallocationSensitivity:SaveV73Fallback', ...
        'v7.3 save failed (%s); retrying with -v7.',ME.message);
    if exist(fileName,'file'), delete(fileName); end
    save(fileName,'S','scenarioTable','summaryTable','opt','-v7');
end
end

function cfg = configureCase(cfgBase,commM,newTargets,lazyExact, ...
        capacityScale,enduranceScale,verbose)
cfg = cfgBase;
cfg.comm.maxDistanceM = commM;
cfg.comm.maxDistance = commM;
cfg.dynamic.newTargets = newTargets;
cfg.reallocation.lazyExactEvaluation = lazyExact;
cfg.uav.capacity = cfgBase.uav.capacity*capacityScale;
cfg.uav.maxEndurance = cfgBase.uav.maxEndurance*enduranceScale;
cfg.uav.maxTravel = cfg.uav.speed*cfg.uav.maxEndurance;
cfg.plot.verbose = verbose;
cfg.reallocation.costUpdateVerbose = verbose;
end

function targets = generateDynamicTargets(Target,map,cfg,numTasks,maxOffset)
anchors = Target(2:end,1:2);
demSize = size(map);
targets = zeros(numTasks,4);
for k = 1:numTasks
    a = randi(size(anchors,1));
    rc = round(anchors(a,:) + randi([-maxOffset,maxOffset],1,2));
    rc(1) = min(max(rc(1),1),demSize(1));
    rc(2) = min(max(rc(2),1),demSize(2));
    z = map(rc(1),rc(2)) + cfg.uav.baseClearance;
    demand = randi(cfg.initial.demandRange);
    targets(k,:) = [rc(1),rc(2),z,demand];
end
end

function row = summarizeCase(caseIndex,commM,capacityScale,enduranceScale, ...
        numTasks,seed,newTargets,Rfull,Rlazy)
row = emptyScenarioRow();
row.caseIndex = caseIndex;
row.commMaxDistanceM = commM;
row.capacityScale = capacityScale;
row.enduranceScale = enduranceScale;
row.numDynamicTasks = numTasks;
row.seed = seed;
row.numEventsFull = numel(Rfull.eventLog);
row.numEventsLazy = numel(Rlazy.eventLog);
row.numAssignedFull = countAssigned(Rfull.eventLog);
row.numAssignedLazy = countAssigned(Rlazy.eventLog);
row.sameNumEvents = row.numEventsFull == row.numEventsLazy;
row.routeConsistent = isequaln(Rfull.sequences,Rlazy.sequences);
row.winnerConsistencyRate = winnerConsistencyRate(Rfull.eventLog,Rlazy.eventLog);
row.decisionConsistencyRate = decisionConsistencyRate(Rfull.eventLog,Rlazy.eventLog);
row.bidDeltaConsistencyRate = bidDeltaConsistencyRate(Rfull.eventLog,Rlazy.eventLog);

row.exactEvaluationsFull = sumEventField(Rfull.eventLog,'exactCandidateEvaluations');
row.exactEvaluationsLazy = sumEventField(Rlazy.eventLog,'exactCandidateEvaluations');
row.exactEvaluationReduction = fractionalReduction( ...
    row.exactEvaluationsFull,row.exactEvaluationsLazy);

row.demaQueriesFull = demaQueryCount(Rfull.eventLog);
row.demaQueriesLazy = demaQueryCount(Rlazy.eventLog);
row.demaQueryReduction = fractionalReduction(row.demaQueriesFull,row.demaQueriesLazy);

row.planningTimeFullS = sumEventField(Rfull.eventLog,'terrainPlanningRuntimeS');
row.planningTimeLazyS = sumEventField(Rlazy.eventLog,'terrainPlanningRuntimeS');
row.planningTimeReduction = fractionalReduction(row.planningTimeFullS,row.planningTimeLazyS);

row.totalEnumeratedCandidates = sumEventField(Rfull.eventLog,'totalEnumeratedCandidates');
row.totalPotentialCandidates = sumEventField(Rfull.eventLog,'potentialCandidateCount');
row.boundPrunedCandidates = sumEventField(Rlazy.eventLog,'boundPrunedCandidates');
row.endurancePrunedCandidates = sumEventField(Rlazy.eventLog, ...
    'optimisticEndurancePrunedCandidates');
row.dynamicTargets = {newTargets};
row.fullSequences = {Rfull.sequences};
row.lazySequences = {Rlazy.sequences};
end

function row = emptyScenarioRow()
row = struct('caseIndex',NaN,'commMaxDistanceM',NaN,'numDynamicTasks',NaN, ...
    'capacityScale',NaN,'enduranceScale',NaN, ...
    'seed',NaN,'numEventsFull',NaN,'numEventsLazy',NaN, ...
    'numAssignedFull',NaN,'numAssignedLazy',NaN,'sameNumEvents',false, ...
    'routeConsistent',false,'winnerConsistencyRate',NaN, ...
    'decisionConsistencyRate',NaN,'bidDeltaConsistencyRate',NaN, ...
    'exactEvaluationsFull',NaN,'exactEvaluationsLazy',NaN, ...
    'exactEvaluationReduction',NaN,'demaQueriesFull',NaN, ...
    'demaQueriesLazy',NaN,'demaQueryReduction',NaN, ...
    'planningTimeFullS',NaN,'planningTimeLazyS',NaN, ...
    'planningTimeReduction',NaN,'totalEnumeratedCandidates',NaN, ...
    'totalPotentialCandidates',NaN,'boundPrunedCandidates',NaN, ...
    'endurancePrunedCandidates',NaN,'dynamicTargets',{{}}, ...
    'fullSequences',{{}},'lazySequences',{{}});
end

function n = countAssigned(eventLog)
if isempty(eventLog)
    n = 0;
else
    n = sum([eventLog.assigned]);
end
end

function rate = winnerConsistencyRate(A,B)
n = max(numel(A),numel(B));
if n == 0
    rate = NaN;
    return;
end
same = false(n,1);
for k = 1:n
    same(k) = k <= numel(A) && k <= numel(B) && ...
        isequaln(A(k).winner,B(k).winner);
end
rate = mean(same);
end

function rate = decisionConsistencyRate(A,B)
n = max(numel(A),numel(B));
if n == 0
    rate = NaN;
    return;
end
same = false(n,1);
for k = 1:n
    same(k) = k <= numel(A) && k <= numel(B) && ...
        isequaln(A(k).winner,B(k).winner) && ...
        isequaln(A(k).insertAfterTask,B(k).insertAfterTask) && ...
        isequaln(A(k).insertBeforeTask,B(k).insertBeforeTask);
end
rate = mean(same);
end

function rate = bidDeltaConsistencyRate(A,B)
n = max(numel(A),numel(B));
if n == 0
    rate = NaN;
    return;
end
same = false(n,1);
tol = 1e-8;
for k = 1:n
    same(k) = k <= numel(A) && k <= numel(B) && ...
        abs(A(k).score-B(k).score) <= tol && ...
        abs(A(k).selectedDeltaDistanceM-B(k).selectedDeltaDistanceM) <= tol;
end
rate = mean(same);
end

function v = sumEventField(eventLog,fieldName)
if isempty(eventLog)
    v = 0;
else
    v = sum([eventLog.(fieldName)]);
end
end

function q = demaQueryCount(eventLog)
q = sumEventField(eventLog,'taskEdgePlans') + ...
    sumEventField(eventLog,'currentStatePlans');
end

function r = fractionalReduction(fullValue,lazyValue)
if fullValue <= 0
    r = NaN;
else
    r = (fullValue-lazyValue)/fullValue;
end
end

function T = aggregateSummary(scenarioTable)
[G,commM,capacityScale,enduranceScale,numTasks] = findgroups( ...
    scenarioTable.commMaxDistanceM,scenarioTable.capacityScale, ...
    scenarioTable.enduranceScale,scenarioTable.numDynamicTasks);
T = table(commM,numTasks,splitapply(@numel,scenarioTable.caseIndex,G), ...
    capacityScale,enduranceScale, ...
    splitapply(@mean,scenarioTable.winnerConsistencyRate,G), ...
    splitapply(@mean,scenarioTable.routeConsistent,G), ...
    splitapply(@mean,scenarioTable.exactEvaluationReduction,G), ...
    splitapply(@mean,scenarioTable.demaQueryReduction,G), ...
    splitapply(@mean,scenarioTable.planningTimeReduction,G), ...
    splitapply(@mean,scenarioTable.decisionConsistencyRate,G), ...
    splitapply(@mean,scenarioTable.bidDeltaConsistencyRate,G), ...
    'VariableNames',{'commMaxDistanceM','numDynamicTasks','numCases', ...
    'capacityScale','enduranceScale', ...
    'winnerConsistencyRate','routeConsistencyRate', ...
    'meanExactEvaluationReduction','meanDemaQueryReduction', ...
    'meanPlanningTimeReduction','decisionConsistencyRate', ...
    'bidDeltaConsistencyRate'});
end

function saveProgress(cfgBase,rows,opt)
% Save a compact checkpoint after each scenario so a long campaign can be
% inspected or recovered without changing the deterministic case seeds.
outDir = fullfile(cfgBase.paths.outputDir,'experiment_results');
if ~exist(outDir,'dir'), mkdir(outDir); end
progressFile = fullfile(outDir,'in_progress_reallocation_sensitivity.mat');
scenarioTable = struct2table(rows);
try
    save(progressFile,'scenarioTable','opt','-v7');
catch
    % A checkpoint must not abort the experiment; the final save below is
    % still attempted after all cases complete.
end
end

function printCaseSummary(row)
fprintf(['Case %d summary: winner %.3f | route %d | exact eval %.1f -> %.1f ', ...
    '(%.1f%% red.) | DEMA* %.1f -> %.1f (%.1f%% red.) | time %.2f -> %.2f s ', ...
    '(%.1f%% red.)\n'], ...
    row.caseIndex,row.winnerConsistencyRate,row.routeConsistent, ...
    row.exactEvaluationsFull,row.exactEvaluationsLazy, ...
    100*row.exactEvaluationReduction,row.demaQueriesFull,row.demaQueriesLazy, ...
    100*row.demaQueryReduction,row.planningTimeFullS,row.planningTimeLazyS, ...
    100*row.planningTimeReduction);
end
