function S = runPublishedCBBAComparison(varargin)
%RUNPUBLISHEDCBBACOMPARISON Compare against a published CBBA adaptation.
% Choi, Brunet, and How, IEEE Transactions on Robotics, 2009, introduced
% consensus-based bundle auctions. With one task announced per event, CBBA
% reduces to local best-insertion bids followed by winner consensus. The
% baseline here uses straight-line marginal route costs for those bids and
% validates the winning insertion using the same terrain and resource model
% as the proposed terrain-exact auction.

p = inputParser;
addParameter(p,'NumScenarios',50,@(x)isnumeric(x)&&isscalar(x)&&x>=1);
addParameter(p,'NumDynamicTasks',2,@(x)isnumeric(x)&&isscalar(x)&&x>=1);
addParameter(p,'CommDistanceM',1350,@(x)isnumeric(x)&&isscalar(x)&&x>0);
addParameter(p,'RandomSeed',20261011,@(x)isnumeric(x)&&isscalar(x));
addParameter(p,'MaxOffsetGrid',80,@(x)isnumeric(x)&&isscalar(x)&&x>=0);
addParameter(p,'ScenarioOffset',0,@(x)isnumeric(x)&&isscalar(x)&&x>=0);
addParameter(p,'OutputTag','',@(x)ischar(x)||isstring(x));
addParameter(p,'Verbose',false,@(x)islogical(x)&&isscalar(x));
parse(p,varargin{:});
opt = p.Results;

root = fileparts(mfilename('fullpath'));
cfgBase = uavConfig(root);
cfgBase.uav.numVehicles = 6;
cfgBase.plot.enable = false;
cfgBase.plot.verbose = false;
cfgBase.reallocation.storeCandidateDiagnostics = false;
cfgBase.reallocation.costUpdateVerbose = false;
cfgBase.comm.maxDistanceM = opt.CommDistanceM;
cfgBase.comm.maxDistance = opt.CommDistanceM;

map = load(cfgBase.paths.mapTxt);
staticData = load(cfgBase.paths.staticData,'Target');
staticCost = load(cfgBase.paths.staticCostLibrary,'Cost');
rng(opt.RandomSeed,'twister');
data = initialTaskAllocation(map,cfgBase,false, ...
    'Target',staticData.Target,'Cost',staticCost.Cost);
data.map = map;

rows = repmat(emptyRow(),opt.NumScenarios,1);
for s = 1:opt.NumScenarios
    scenarioIndex = opt.ScenarioOffset+s;
    seed = opt.RandomSeed+scenarioIndex-1;
    rng(seed,'twister');
    newTargets = generateDynamicTargets(data.Target,map,cfgBase, ...
        opt.NumDynamicTasks,opt.MaxOffsetGrid);

    cfgProposed = cfgBase;
    cfgProposed.dynamic.newTargets = newTargets;
    cfgProposed.reallocation.selectionPolicy = 'terrain-exact';
    cfgProposed.reallocation.lazyExactEvaluation = true;

    cfgCBBA = cfgBase;
    cfgCBBA.dynamic.newTargets = newTargets;
    cfgCBBA.reallocation.selectionPolicy = 'cbba-euclidean';
    cfgCBBA.reallocation.lazyExactEvaluation = false;

    try
        if opt.Verbose
            Rproposed = simulateDynamicMission(data,cfgProposed);
            Rcbba = simulateDynamicMission(data,cfgCBBA);
        else
            evalc('Rproposed = simulateDynamicMission(data,cfgProposed);');
            evalc('Rcbba = simulateDynamicMission(data,cfgCBBA);');
        end
        rows(s) = summarizeScenario(scenarioIndex,seed,Rproposed,Rcbba,newTargets);
    catch ME
        rows(s).scenario = scenarioIndex;
        rows(s).seed = seed;
        rows(s).errorMessage = ME.message;
    end

    if opt.Verbose || mod(s,5)==0 || s==opt.NumScenarios
        fprintf('Published-CBBA comparison scenario %d/%d complete.\n', ...
            s,opt.NumScenarios);
    end

    saveCheckpoint(root,rows(1:s),opt);
end

scenarioTable = struct2table(rows,'AsArray',true);
summaryTable = aggregateResults(scenarioTable,opt.NumDynamicTasks);

outDir = fullfile(cfgBase.paths.outputDir,'experiment_results');
if ~exist(outDir,'dir'), mkdir(outDir); end
tag = char(opt.OutputTag);
if isempty(tag), tag='50'; end
csvFile = fullfile(outDir,['published_cbba_comparison_' tag '.csv']);
matFile = fullfile(outDir,['published_cbba_comparison_' tag '.mat']);
writetable(scenarioTable,csvFile);
save(matFile,'scenarioTable','summaryTable','opt','-v7');

S = struct('options',opt,'scenarioTable',scenarioTable, ...
    'summaryTable',summaryTable,'csvFile',csvFile,'matFile',matFile);
disp(summaryTable);
fprintf('Saved CSV: %s\n',csvFile);
fprintf('Saved MAT: %s\n',matFile);
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

function row = emptyRow()
row = struct('scenario',NaN,'seed',NaN, ...
    'eventsProposed',NaN,'eventsCBBA',NaN, ...
    'assignedProposed',NaN,'assignedCBBA',NaN, ...
    'assignmentRateProposed',NaN,'assignmentRateCBBA',NaN, ...
    'meanExactBidProposed',NaN,'meanExactBidCBBA',NaN, ...
    'incrementalDistanceKmProposed',NaN,'incrementalDistanceKmCBBA',NaN, ...
    'incrementalLatenessMinProposed',NaN,'incrementalLatenessMinCBBA',NaN, ...
    'deadlineAffectedEventsProposed',NaN,'deadlineAffectedEventsCBBA',NaN, ...
    'terrainTimeSProposed',NaN,'terrainTimeSCBBA',NaN, ...
    'exactEvaluationsProposed',NaN,'exactEvaluationsCBBA',NaN, ...
    'decisionAgreementRate',NaN,'bothCompletedProposed',false, ...
    'bothCompletedCBBA',false,'dynamicTargets',{{}},'errorMessage','');
end

function row = summarizeScenario(s,seed,P,B,newTargets)
row = emptyRow();
row.scenario = s;
row.seed = seed;
row.eventsProposed = numel(P.eventLog);
row.eventsCBBA = numel(B.eventLog);
row.assignedProposed = countAssigned(P.eventLog);
row.assignedCBBA = countAssigned(B.eventLog);
row.assignmentRateProposed = row.assignedProposed/size(newTargets,1);
row.assignmentRateCBBA = row.assignedCBBA/size(newTargets,1);
row.meanExactBidProposed = meanAssigned(P.eventLog,'score');
row.meanExactBidCBBA = meanAssigned(B.eventLog,'score');
row.incrementalDistanceKmProposed = sumAssigned(P.eventLog,'selectedDeltaDistanceM')/1000;
row.incrementalDistanceKmCBBA = sumAssigned(B.eventLog,'selectedDeltaDistanceM')/1000;
row.incrementalLatenessMinProposed = sumPositiveAssigned(P.eventLog,'selectedDeltaLatenessS')/60;
row.incrementalLatenessMinCBBA = sumPositiveAssigned(B.eventLog,'selectedDeltaLatenessS')/60;
row.deadlineAffectedEventsProposed = countPositiveAssigned(P.eventLog,'selectedDeltaLatenessS');
row.deadlineAffectedEventsCBBA = countPositiveAssigned(B.eventLog,'selectedDeltaLatenessS');
row.terrainTimeSProposed = sumField(P.eventLog,'terrainPlanningRuntimeS');
row.terrainTimeSCBBA = sumField(B.eventLog,'terrainPlanningRuntimeS');
row.exactEvaluationsProposed = sumField(P.eventLog,'exactCandidateEvaluations');
row.exactEvaluationsCBBA = sumField(B.eventLog,'exactCandidateEvaluations');
row.decisionAgreementRate = decisionAgreement(P.eventLog,B.eventLog);
row.bothCompletedProposed = all(P.completed);
row.bothCompletedCBBA = all(B.completed);
row.dynamicTargets = {newTargets};
end

function n = countAssigned(E)
if isempty(E), n=0; else, n=sum([E.assigned]); end
end

function v = meanAssigned(E,field)
if isempty(E), v=NaN; return; end
mask=[E.assigned];
if ~any(mask), v=NaN; else, v=mean([E(mask).(field)]); end
end

function v = sumAssigned(E,field)
if isempty(E), v=0; return; end
mask=[E.assigned];
if ~any(mask), v=0; else, v=sum([E(mask).(field)]); end
end

function v = sumPositiveAssigned(E,field)
if isempty(E), v=0; return; end
mask=[E.assigned];
if ~any(mask), v=0; else, v=sum(max(0,[E(mask).(field)])); end
end

function n = countPositiveAssigned(E,field)
if isempty(E), n=0; return; end
mask=[E.assigned];
if ~any(mask), n=0; else, n=sum([E(mask).(field)]>0); end
end

function v = sumField(E,field)
if isempty(E), v=0; else, v=sum([E.(field)]); end
end

function r = decisionAgreement(A,B)
ids = intersect([A.targetIndex],[B.targetIndex]);
if isempty(ids), r=NaN; return; end
same=false(size(ids));
for k=1:numel(ids)
    a=A(find([A.targetIndex]==ids(k),1));
    b=B(find([B.targetIndex]==ids(k),1));
    same(k)=isequaln(a.winner,b.winner) && ...
        isequaln(a.insertAfterTask,b.insertAfterTask) && ...
        isequaln(a.insertBeforeTask,b.insertBeforeTask);
end
r=mean(same);
end

function T = aggregateResults(A,numTasks)
valid = cellfun(@isempty,A.errorMessage);
A=A(valid,:);
method = ["Proposed terrain-exact";"CBBA-Euclidean adaptation"];
numScenarios = repmat(height(A),2,1);
assignmentRate = [mean(A.assignmentRateProposed);mean(A.assignmentRateCBBA)];
meanExactBid = [mean(A.meanExactBidProposed,'omitnan');mean(A.meanExactBidCBBA,'omitnan')];
incrementalDistanceKm = [mean(A.incrementalDistanceKmProposed);mean(A.incrementalDistanceKmCBBA)];
incrementalLatenessMin = [mean(A.incrementalLatenessMinProposed);mean(A.incrementalLatenessMinCBBA)];
deadlineAffectedEvents = [sum(A.deadlineAffectedEventsProposed);sum(A.deadlineAffectedEventsCBBA)];
terrainTimeS = [mean(A.terrainTimeSProposed);mean(A.terrainTimeSCBBA)];
exactEvaluations = [mean(A.exactEvaluationsProposed);mean(A.exactEvaluationsCBBA)];
missionCompletionRate = [mean(A.bothCompletedProposed);mean(A.bothCompletedCBBA)];
generatedTasks = repmat(height(A)*numTasks,2,1);
T=table(method,numScenarios,generatedTasks,assignmentRate,meanExactBid, ...
    incrementalDistanceKm,incrementalLatenessMin,deadlineAffectedEvents, ...
    terrainTimeS,exactEvaluations,missionCompletionRate);
end

function saveCheckpoint(root,rows,opt)
outDir=fullfile(root,'output','experiment_results');
if ~exist(outDir,'dir'), mkdir(outDir); end
tag=char(opt.OutputTag);
if isempty(tag), tag='50'; end
checkpoint=fullfile(outDir,['in_progress_published_cbba_comparison_' tag '.mat']);
scenarioTable=struct2table(rows,'AsArray',true);
save(checkpoint,'scenarioTable','opt','-v7');
end
