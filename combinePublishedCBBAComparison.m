function S = combinePublishedCBBAComparison()
%COMBINEPUBLISHEDCBBACOMPARISON Merge isolated MATLAB batch outputs.
root=fileparts(mfilename('fullpath'));
outDir=fullfile(root,'output','experiment_results');
files=dir(fullfile(outDir,'published_cbba_comparison_batch_*.mat'));
assert(~isempty(files),'No published-CBBA batch files were found.');

scenarioTable=table();
for k=1:numel(files)
    D=load(fullfile(files(k).folder,files(k).name),'scenarioTable');
    scenarioTable=[scenarioTable;D.scenarioTable]; %#ok<AGROW>
end
scenarioTable=sortrows(scenarioTable,'scenario');
[~,uniqueRows]=unique(scenarioTable.scenario,'stable');
scenarioTable=scenarioTable(uniqueRows,:);
assert(height(scenarioTable)==50, ...
    'Expected 50 unique scenarios, found %d.',height(scenarioTable));

valid=cellfun(@isempty,scenarioTable.errorMessage);
A=scenarioTable(valid,:);
method=["Proposed terrain-exact";"CBBA-Euclidean adaptation"];
numScenarios=repmat(height(A),2,1);
generatedTasks=repmat(2*height(A),2,1);
assignmentRate=[mean(A.assignmentRateProposed);mean(A.assignmentRateCBBA)];
meanExactBid=[mean(A.meanExactBidProposed,'omitnan');mean(A.meanExactBidCBBA,'omitnan')];
incrementalDistanceKm=[mean(A.incrementalDistanceKmProposed);mean(A.incrementalDistanceKmCBBA)];
incrementalLatenessMin=[mean(A.incrementalLatenessMinProposed);mean(A.incrementalLatenessMinCBBA)];
deadlineAffectedEvents=[sum(A.deadlineAffectedEventsProposed);sum(A.deadlineAffectedEventsCBBA)];
terrainTimeS=[mean(A.terrainTimeSProposed);mean(A.terrainTimeSCBBA)];
exactEvaluations=[mean(A.exactEvaluationsProposed);mean(A.exactEvaluationsCBBA)];
missionCompletionRate=[mean(A.bothCompletedProposed);mean(A.bothCompletedCBBA)];
summaryTable=table(method,numScenarios,generatedTasks,assignmentRate, ...
    meanExactBid,incrementalDistanceKm,incrementalLatenessMin, ...
    deadlineAffectedEvents,terrainTimeS,exactEvaluations,missionCompletionRate);

csvFile=fullfile(outDir,'published_cbba_comparison_50.csv');
matFile=fullfile(outDir,'published_cbba_comparison_50.mat');
writetable(scenarioTable,csvFile);
save(matFile,'scenarioTable','summaryTable','-v7');
S=struct('scenarioTable',scenarioTable,'summaryTable',summaryTable, ...
    'csvFile',csvFile,'matFile',matFile);
disp(summaryTable);
end
