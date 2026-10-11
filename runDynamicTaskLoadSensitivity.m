function S = runDynamicTaskLoadSensitivity()
%RUNDYNAMICTASKLOADSENSITIVITY Paired six-UAV dynamic-load experiment.
% The same 50 random scenarios are evaluated for each event-load level by
% the exhaustive exact baseline and the lazy B&B implementation.

root = fileparts(mfilename('fullpath'));
outDir = fullfile(root,'output','experiment_results');
latestCsv = fullfile(outDir,'latest_reallocation_sensitivity.csv');
latestMat = fullfile(outDir,'latest_reallocation_sensitivity.mat');
backupCsv = fullfile(outDir,'latest_before_dynamic_load.csv');
backupMat = fullfile(outDir,'latest_before_dynamic_load.mat');

hadCsv = exist(latestCsv,'file') == 2;
hadMat = exist(latestMat,'file') == 2;
if hadCsv, copyfile(latestCsv,backupCsv,'f'); end
if hadMat, copyfile(latestMat,backupMat,'f'); end
restoreLatest = onCleanup(@()restoreMain(latestCsv,latestMat,backupCsv,backupMat,hadCsv,hadMat));

S = runReallocationSensitivity( ...
    'NumScenarios',50, ...
    'DynamicTaskCounts',1:5, ...
    'CommDistancesM',1350, ...
    'CapacityScales',1, ...
    'EnduranceScales',1, ...
    'RandomSeed',20260923, ...
    'Verbose',false, ...
    'NumVehicles',6);

copyfile(S.csvFile,fullfile(outDir,'dynamic_task_load_sensitivity_50x5.csv'),'f');
copyfile(S.outputFile,fullfile(outDir,'dynamic_task_load_sensitivity_50x5.mat'),'f');
clear restoreLatest
fprintf('Dynamic task-load sensitivity complete.\n');
end

function restoreMain(latestCsv,latestMat,backupCsv,backupMat,hadCsv,hadMat)
if hadCsv && exist(backupCsv,'file'), copyfile(backupCsv,latestCsv,'f'); end
if hadMat && exist(backupMat,'file'), copyfile(backupMat,latestMat,'f'); end
end
