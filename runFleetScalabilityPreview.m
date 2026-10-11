function runFleetScalabilityPreview()
%RUNFLEETSCALABILITYPREVIEW Controlled fleet-size preview for the paper figure.

root = fileparts(mfilename('fullpath'));
outDir = fullfile(root,'output','experiment_results');
latestCsv = fullfile(outDir,'latest_reallocation_sensitivity.csv');
latestMat = fullfile(outDir,'latest_reallocation_sensitivity.mat');
backupCsv = fullfile(outDir,'six_uav_main_latest_backup.csv');
backupMat = fullfile(outDir,'six_uav_main_latest_backup.mat');

copyfile(latestCsv,backupCsv,'f');
copyfile(latestMat,backupMat,'f');
restoreMain = onCleanup(@()restoreLatest(latestCsv,latestMat,backupCsv,backupMat));

for nUav = 3:6
    fprintf('\n===== Fleet scalability preview: %d UAVs =====\n',nUav);
    S = runReallocationSensitivity( ...
        'NumScenarios',10, ...
        'DynamicTaskCounts',2, ...
        'CommDistancesM',1350, ...
        'RandomSeed',20260923, ...
        'Verbose',false, ...
        'NumVehicles',nUav);
    copyfile(S.csvFile,fullfile(outDir,sprintf('fleet_scalability_%duav.csv',nUav)),'f');
    save(fullfile(outDir,sprintf('fleet_scalability_%duav.mat',nUav)),'S','-v7.3');
end

clear restoreMain
fprintf('\nFleet scalability preview complete.\n');
end

function restoreLatest(latestCsv,latestMat,backupCsv,backupMat)
if exist(backupCsv,'file'), copyfile(backupCsv,latestCsv,'f'); end
if exist(backupMat,'file'), copyfile(backupMat,latestMat,'f'); end
end
