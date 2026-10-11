function C = compareLazyExactEvaluation()
%COMPARELAZYEXACTEVALUATION Compare local reallocation with pruning off/on.
%   C = compareLazyExactEvaluation() runs the same dynamic mission twice:
%   first with cfg.reallocation.lazyExactEvaluation=false, then with it true.
%   It compares winner, insertion position, best bid, DeltaD, and final
%   Rdyn.sequences, then saves all data to output/V28_pruning_comparison.mat.

root = fileparts(mfilename('fullpath'));
cfgBase = uavConfig(root);
cfgBase.plot.enable = false;
cfgBase.plot.verbose = true;

map = load(cfgBase.paths.mapTxt);
data = load(cfgBase.paths.generatedData);
data.map = map;

fprintf('\nRunning baseline: lazyExactEvaluation = false\n');
cfgFull = cfgBase;
cfgFull.reallocation.lazyExactEvaluation = false;
Rfull = simulateDynamicMission(data,cfgFull);

fprintf('\nRunning pruned: lazyExactEvaluation = true\n');
cfgLazy = cfgBase;
cfgLazy.reallocation.lazyExactEvaluation = true;
Rlazy = simulateDynamicMission(data,cfgLazy);

C = buildComparison(Rfull,Rlazy);

outFile = fullfile(cfgBase.paths.outputDir,'V28_pruning_comparison.mat');
save(outFile,'Rfull','Rlazy','C','cfgFull','cfgLazy','-v7.3');
fprintf('\nSaved pruning comparison: %s\n',outFile);

printComparison(C);
end

function C = buildComparison(Rfull,Rlazy)
n = max(numel(Rfull.eventLog),numel(Rlazy.eventLog));
event = (1:n).';
winnerFull = nan(n,1);
winnerLazy = nan(n,1);
insertAfterFull = nan(n,1);
insertAfterLazy = nan(n,1);
insertBeforeFull = nan(n,1);
insertBeforeLazy = nan(n,1);
bidFull = nan(n,1);
bidLazy = nan(n,1);
deltaDFullM = nan(n,1);
deltaDLazyM = nan(n,1);
sameEvent = false(n,1);

for e = 1:n
    hasFull = e <= numel(Rfull.eventLog);
    hasLazy = e <= numel(Rlazy.eventLog);
    if hasFull
        F = Rfull.eventLog(e);
        winnerFull(e) = F.winner;
        insertAfterFull(e) = F.insertAfterTask;
        insertBeforeFull(e) = F.insertBeforeTask;
        bidFull(e) = F.score;
        deltaDFullM(e) = F.selectedDeltaDistanceM;
    end
    if hasLazy
        L = Rlazy.eventLog(e);
        winnerLazy(e) = L.winner;
        insertAfterLazy(e) = L.insertAfterTask;
        insertBeforeLazy(e) = L.insertBeforeTask;
        bidLazy(e) = L.score;
        deltaDLazyM(e) = L.selectedDeltaDistanceM;
    end
    sameEvent(e) = hasFull && hasLazy && ...
        isequaln(winnerFull(e),winnerLazy(e)) && ...
        isequaln(insertAfterFull(e),insertAfterLazy(e)) && ...
        isequaln(insertBeforeFull(e),insertBeforeLazy(e)) && ...
        isequaln(bidFull(e),bidLazy(e)) && ...
        isequaln(deltaDFullM(e),deltaDLazyM(e));
end

C = struct();
C.eventTable = table(event,winnerFull,winnerLazy,insertAfterFull, ...
    insertAfterLazy,insertBeforeFull,insertBeforeLazy,bidFull,bidLazy, ...
    deltaDFullM,deltaDLazyM,sameEvent);
C.sameNumEvents = numel(Rfull.eventLog) == numel(Rlazy.eventLog);
C.sameSequences = isequaln(Rfull.sequences,Rlazy.sequences);
C.allEventsIdentical = C.sameNumEvents && all(sameEvent);
C.allRequestedQuantitiesIdentical = C.allEventsIdentical && C.sameSequences;
C.fullSequences = Rfull.sequences;
C.lazySequences = Rlazy.sequences;
end

function printComparison(C)
fprintf('\n===== LAZY EXACT EVALUATION COMPARISON =====\n');
disp(C.eventTable);
fprintf('Same number of events: %d\n',C.sameNumEvents);
fprintf('Same final Rdyn.sequences: %d\n',C.sameSequences);
fprintf('All requested quantities identical: %d\n', ...
    C.allRequestedQuantitiesIdentical);

fprintf('\nFull-evaluation final sequences:\n');
printSequences(C.fullSequences);
fprintf('\nLazy-pruned final sequences:\n');
printSequences(C.lazySequences);
end

function printSequences(seq)
for i = 1:numel(seq)
    fprintf('UAV %d: %s\n',i,sprintf('%d ',seq{i}));
end
end
