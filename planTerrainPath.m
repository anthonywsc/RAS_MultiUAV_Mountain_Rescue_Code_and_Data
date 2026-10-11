function result = planTerrainPath(startPos, goalPos, map, cfg, doPlot)
%PLANTERRAINPATH Paper-aligned DEM-informed A* path planner.
%   RESULT = PLANTERRAINPATH(STARTPOS, GOALPOS, MAP, CFG, DOPLOT)
%   plans a terrain-following path on a 2-D DEM grid while assigning a
%   feasible flight altitude to every expanded horizontal node.
%
%   STARTPOS and GOALPOS may be [row col] or [row col z].  When altitude is
%   omitted (or is below the safe corridor), the endpoint flight altitude
%   is projected into [H+h_min, H+h_max].
%
%   This implementation is aligned with Sec. III of the ISJ manuscript:
%     1) standard A* selection from the complete OPEN set;
%     2) dynamic altitude assignment using the DEM clearance corridor;
%     3) slope-aware node priority f=g+h+c_a*s_a+c_b*s_e;
%     4) feature-based path simplification with explicit clearance checks;
%     5) cubic B-spline smoothing followed by clearance verification;
%     6) physically consistent task-edge distance D_jk and energy proxy
%        E_jk = sum(mu_h*d_h + mu_v*d_v).
%
%   Important output fields:
%     result.rawPath        - raw DEMA* grid path [row col z]
%     result.controlPath    - simplified clearance-preserving waypoints
%     result.path           - final smoothed/verified path
%     result.D              - final 3-D path distance D_jk [m]
%     result.E              - final energy-like cost E_jk [J]
%     result.energyKJ       - E_jk [kJ]
%     result.metrics        - detailed final path metrics
%     result.searchInfo     - A* diagnostics
%
%   Backward-compatible aliases result.length/result.nodeCount/result.path
%   are retained because the current ACO/dynamic modules use them.

if nargin < 5 || isempty(doPlot)
    doPlot = false;
end

validateattributes(map, {'numeric'}, {'2d','nonempty','finite','real'});
[startRC, startZ] = parseEndpoint(startPos, map, cfg);
[goalRC,  goalRefZ] = parseEndpoint(goalPos, map, cfg);

tTotal = tic;
[rawPath, searchInfo] = terrainAStar(startRC, goalRC, startZ, goalRefZ, map, cfg);

if isempty(rawPath)
    if isfield(cfg.path,'failOnNoPath') && cfg.path.failOnNoPath
        error('planTerrainPath:NoPath', ...
            'No feasible terrain path found between [%g,%g] and [%g,%g].', ...
            startRC(1), startRC(2), goalRC(1), goalRC(2));
    end
    result = failedResult(searchInfo, toc(tTotal));
    return;
end

[controlPath, simplifyInfo] = simplifyAndRestorePath(rawPath, map, cfg);
[finalPath, smoothInfo] = smoothAndVerifyPath(controlPath, map, cfg);

rawMetrics     = computePathMetrics(rawPath, cfg);
controlMetrics = computePathMetrics(controlPath, cfg);
finalMetrics   = computePathMetrics(finalPath, cfg);

result = struct();
result.success = true;
result.rawPath = rawPath;
result.controlPath = controlPath;
result.path = finalPath;
result.waypoints = finalPath;

% Paper quantities.
result.D = finalMetrics.length3D;     % D_jk [m]
result.E = finalMetrics.energyJ;      % E_jk [J]
result.energyKJ = finalMetrics.energyJ/1000;

% Detailed metrics and diagnostics.
result.rawMetrics = rawMetrics;
result.controlMetrics = controlMetrics;
result.metrics = finalMetrics;
result.searchInfo = searchInfo;
result.simplifyInfo = simplifyInfo;
result.smoothInfo = smoothInfo;
result.runtime = toc(tTotal);

% Backward-compatible fields used by the refactored v1 modules.
result.rawLength = rawMetrics.length3D;
result.length = finalMetrics.length3D;
result.rawNodeCount = size(rawPath,1);
result.controlNodeCount = size(controlPath,1);
result.nodeCount = size(finalPath,1);
result.rawSlopeMetric = rawMetrics.meanAbsSlopeDeg;
result.slopeMetric = finalMetrics.meanAbsSlopeDeg;
result.rawLegacyEnergyMetric = rawMetrics.energyJ;
result.legacyEnergyMetric = finalMetrics.energyJ;

if doPlot
    plotPathComparison(map, rawPath, controlPath, finalPath, cfg);
end
end

%% ========================================================================
%  A* search
%  ========================================================================
function [path,info] = terrainAStar(startRC, goalRC, startZ, goalRefZ, map, cfg)
[nRows,nCols] = size(map);
nNodes = nRows*nCols;
res = cfg.map.resolution;

startIdx = sub2ind([nRows,nCols], startRC(1), startRC(2));
goalIdx  = sub2ind([nRows,nCols], goalRC(1),  goalRC(2));

if startIdx == goalIdx
    path = [startRC, goalRefZ];
    info = struct('expandedNodes',1,'generatedNodes',1,'reopenedNodes',0, ...
        'queuePushes',1,'iterations',1,'reachedGoal',true, ...
        'finalSearchDistance',abs(goalRefZ-startZ),'finalSearchScore',abs(goalRefZ-startZ));
    return;
end

gScore = inf(nNodes,1);
fScore = inf(nNodes,1);
zScore = nan(nNodes,1);
parent = zeros(nNodes,1,'uint32');
closed = false(nNodes,1);
discovered = false(nNodes,1);

% Binary min-heap storing (node index, f, h).  Lazy duplicate entries are
% intentionally allowed: whenever a node obtains a lower f-value, a new
% heap entry is pushed.  Stale entries are discarded on pop.  This avoids
% the O(|OPEN|) global scan used in v2.1 and preserves the same A* priority.
heapCapacity = max(1024,min(nNodes,16384));
heapIdx = zeros(heapCapacity,1,'uint32');
heapF   = inf(heapCapacity,1);
heapH   = inf(heapCapacity,1);
heapCount = 0;

startH = heuristicDistance(startRC, startZ, goalRC, goalRefZ, res);
startExpectedSlope = expectedSlope(startRC, startZ, goalRC, goalRefZ, res, cfg.path.epsilon);
startF = startH + cfg.path.cExpectedSlope*startExpectedSlope;

gScore(startIdx) = 0;
fScore(startIdx) = startF;
zScore(startIdx) = startZ;
discovered(startIdx) = true;
[heapIdx,heapF,heapH,heapCount] = heapPush( ...
    heapIdx,heapF,heapH,heapCount,uint32(startIdx),startF,startH,cfg.path.heapTieTolerance);

expandedNodes = 0;
reopenedNodes = 0;
queuePushes = 1;
iterations = 0;
reachedGoal = false;
maxExpansions = min(cfg.path.maxExpansions, max(1,4*nNodes));

neighborOffsets = [ ...
    -1,-1; -1,0; -1,1; ...
     0,-1;        0,1; ...
     1,-1;  1,0;  1,1];

while heapCount > 0 && expandedNodes < maxExpansions
    iterations = iterations + 1;

    [currentIdx,currentF,~,heapIdx,heapF,heapH,heapCount] = heapPop( ...
        heapIdx,heapF,heapH,heapCount,cfg.path.heapTieTolerance);
    currentIdx = double(currentIdx);

    % Lazy-heap stale entry or an already-expanded copy.
    if currentF > fScore(currentIdx) + cfg.path.gTolerance || closed(currentIdx)
        continue;
    end

    closed(currentIdx) = true;
    expandedNodes = expandedNodes + 1;

    if currentIdx == goalIdx
        reachedGoal = true;
        break;
    end

    [rCur,cCur] = ind2sub([nRows,nCols], currentIdx);
    zCur = zScore(currentIdx);

    for m = 1:size(neighborOffsets,1)
        rNew = rCur + neighborOffsets(m,1);
        cNew = cCur + neighborOffsets(m,2);
        if rNew < 1 || rNew > nRows || cNew < 1 || cNew > nCols
            continue;
        end

        nextIdx = sub2ind([nRows,nCols], rNew, cNew);
        terrainNew = map(rNew,cNew);

        % Internal nodes follow the dynamic clearance corridor.  The goal
        % node is forced to the requested/projected terminal altitude so
        % that h(goal)=0 and P_jk has the prescribed endpoint p_k.
        if nextIdx == goalIdx
            zNew = goalRefZ;
        else
            zNew = assignAltitude(zCur, terrainNew, ...
                cfg.map.minClearance, cfg.map.maxClearance);
        end

        horizontal = res*hypot(rNew-rCur, cNew-cCur);
        dz = zNew-zCur;
        stepDistance = hypot(horizontal,dz);
        tentativeG = gScore(currentIdx) + stepDistance;

        actualSlope = atan(abs(dz)/(horizontal + cfg.path.epsilon));
        h = heuristicDistance([rNew,cNew], zNew, goalRC, goalRefZ, res);
        expSlope = expectedSlope([rNew,cNew], zNew, goalRC, goalRefZ, res, cfg.path.epsilon);
        f = tentativeG + cfg.path.heuristicWeight*h + ...
            cfg.path.cActualSlope*actualSlope + ...
            cfg.path.cExpectedSlope*expSlope;

        % Retain the lower paper-defined node priority at each horizontal
        % grid cell.  A better state reopens a previously closed cell.
        if f + cfg.path.gTolerance >= fScore(nextIdx)
            continue;
        end

        parent(nextIdx) = uint32(currentIdx);
        gScore(nextIdx) = tentativeG;
        fScore(nextIdx) = f;
        zScore(nextIdx) = zNew;

        if ~discovered(nextIdx)
            discovered(nextIdx) = true;
        end
        if closed(nextIdx)
            closed(nextIdx) = false;
            reopenedNodes = reopenedNodes + 1;
        end

        [heapIdx,heapF,heapH,heapCount] = heapPush( ...
            heapIdx,heapF,heapH,heapCount,uint32(nextIdx),f,h,cfg.path.heapTieTolerance);
        queuePushes = queuePushes + 1;
    end
end

if ~reachedGoal
    path = [];
else
    path = reconstructPath(parent,zScore,startIdx,goalIdx,[nRows,nCols]);
end

info = struct();
info.expandedNodes = expandedNodes;
info.generatedNodes = nnz(discovered);
info.reopenedNodes = reopenedNodes;
info.queuePushes = queuePushes;
info.iterations = iterations;
info.reachedGoal = reachedGoal;
info.finalSearchDistance = gScore(goalIdx);
info.finalSearchScore = fScore(goalIdx);
end

function [idxArr,fArr,hArr,count] = heapPush(idxArr,fArr,hArr,count,nodeIdx,f,h,tol)
count = count + 1;
if count > numel(idxArr)
    newCap = max(count,2*numel(idxArr));
    idxArr(newCap,1) = uint32(0);
    fArr(newCap,1) = inf;
    hArr(newCap,1) = inf;
end
idxArr(count) = nodeIdx;
fArr(count) = f;
hArr(count) = h;

k = count;
while k > 1
    p = floor(k/2);
    if ~heapKeyLess(fArr(k),hArr(k),fArr(p),hArr(p),tol)
        break;
    end
    [idxArr(k),idxArr(p)] = swapScalar(idxArr(k),idxArr(p));
    [fArr(k),fArr(p)] = swapScalar(fArr(k),fArr(p));
    [hArr(k),hArr(p)] = swapScalar(hArr(k),hArr(p));
    k = p;
end
end

function [nodeIdx,f,h,idxArr,fArr,hArr,count] = heapPop(idxArr,fArr,hArr,count,tol)
nodeIdx = idxArr(1);
f = fArr(1);
h = hArr(1);

idxArr(1) = idxArr(count);
fArr(1) = fArr(count);
hArr(1) = hArr(count);
count = count - 1;

k = 1;
while true
    left = 2*k;
    right = left + 1;
    if left > count
        break;
    end
    best = left;
    if right <= count && heapKeyLess(fArr(right),hArr(right),fArr(left),hArr(left),tol)
        best = right;
    end
    if ~heapKeyLess(fArr(best),hArr(best),fArr(k),hArr(k),tol)
        break;
    end
    [idxArr(k),idxArr(best)] = swapScalar(idxArr(k),idxArr(best));
    [fArr(k),fArr(best)] = swapScalar(fArr(k),fArr(best));
    [hArr(k),hArr(best)] = swapScalar(hArr(k),hArr(best));
    k = best;
end
end

function tf = heapKeyLess(f1,h1,f2,h2,tol)
if f1 < f2 - tol
    tf = true;
elseif abs(f1-f2) <= tol
    tf = h1 < h2;
else
    tf = false;
end
end

function [a,b] = swapScalar(a,b)
t = a; a = b; b = t;
end

function zNew = assignAltitude(zParent, terrainNew, hMin, hMax)
lower = terrainNew + hMin;
upper = terrainNew + hMax;
if zParent < lower
    zNew = lower;
elseif zParent <= upper
    zNew = zParent;
else
    zNew = upper;
end
end

function d = heuristicDistance(rc, z, goalRC, goalZ, resolution)
horizontal = resolution*hypot(rc(1)-goalRC(1), rc(2)-goalRC(2));
d = hypot(horizontal, z-goalZ);
end

function s = expectedSlope(rc, z, goalRC, goalZ, resolution, epsValue)
horizontal = resolution*hypot(rc(1)-goalRC(1), rc(2)-goalRC(2));
s = atan(abs(goalZ-z)/(horizontal + epsValue));
end

function path = reconstructPath(parent,zScore,startIdx,goalIdx,mapSize)
idx = goalIdx;
idxPath = zeros(1024,1,'uint32');
count = 0;
while true
    count = count + 1;
    if count > numel(idxPath)
        idxPath = [idxPath; zeros(numel(idxPath),1,'uint32')]; %#ok<AGROW>
    end
    idxPath(count) = uint32(idx);
    if idx == startIdx
        break;
    end
    idx = double(parent(idx));
    if idx == 0
        path = [];
        return;
    end
end
idxPath = flipud(idxPath(1:count));
[r,c] = ind2sub(mapSize,double(idxPath));
path = [r(:),c(:),zScore(double(idxPath))];
end

%% ========================================================================
%  Path simplification and terrain-clearance restoration
%  ========================================================================
function [path,info] = simplifyAndRestorePath(rawPath,map,cfg)
N = size(rawPath,1);
if N <= 2
    path = rawPath;
    info = struct('featureNodes',N,'restoredRawNodes',0,'syntheticRepairs',0);
    return;
end

featureIdx = detectFeatureNodes(rawPath,cfg);
keptIdx = featureIdx(:).';
restoredRawNodes = 0;

% Restore raw A* nodes when a candidate shortcut violates h_min clearance.
changed = true;
while changed
    changed = false;
    for k = 1:numel(keptIdx)-1
        ia = keptIdx(k);
        ib = keptIdx(k+1);
        [isClear,worstPoint] = checkSegmentClearance(rawPath(ia,:),rawPath(ib,:),map,cfg);
        if isClear
            continue;
        end
        candidates = (ia+1):(ib-1);
        if isempty(candidates)
            continue;
        end
        d2 = (rawPath(candidates,1)-worstPoint(1)).^2 + ...
             (rawPath(candidates,2)-worstPoint(2)).^2;
        [~,jj] = min(d2);
        restoreIdx = candidates(jj);
        keptIdx = sort(unique([keptIdx,restoreIdx])); %#ok<AGROW>
        restoredRawNodes = restoredRawNodes + 1;
        changed = true;
        break;
    end
end

path = rawPath(keptIdx,:);
[path,syntheticRepairs] = repairPolylineClearance(path,map,cfg,cfg.path.maxClearanceRepairs);
path = removeNearDuplicatePoints(path,cfg.path.duplicateTolerance);

% Never pass an infeasible control polyline to the smoothing stage. If a
% highly aggressive simplification cannot be repaired within the configured
% budget, fall back to the raw A* path and repair only its short segments.
if ~checkWholePathClearance(path,map,cfg)
    path = rawPath;
    [path,extraRepairs] = repairPolylineClearance(path,map,cfg,cfg.path.maxClearanceRepairs);
    syntheticRepairs = syntheticRepairs + extraRepairs;
    path = removeNearDuplicatePoints(path,cfg.path.duplicateTolerance);
end
if ~checkWholePathClearance(path,map,cfg)
    error('planTerrainPath:ClearanceRepairFailed', ...
        'Unable to construct a path satisfying the minimum terrain clearance.');
end

info = struct();
info.featureNodes = numel(featureIdx);
info.restoredRawNodes = restoredRawNodes;
info.syntheticRepairs = syntheticRepairs;
info.finalControlNodes = size(path,1);
end

function idx = detectFeatureNodes(path,cfg)
N = size(path,1);
keep = false(N,1);
keep([1,N]) = true;

segSlope = zeros(N-1,1);
segHeading = zeros(N-1,1);
for i = 1:N-1
    dr = path(i+1,1)-path(i,1);
    dc = path(i+1,2)-path(i,2);
    horizontal = cfg.map.resolution*hypot(dr,dc);
    segSlope(i) = atan2d(path(i+1,3)-path(i,3), max(horizontal,cfg.path.epsilon));
    segHeading(i) = atan2d(dr,dc);
end

for i = 2:N-1
    dzPrev = abs(path(i,3)-path(i-1,3));
    dzNext = abs(path(i+1,3)-path(i,3));
    slopeChange = abs(segSlope(i)-segSlope(i-1));
    slopeReversal = signWithDeadband(segSlope(i-1),cfg.path.slopeDeadbandDeg) ~= ...
                    signWithDeadband(segSlope(i),cfg.path.slopeDeadbandDeg);
    headingChange = angularDifferenceDeg(segHeading(i),segHeading(i-1));

    if dzPrev >= cfg.path.simplifyHeightTol || ...
       dzNext >= cfg.path.simplifyHeightTol || ...
       slopeChange >= cfg.path.slopeChangeTolDeg || ...
       slopeReversal || ...
       headingChange >= cfg.path.headingChangeTolDeg
        keep(i) = true;
    end
end
idx = find(keep);
end

function s = signWithDeadband(x,deadband)
if x > deadband
    s = 1;
elseif x < -deadband
    s = -1;
else
    s = 0;
end
end

function d = angularDifferenceDeg(a,b)
d = abs(mod(a-b+180,360)-180);
end

%% ========================================================================
%  B-spline smoothing and post-smoothing clearance verification
%  ========================================================================
function [path,info] = smoothAndVerifyPath(controlPath,map,cfg)
if size(controlPath,1) <= 2 || ~cfg.path.enableSmoothing
    path = controlPath;
    [path,repairs] = repairPolylineClearance(path,map,cfg,cfg.path.maxClearanceRepairs);
    info = struct('usedBSpline',false,'fallbackToControlPath',false, ...
        'pointwiseLifts',0,'segmentRepairs',repairs);
    return;
end

controlLength = computePathMetrics(controlPath,cfg);
nSamples = max(size(controlPath,1), ...
    ceil(controlLength.length3D/cfg.path.smoothingSpacing)+1);

smoothed = evaluateOpenUniformBSpline(controlPath,nSamples);
smoothed(1,:) = controlPath(1,:);
smoothed(end,:) = controlPath(end,:);

% B-spline overshoot outside the DEM is not accepted.
outside = (smoothed(:,1) < 1) | (smoothed(:,1) > size(map,1)) | ...
          (smoothed(:,2) < 1) | (smoothed(:,2) > size(map,2));
if any(outside)
    path = controlPath;
    [path,repairs] = repairPolylineClearance(path,map,cfg,cfg.path.maxClearanceRepairs);
    info = struct('usedBSpline',false,'fallbackToControlPath',true, ...
        'pointwiseLifts',0,'segmentRepairs',repairs);
    return;
end

% Project sampled points onto the minimum-clearance surface if the smooth
% curve locally dips toward terrain. This does not replace the subsequent
% continuous segment check; it only provides a safe initial correction.
pointwiseLifts = 0;
for i = 1:size(smoothed,1)
    terrain = terrainHeight(map,smoothed(i,1),smoothed(i,2));
    zMin = terrain + cfg.map.minClearance;
    if smoothed(i,3) < zMin
        smoothed(i,3) = zMin;
        pointwiseLifts = pointwiseLifts + 1;
    end
end

[path,repairs] = repairPolylineClearance(smoothed,map,cfg,cfg.path.maxClearanceRepairs);
path = removeNearDuplicatePoints(path,cfg.path.duplicateTolerance);

% Final verification. If numerical repair still cannot guarantee clearance,
% revert to the already verified control polyline rather than returning an
% infeasible smooth path.
if ~checkWholePathClearance(path,map,cfg)
    path = controlPath;
    [path,repairs2] = repairPolylineClearance(path,map,cfg,cfg.path.maxClearanceRepairs);
    repairs = repairs + repairs2;
    fallback = true;
    usedBSpline = false;
else
    fallback = false;
    usedBSpline = true;
end

info = struct('usedBSpline',usedBSpline,'fallbackToControlPath',fallback, ...
    'pointwiseLifts',pointwiseLifts,'segmentRepairs',repairs);
end

function curve = evaluateOpenUniformBSpline(control,nSamples)
% Toolbox-free open-uniform B-spline evaluator using Cox-de Boor basis.
nCtrl = size(control,1);
degree = min(3,nCtrl-1);
if degree <= 0
    curve = control;
    return;
end

nInterior = nCtrl-degree-1;
if nInterior > 0
    interior = (1:nInterior)/(nInterior+1);
else
    interior = [];
end
knots = [zeros(1,degree+1), interior, ones(1,degree+1)];
uVec = linspace(0,1,nSamples);
curve = zeros(nSamples,3);

for q = 1:nSamples
    u = uVec(q);
    basis = bsplineBasisVector(u,degree,knots,nCtrl);
    curve(q,:) = basis*control;
end
curve(1,:) = control(1,:);
curve(end,:) = control(end,:);
end

function N = bsplineBasisVector(u,degree,knots,nCtrl)
N = zeros(1,nCtrl);
for i = 1:nCtrl
    if (knots(i) <= u && u < knots(i+1)) || (u == 1 && i == nCtrl)
        N(i) = 1;
    end
end
for p = 1:degree
    Nprev = N;
    N = zeros(1,nCtrl);
    for i = 1:nCtrl
        left = 0;
        right = 0;
        denomLeft = knots(i+p)-knots(i);
        if denomLeft > 0
            left = (u-knots(i))/denomLeft*Nprev(i);
        end
        if i < nCtrl
            denomRight = knots(i+p+1)-knots(i+1);
            if denomRight > 0
                right = (knots(i+p+1)-u)/denomRight*Nprev(i+1);
            end
        end
        N(i) = left+right;
    end
end
if u == 1
    N(:) = 0;
    N(end) = 1;
end
end

%% ========================================================================
%  Terrain clearance helpers
%  ========================================================================
function [path,nRepairs] = repairPolylineClearance(path,map,cfg,maxRepairs)
nRepairs = 0;
i = 1;
while i < size(path,1)
    [isClear,worstPoint] = checkSegmentClearance(path(i,:),path(i+1,:),map,cfg);
    if isClear
        i = i + 1;
        continue;
    end

    nRepairs = nRepairs + 1;
    if nRepairs > maxRepairs
        break;
    end

    % Prevent insertion of a numerically duplicated endpoint.
    if norm(worstPoint(1:2)-path(i,1:2)) < cfg.path.duplicateTolerance || ...
       norm(worstPoint(1:2)-path(i+1,1:2)) < cfg.path.duplicateTolerance
        rcMid = 0.5*(path(i,1:2)+path(i+1,1:2));
        hMid = terrainHeight(map,rcMid(1),rcMid(2));
        worstPoint = [rcMid, hMid+cfg.map.minClearance+cfg.path.clearanceRepairMargin];
    end

    path = [path(1:i,:); worstPoint; path(i+1:end,:)]; %#ok<AGROW>
    % Recheck the first of the two newly created segments.
end
end

function ok = checkWholePathClearance(path,map,cfg)
ok = true;
for i = 1:size(path,1)-1
    if ~checkSegmentClearance(path(i,:),path(i+1,:),map,cfg)
        ok = false;
        return;
    end
end
end

function [isClear,worstPoint,minMargin] = checkSegmentClearance(pA,pB,map,cfg)
res = cfg.map.resolution;
horizontal = res*hypot(pB(1)-pA(1),pB(2)-pA(2));
nSamples = max(2,ceil(horizontal/cfg.path.clearanceCheckSpacing)+1);
lambda = linspace(0,1,nSamples).';
rows = (1-lambda)*pA(1) + lambda*pB(1);
cols = (1-lambda)*pA(2) + lambda*pB(2);
zs   = (1-lambda)*pA(3) + lambda*pB(3);

terr = interp2(map,cols,rows,'linear');
bad = ~isfinite(terr);
if any(bad)
    rr = min(max(round(rows(bad)),1),size(map,1));
    cc = min(max(round(cols(bad)),1),size(map,2));
    terr(bad) = map(sub2ind(size(map),rr,cc));
end
margin = zs-terr-cfg.map.minClearance;
[minMargin,idx] = min(margin);
isClear = minMargin >= -cfg.path.clearanceTolerance;
worstPoint = [rows(idx), cols(idx), ...
    terr(idx)+cfg.map.minClearance+cfg.path.clearanceRepairMargin];
end

function h = terrainHeight(map,row,col)
row = min(max(row,1),size(map,1));
col = min(max(col,1),size(map,2));
h = interp2(map,col,row,'linear');
if ~isfinite(h)
    h = map(round(row),round(col));
end
end

function path = removeNearDuplicatePoints(path,tol)
if size(path,1) <= 1
    return;
end
keep = true(size(path,1),1);
for i = 2:size(path,1)
    if norm(path(i,:)-path(i-1,:)) <= tol
        keep(i) = false;
    end
end
path = path(keep,:);
end

%% ========================================================================
%  Paper-aligned D_jk / E_jk and path metrics
%  ========================================================================
function m = computePathMetrics(path,cfg)
m = struct();
m.length3D = 0;
m.horizontalLength = 0;
m.verticalDistance = 0;
m.climbDistance = 0;
m.descentDistance = 0;
m.energyJ = 0;
m.energyKJ = 0;
m.meanAbsSlopeDeg = 0;
m.meanClimbAngleDeg = 0;
m.maxAbsSlopeDeg = 0;
m.numWaypoints = size(path,1);

if size(path,1) < 2
    return;
end

dr = diff(path(:,1))*cfg.map.resolution;
dc = diff(path(:,2))*cfg.map.resolution;
dz = diff(path(:,3));
dh = hypot(dr,dc);
dv = abs(dz);
ds = hypot(dh,dz);

m.length3D = sum(ds);
m.horizontalLength = sum(dh);
m.verticalDistance = sum(dv);
m.climbDistance = sum(max(dz,0));
m.descentDistance = sum(max(-dz,0));
m.energyJ = sum(cfg.path.muHorizontal*dh + cfg.path.muVertical*dv);
m.energyKJ = m.energyJ/1000;

angles = atan2d(dz,max(dh,cfg.path.epsilon));
m.meanAbsSlopeDeg = mean(abs(angles));
climbAngles = angles(angles>0);
if ~isempty(climbAngles)
    m.meanClimbAngleDeg = mean(climbAngles);
end
m.maxAbsSlopeDeg = max(abs(angles));
end

%% ========================================================================
%  Input/output and plotting helpers
%  ========================================================================
function [rc,z] = parseEndpoint(pos,map,cfg)
validateattributes(pos,{'numeric'},{'vector','real','finite'});
pos = pos(:).';
if ~(numel(pos)==2 || numel(pos)==3)
    error('Endpoint must be [row col] or [row col z].');
end
rc = round(pos(1:2));
if any(rc<1) || rc(1)>size(map,1) || rc(2)>size(map,2)
    error('Endpoint [%g,%g] lies outside the DEM.',pos(1),pos(2));
end
terrain = map(rc(1),rc(2));
if numel(pos)==3
    zRequested = pos(3);
else
    zRequested = terrain + cfg.map.minClearance;
end
z = min(max(zRequested,terrain+cfg.map.minClearance), ...
        terrain+cfg.map.maxClearance);
end

function result = failedResult(searchInfo,runtime)
result = struct();
result.success = false;
result.rawPath = [];
result.controlPath = [];
result.path = [];
result.waypoints = [];
result.D = inf;
result.E = inf;
result.energyKJ = inf;
result.length = inf;
result.nodeCount = 0;
result.metrics = struct();
result.searchInfo = searchInfo;
result.runtime = runtime;
end

function plotPathComparison(map,rawPath,controlPath,finalPath,cfg)
res = cfg.map.resolution;
[X,Y] = meshgrid((0:size(map,2)-1)*res,(0:size(map,1)-1)*res);
figure('Name','Paper-aligned terrain path planning');
surf(X,Y,map,'EdgeColor','none'); hold on; shading interp;
plot3((rawPath(:,2)-1)*res,(rawPath(:,1)-1)*res,rawPath(:,3), ...
    '--','LineWidth',0.8);
plot3((controlPath(:,2)-1)*res,(controlPath(:,1)-1)*res,controlPath(:,3), ...
    '-.','LineWidth',1.2);
plot3((finalPath(:,2)-1)*res,(finalPath(:,1)-1)*res,finalPath(:,3), ...
    '-','LineWidth',2.0);
plot3((finalPath(1,2)-1)*res,(finalPath(1,1)-1)*res,finalPath(1,3), ...
    'o','MarkerFaceColor','auto','MarkerSize',6);
plot3((finalPath(end,2)-1)*res,(finalPath(end,1)-1)*res,finalPath(end,3), ...
    's','MarkerFaceColor','auto','MarkerSize',6);
axis tight; grid on; view(cfg.plot.view);
xlabel('x (m)'); ylabel('y (m)'); zlabel('Elevation (m)');
legend('Terrain','Raw DEMA*','Simplified control path','Smoothed path', ...
    'Start','Goal','Location','best');
end
