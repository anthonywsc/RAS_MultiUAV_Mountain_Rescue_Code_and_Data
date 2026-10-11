function map = prepareDEM(tifFile, outputTxt, cfg, doPlot)
%PREPAREDEM Crop and interpolate SRTM DEM data.
% This consolidates the original 1.map/DEMmap.m script.

if nargin < 3 || isempty(cfg)
    cfg = uavConfig(fileparts(mfilename('fullpath')));
end
if nargin < 4, doPlot = true; end

assert(exist(tifFile, 'file') == 2, 'DEM TIFF not found: %s', tifFile);
[Z, ~] = readgeoraster(tifFile);
raw = double(Z(cfg.map.cropRows, cfg.map.cropCols));
[nr, nc] = size(raw);

[xGrid, yGrid] = meshgrid((0:nc-1)*cfg.map.originalResolution, ...
                          (0:nr-1)*cfg.map.originalResolution);
newX = 0:cfg.map.resolution:(nc-1)*cfg.map.originalResolution;
newY = 0:cfg.map.resolution:(nr-1)*cfg.map.originalResolution;
[newXGrid, newYGrid] = meshgrid(newX, newY);
map = interp2(xGrid, yGrid, raw, newXGrid, newYGrid, 'cubic');
map = round(map*100)/100;

writematrix(map, outputTxt, 'Delimiter', ' ');

fprintf('DEM crop: %d x %d -> interpolated map: %d x %d\n', nr, nc, size(map,1), size(map,2));
fprintf('Physical span: %.2f km x %.2f km at %.1f m resolution.\n', ...
    (size(map,1)-1)*cfg.map.resolution/1000, ...
    (size(map,2)-1)*cfg.map.resolution/1000, cfg.map.resolution);

if doPlot
    figure('Name','Interpolated DEM');
    surf((0:size(map,2)-1)*cfg.map.resolution, ...
         (0:size(map,1)-1)*cfg.map.resolution, map, 'EdgeColor','none');
    axis tight; view(-30,45); grid on;
    xlabel('x (m)'); ylabel('y (m)'); zlabel('Elevation (m)');
end
end
