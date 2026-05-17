function plot_results(combinedResultsFile, metricSelection)
%PLOT_RESULTS Plot all RMSE cases from the combined results file.
%
% Usage:
%   plot_results
%   plot_results('results/Combined_RMSE_Results.mat')
%   plot_results('results/Combined_RMSE_Results.mat', "X")
%   plot_results('results/Combined_RMSE_Results.mat', ["X" "Y" "Z"])

if nargin < 1 || strlength(string(combinedResultsFile)) == 0
    scriptDir = fileparts(mfilename('fullpath'));
    projectRoot = fileparts(fileparts(scriptDir));
    combinedResultsFile = fullfile(projectRoot, 'results', 'Combined_RMSE_Results.mat');
end

if nargin < 2 || strlength(string(metricSelection)) == 0
    metricSelection = ["X", "Y", "Z"];
end

data = load(combinedResultsFile, 'Results', 'Method_Names', 'Waveform_Names', ...
                            'Motion_Names', 'Std_Dev_Vel', 'Metric_Names');
[resultsDir, ~, ~] = fileparts(combinedResultsFile);

metricSelection = string(metricSelection);
if any(strcmpi(metricSelection, "all"))
    selectedMetricIdx = 1:numel(data.Metric_Names);
else
    selectedMetricIdx = zeros(1, numel(metricSelection));
    for idx = 1:numel(metricSelection)
        matchedMetricIdx = find(strcmpi(data.Metric_Names, metricSelection(idx)), 1);
        if isempty(matchedMetricIdx)
            error('Unknown metric "%s". Valid metrics are: %s.', ...
                  metricSelection(idx), strjoin(data.Metric_Names, ', '));
        end
        selectedMetricIdx(idx) = matchedMetricIdx;
    end
end

lineStyles = {'-o', '-s', '-^', '-d'};
numSmoothPoints = 20;
smoothStdVel = logspace(log10(min(data.Std_Dev_Vel)), log10(max(data.Std_Dev_Vel)), numSmoothPoints);
curveLabels = strings(numel(data.Method_Names) * numel(data.Waveform_Names), 1);
curveIdx = 1;
for method = 1:numel(data.Method_Names)
for waveform = 1:numel(data.Waveform_Names)
    curveLabels(curveIdx) = data.Waveform_Names(waveform) + " - " + data.Method_Names(method);
    curveIdx = curveIdx + 1;
end
end

figure;
tileLayout = tiledlayout(numel(selectedMetricIdx), numel(data.Motion_Names), ...
                         'TileSpacing', 'compact', 'Padding', 'compact');
legendHandles = gobjects(numel(curveLabels), 1);

for metricPlotIdx = 1:numel(selectedMetricIdx)
    metricIdx = selectedMetricIdx(metricPlotIdx);

    for motion = 1:numel(data.Motion_Names)
        nexttile;
        hold on;
        grid on;

        curveIdx = 1;
        for method = 1:numel(data.Method_Names)
        for waveform = 1:numel(data.Waveform_Names)
            rmseCurve = squeeze(data.Results(method, waveform, motion, :, metricIdx));
            smoothRmseCurve = pchip(log10(data.Std_Dev_Vel), rmseCurve, log10(smoothStdVel));
            curveHandle = plot(smoothStdVel, smoothRmseCurve, lineStyles{curveIdx}, 'LineWidth', 1.6);
            if metricPlotIdx == 1 && motion == 1
                legendHandles(curveIdx) = curveHandle;
            end
            curveIdx = curveIdx + 1;
        end
        end

        set(gca, 'XScale', 'log');
        xlabel('Velocity Std Dev');
        ylabel("RMSE - " + data.Metric_Names(metricIdx));
        title(data.Metric_Names(metricIdx) + " RMSE, " + data.Motion_Names(motion));

    end
end

plotLegend = legend(legendHandles, curveLabels, 'Orientation', 'horizontal');
plotLegend.Layout.Tile = 'south';
sgtitle(tileLayout, 'RMSE vs Velocity Std Dev');

plotFile = fullfile(resultsDir, "RMSE_" + strjoin(data.Metric_Names(selectedMetricIdx), "_") + "_Plots.png");
exportgraphics(gcf, plotFile, 'Resolution', 300);
end
