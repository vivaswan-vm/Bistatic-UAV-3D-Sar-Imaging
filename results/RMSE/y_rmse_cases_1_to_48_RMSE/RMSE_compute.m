% Generic script to calculate RMSE for all .mat files in the folder
folderPath = pwd; 

% 1. Get a list of ALL .mat files
allFiles = dir(fullfile(folderPath, '*.mat'));

% 2. Filter out files we don't want to process (results from previous runs)
% We skip any file that ends in '_rmse.mat' or is named 'finalRMSE.mat'
validFiles = [];
for k = 1:length(allFiles)
    fname = allFiles(k).name;
    if ~contains(fname, '_rmse.mat') && ~strcmp(fname, 'finalRMSE.mat')
        validFiles = [validFiles; allFiles(k)];
    end
end

numFiles = length(validFiles);
fprintf('Found %d data files to process.\n', numFiles);

% 3. Initialize the 1 x N array for consolidated results
all_rmse_values = zeros(1, numFiles);
fileNamesList = cell(1, numFiles); % To keep track of which value belongs to which file

% 4. Loop through each valid file
for i = 1:numFiles
    baseFileName = validFiles(i).name;
    fullFileName = fullfile(folderPath, baseFileName);
    
    % Load the file
    dataStruct = load(fullFileName);
    
    % Get the variable name inside (e.g., x_case_1, myData, etc.)
    vars = fieldnames(dataStruct);
    if isempty(vars)
        continue; 
    end
    arrayData = dataStruct.(vars{1});
    
    % Calculate RMSE: sqrt(mean(array^2))
    rmse_val = sqrt(mean(arrayData.^2, 'all'));
    
    % Store for the final consolidated array
    all_rmse_values(i) = rmse_val;
    fileNamesList{i} = baseFileName;
    
    % Save individual file: [OriginalName]_rmse.mat
    [~, nameOnly, ~] = fileparts(baseFileName);
    individualFileName = fullfile(folderPath, strcat(nameOnly, '_rmse.mat'));
    save(individualFileName, 'rmse_val');
    
    fprintf('Processed (%d/%d): %s | RMSE: %.4f\n', i, numFiles, baseFileName, rmse_val);
end

% 5. Save the final summary file
% This file will contain both the values and the list of filenames in order
save('finalRMSE.mat', 'all_rmse_values', 'fileNamesList');

disp('---------------------------------------------------------');
disp('Done! Created individual _rmse files and "finalRMSE.mat".');