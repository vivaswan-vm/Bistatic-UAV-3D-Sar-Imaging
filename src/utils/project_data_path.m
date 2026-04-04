function filePath = project_data_path(fileName)
%PROJECT_DATA_PATH Resolve a file path inside the repository data folder.

    utilsDir = fileparts(mfilename('fullpath'));
    projectRoot = fileparts(fileparts(utilsDir));
    filePath = fullfile(projectRoot, 'data', fileName);
end
