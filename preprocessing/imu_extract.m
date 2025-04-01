% Alexander Beattie IMU Data Extraction Kuopio Gait Dataset
clear; close all; clc

root_dir = '~/Downloads/kuopio-gait-dataset';
output_dir = '~/data/kuopio-gait-dataset';

load_file = strcat(root_dir,'/measurement_data_1_to_17/01/imu_extracted/data_l_comf_01.mat');
load(load_file);
% Use fileparts to split the path
[filepath, name, ext] = fileparts(load_file);
disp(['Filename without extension: ', name]);

sampleRate = 100; % Known sample rate (e.g., 100 Hz for Kuopio Gait)

processFilesInDirectory(root_dir, sampleRate, output_dir);

function processFilesInDirectory(directoryPath, sampleRate, output_dir)
    % Check if the provided path is a valid directory
    if ~isfolder(directoryPath)
        error('The provided path is not a valid directory: %s', directoryPath);
    end
    
    % Get a list of all files and folders in the directory
    filesAndFolders = dir(directoryPath);
    
    % Loop through each item
    for i = 1:length(filesAndFolders)
        % Get the name of the current item
        itemName = filesAndFolders(i).name;
        
        % Skip the current directory (.) and parent directory (..)
        if strcmp(itemName, '.') || strcmp(itemName, '..')
            continue;
        end
        
        % Construct the full path of the item
        fullPath = fullfile(directoryPath, itemName);
        [filepath, name, ext] = fileparts(fullPath);
        
        % Check if the item is a directory
        if filesAndFolders(i).isdir
            % If it's a directory, recursively call this function
            processFilesInDirectory(fullPath, sampleRate,output_dir);
        elseif strcmp(ext, '.mat')
            % If it's a file, print its name
            fprintf('Processing File path: %s ; File name: %s\n', filepath, name);
            load(fullPath);
            % Get the field names of the struct
            fieldNames = fieldnames(sensors);
            % Get the first parent directory name
            [parentPath1, parentDir1, ~] = fileparts(filepath);
            
            % Get the second parent directory name
            [~, parentDir2, ~] = fileparts(parentPath1);
            
            % Display the results
            disp(['First parent directory name: ', parentDir1]);
            disp(['Second parent directory name: ', parentDir2]);
            write_dir = fullfile(output_dir,parentDir2,parentDir1);
            disp(write_dir);
            % Check if the directory exists
            if ~exist(write_dir, 'dir')
                % Create the directory
                mkdir(write_dir);
            end
            % Iterate through the fields of the struct
            for i = 1:length(fieldNames)
                fieldName = fieldNames{i}; % Get the current field name
                fieldValue = sensors.(fieldName); % Access the value using dynamic field names
                % Display the field name and its value
                fprintf('Writing Sensor Name: %s \n', fieldName);
                writeFile(fullfile(write_dir,name),fieldValue, fieldName, sampleRate);
            end
        end
    end
end

function writeFile(filename, sensor, sensorName, updateRate)
% Define the header for the CSV file
% Mat [R#][C#] Rotation matrix format [Row][Column] (3x3).
header = {'PacketCounter', ...
          'Acc_X', 'Acc_Y', 'Acc_Z', ...
          'Gyr_X', 'Gyr_Y', 'Gyr_Z', ...
          'Mat[1][1]', 'Mat[2][1]', 'Mat[3][1]', ...
          'Mat[1][2]', 'Mat[2][2]', 'Mat[3][2]', ...
          'Mat[1][3]', 'Mat[2][3]', 'Mat[3][3]'};

numRows = length(sensor.quaternion);
time = sensor.time;
data = zeros(numRows, length(header)); % Preallocate data array

data(:,1) = [time; time(1) + 1]; % PacketCounter
data(:,2:4) = sensor.calibratedAcceleration;
% data(:,5:7) = % should be gyros but no data
data(:,8:10) = squeeze(sensor.rotationMatrix(:,1,:))'; % Column 1 all Rows
data(:,11:13) = squeeze(sensor.rotationMatrix(:,2,:))';
data(:,14:16) = squeeze(sensor.rotationMatrix(:,3,:))';
data(end,:) = []; % remove the last row of junk data

% Open a file for writing
write_path = strcat(filename,'_00',sensorName,'.txt');
fileID = fopen(write_path, 'w');

% Write the update rate
fprintf(fileID,'// General information: \n');
fprintf(fileID, '//DeviceId: %s\n', sensorName); % Write the update rate
fprintf(fileID, '//Update Rate: %d\n', updateRate); % Write the update rate
% Write the header
fprintf(fileID, '%s\t', header{1:end-1}); % Write all but the last header
fprintf(fileID, '%s\n', header{end}); % Write the last header with a newline

% Write the data
writematrix(data,write_path,'Delimiter','tab','WriteMode', 'append')

% Close the file
fclose(fileID);

fprintf('Wrote: %s \n', write_path);

end