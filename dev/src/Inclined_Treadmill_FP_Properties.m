%Music and lyrics by John Porter and Nev Pires

%Ths script will calculate the position and orientation of
%instrumented treadmill force plates at any given incline. The position and
%orientation of the plates in the level configuration are required to be
%entered into the system configuration ahead of time. It is preferred that
%only the treadmill's instrumented plates are defined within the system
%configuration. It is required that all devices be given unique names.

%The script also requires at least three non-collinear reference
%markers to be placed on the treadmill frame and a VSK created for them.
%There are no other restrictions as to the number of markers (greater than 3), their names or
%arrangement as long as they are not collinear. It is recommended to distribute the markers around
%the perimeter of the treadmill frame.

%A trial must be captured in the level position along with any number of
%trials in the desired inclined positions. All trials must be
%reconstructed, labeled and saved prior to running this script. There can
%be gaps in the marker data (don't fill the gaps) but all markers in the
%VSK must have some associated data.

%Trial names cannot contain spaces and must begin with a letter.

%The script file should be copied to the session folder and run from there.
%Run it through Matlab, not from the Nexus Matlab tab or pipeline
%operation. The script will prompt you to select the level trial and then
%to multi-select all incline trials for processing. The script expects at
%least two incline trials.

%The script computes the optimal translation and rotation between the level
%marker configuration and the inclined marker configuration. Then it
%applies the same transformation to the force plate position and
%orientation values.

%The script outputs a spreadsheet file to the session folder with a tab for each trial. Each tab
%presents the position and orientation values for each force plate. These
%values are to be entered into a new Nexus system configuration corresponding
%to an inclined position.

%The script applies a number of functions defined within the script.
%Therefore, the script will need to be run within a version
%of Matlab that supports built-in functions.

%This script was created under Matlab R2018a.

%%

clear
clc

%% Connect to Nexus

vicon = ViconNexus();

%% Start with level trial %%

uiwait(msgbox('Load Level Treadmill Trial','modal'));

[fileName,filePath] = uigetfile('*.c3d');
fileName = fileName(1:end - 4);
fullFileName = strcat(filePath,fileName);
vicon.OpenTrial(fullFileName,30);

%% Read level treadmill force plate position etc. 

deviceNames = vicon.GetDeviceNames;

if isempty(deviceNames) %Check to make sure devices exist and have names
    
    uiwait(msgbox('Devices not found or they are unnamed','modal'));
    return
    
end

plateNames = [];

for device = 1:length(deviceNames)
    
    deviceIDs.(deviceNames{device}) = vicon.GetDeviceIDFromName(deviceNames{device}); %Create a structure for storing device ids
    [~, deviceType, ~, ~, PlateProps.(deviceNames{device}), ~] = vicon.GetDeviceDetails(deviceIDs.(deviceNames{device})); %Create a structure for storing device properties
    
    if strcmp(deviceType,'ForcePlate') %Find force plate devices
        
        level_R.(deviceNames{device}) = reshape(PlateProps.(deviceNames{device}).WorldR,3,3)'; %Create a structure for storing force plate world 3x3 orientation matrices
        level_Orient.(deviceNames{device}) = AngleAxisFromMatrix(level_R.(deviceNames{device}));
        plateNames = [plateNames,deviceNames(device)];
        
    end
    
end

%%Make sure at least one force plate device exists

if isempty(plateNames)
    
    uiwait(msgbox('No force plates found','modal'));
    return
    
end
%% Get subject and marker information
subjects = vicon.GetSubjectNames();
if isempty(subjects)
    
    uiwait(msgbox('No subject found','modal'));
    return
    
end

subject = char(subjects(1));
%% segment = vicon.GetRootSegment(subject);
%% [~,~,markerList] = vicon.GetSegmentDetails(subject,segment); %Get the marker list
markerList = vicon.GetMarkerNames( subject ); %Get the marker list

if isempty(markerList)
    
    uiwait(msgbox('No marker set associated with the subject','modal'));
    return
    
elseif length(markerList) < 3
    
    uiwait(msgbox('The marker set requires at least three markers','modal'));
    return
    
end
        
%% Read the level marker data and set up the marker matrix

hasData = TrajectoryCheck(vicon,subject,markerList); %Do all the trajectories even exist?

if sum(hasData) < length(markerList) %Kick out if at least one marker is missing throughout the trial
    
    uiwait(msgbox('At least one marker is missing throughout the entire trial','modal')); 
    return
    
end

levelMarkers = GetMarkerData(vicon,subject,markerList);
levelMarkers = SetNaN(levelMarkers);
avgLevelMatrix = CreateAverageMatrix(levelMarkers);

%% Load and process each of the incline trials

uiwait(msgbox('Load Incline Treadmill Trials','modal'));

[inclineFileNames,filePath] = uigetfile('*.c3d','multiselect','on');
inclineFullFileNames{length(inclineFileNames)} = [];

for file = 1:length(inclineFileNames)
    
    inclineFileNames{file} = inclineFileNames{file}(1:end - 4);
    inclineFullFileNames{file} = strcat(filePath,inclineFileNames{file});
    vicon.OpenTrial(inclineFullFileNames{file},30);
    
    hasData = TrajectoryCheck(vicon,subject,markerList); %Do all the trajectories even exist?
    
    if sum(hasData) < length(markerList) %Kick out if at least one marker is missing throughout the trial
        
        uiwait(msgbox('At least one marker is missing throughout the entire trial','modal'));
        return
             
    end
    
    inclineMarkers.(inclineFileNames{file}) = GetMarkerData(vicon,subject,markerList);
    inclineMarkers.(inclineFileNames{file}) = SetNaN(inclineMarkers.(inclineFileNames{file}));
    avgInclineMatrix.(inclineFileNames{file}) = CreateAverageMatrix(inclineMarkers.(inclineFileNames{file}));
    
    %Use SVD to compute translation and rotation to move level marker cloud to incline
    %marker cloud. Then compute inclined force plate position and orientaton
    %directly
    [Translate.(inclineFileNames{file}),Rotate.(inclineFileNames{file})] = ...
        ChangeMarkerCloudPose(avgLevelMatrix,avgInclineMatrix.(inclineFileNames{file}));
    
    for plate = 1:length(plateNames)
        
        incline_Pos.(inclineFileNames{file}).(plateNames{plate}) = Translate.(inclineFileNames{file}) + ...
            Rotate.(inclineFileNames{file}) * PlateProps.(plateNames{plate}).WorldT';
        incline_R.(inclineFileNames{file}).(plateNames{plate}) = Rotate.(inclineFileNames{file}) * level_R.(plateNames{plate});
        incline_Orient.(inclineFileNames{file}).(plateNames{plate}) = AngleAxisFromMatrix(incline_R.(inclineFileNames{file}).(plateNames{plate}));
        
    end
    
end
        
%% Compiling level output data

levelPositionOutput = [];
levelOrientationOutput = [];

for plate = 1:length(plateNames)
    
    levelPositionOutput = [levelPositionOutput,PlateProps.(plateNames{plate}).WorldT'];
    levelOrientationOutput = [levelOrientationOutput,level_Orient.(plateNames{plate})];
    
end

%% Compiling incline output data

for file = 1:length(inclineFileNames)
    
    inclinePositionOutput.(inclineFileNames{file}) = [];
    inclineOrientationOutput.(inclineFileNames{file}) = [];
    
    for plate = 1:length(plateNames)
        
        inclinePositionOutput.(inclineFileNames{file}) = [inclinePositionOutput.(inclineFileNames{file}),incline_Pos.(inclineFileNames{file}).(plateNames{plate})];
        inclineOrientationOutput.(inclineFileNames{file}) = [inclineOrientationOutput.(inclineFileNames{file}),incline_Orient.(inclineFileNames{file}).(plateNames{plate})];
        
    end
    
end
        
%% Write Level Output Data

xlswrite('InclineOutputs.xlsx',{'Position'}, fileName, 'A1')
xlswrite('InclineOutputs.xlsx', levelPositionOutput, fileName, 'B2')
xlswrite('InclineOutputs.xlsx', plateNames, fileName, 'B1')
xlswrite('InclineOutputs.xlsx', {'x'; 'y'; 'z'}, fileName, 'A2')

xlswrite('InclineOutputs.xlsx',{'Orientation'}, fileName, 'A5')
xlswrite('InclineOutputs.xlsx', levelOrientationOutput, fileName, 'B6')
xlswrite('InclineOutputs.xlsx', plateNames, fileName, 'B5')
xlswrite('InclineOutputs.xlsx', {'x'; 'y'; 'z'}, fileName, 'A6')

%% Write Incline Output Data

for file = 1:length(inclineFileNames)
    
    xlswrite('InclineOutputs.xlsx',{'Position'}, inclineFileNames{file}, 'A1')
    xlswrite('InclineOutputs.xlsx', inclinePositionOutput.(inclineFileNames{file}), inclineFileNames{file}, 'B2')
    xlswrite('InclineOutputs.xlsx', plateNames, inclineFileNames{file}, 'B1')
    xlswrite('InclineOutputs.xlsx', {'x'; 'y'; 'z'}, inclineFileNames{file}, 'A2')
    
    xlswrite('InclineOutputs.xlsx',{'Orientation'}, inclineFileNames{file}, 'A5')
    xlswrite('InclineOutputs.xlsx', inclineOrientationOutput.(inclineFileNames{file}), inclineFileNames{file}, 'B6')
    xlswrite('InclineOutputs.xlsx', plateNames, inclineFileNames{file}, 'B5')
    xlswrite('InclineOutputs.xlsx', {'x'; 'y'; 'z'}, inclineFileNames{file}, 'A6')
    
end

%%

function DataExists = TrajectoryCheck(vicon,subject,markerList)

DataExists = zeros(1,length(markerList));

for marker = 1 : length(markerList)
    
    DataExists(:,marker) = vicon.HasTrajectory(subject,char(markerList(marker)));
    
end

end

function MarkerArray = GetMarkerData(vicon,subject,markerList)

numFrames = vicon.GetFrameCount;
[SelectedStart,SelectedEnd] = vicon.GetTrialRegionOfInterest();

%Read the marker data
MarkerArray = zeros(4,length(markerList),numFrames);

for marker = 1 : length(markerList)
    
    [x,y,z,e] = vicon.GetTrajectory(subject,char(markerList(marker)));
    MarkerArray(:,marker,:) = [x;y;z;e];
    
end

MarkerArray = MarkerArray(:,:,SelectedStart:SelectedEnd);

end
            
function OutputArray = SetNaN(MarkerArray) %Use only frames where all markers are present

    OutputArray = MarkerArray;
    
    for frame = 1 : size(MarkerArray,3)
        
      if sum(MarkerArray(4,:,frame)) < size(MarkerArray,2)
          
         OutputArray(1:3,:,frame) = NaN;
         OutputArray(4,:,frame) = false;
         
      end
      
    end
end

function MarkerMatrix = CreateAverageMatrix(MarkerArray) %Construct a matrix from the average XYZ coordinates of each marker
    
    MarkerMatrix = zeros(3,size(MarkerArray,2));
    
    for row = 1 : 3
        
        for column = 1 : size(MarkerArray,2)
            
             MarkerMatrix(row,column) = mean(MarkerArray(row,column,:),'omitnan');
             
        end
        
    end
    
end  

function [T,R] = ChangeMarkerCloudPose(InitialMatrix,FinalMatrix)

    Initial_Origin = mean(InitialMatrix,2); %Centroid of markers
    Centered_Initial_Matrix = InitialMatrix - Initial_Origin; %Subtract the origin coordinates from each marker position
    Final_Origin = mean(FinalMatrix,2); 
    Centered_Final_Matrix = FinalMatrix - Final_Origin;
    C = Centered_Final_Matrix * Centered_Initial_Matrix';
    
    [U,~,V] = svd(C); %Use Singular Value Decomposition to decontruct C matrix into U and V matrix components
    
    detUVT = det(U * V');
    L = eye(3,3);
    
    if detUVT < 0
        
        L(3,3) = -1;
        
    end
    
    R = U * L * V'; %Compute rotation matrix that rotates marker cloud from initial orientation to final orientaton
    T = Final_Origin - R * Initial_Origin; %Compute translation vector that translates marker cloud from initial orientation to final orientation
    
end

function Orientation = AngleAxisFromMatrix(A) %Compute the angle-axis form of the orientation from the rotation matrix

    angle = rad2deg(acos(( A(1,1) + A(2,2) + A(3,3) - 1)/2));
    r = sqrt((A(3,2) - A(2,3))^2+(A(1,3) - A(3,1))^2+(A(2,1) - A(1,2))^2);
    if r ~= 0
        x = (A(3,2) - A(2,3))/r;
        y = (A(1,3) - A(3,1))/r;
        z = (A(2,1) - A(1,2))/r;
        Orientation = [angle*x;angle*y;angle*z];
    else
        Orientation = [0;0;0];
    end
    
end


