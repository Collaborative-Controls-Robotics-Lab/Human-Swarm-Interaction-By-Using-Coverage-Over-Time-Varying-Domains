% Vicon class should have methods for initialization (connect), get data
% (pose), and closing (disconnect)
function [ varargout ] = ViconClientFetch( MyClient, varargin )

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Copyright (C) OMG Plc 2009.
% All rights reserved.  This software is protected by copyright
% law and international treaties.  No part of this software / document
% may be reproduced or distributed in any form or by any means,
% whether transiently or incidentally to some other use of this software,
% without the written permission of the copyright owner.
%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Part of the Vicon DataStream SDK for MATLAB.
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if nargin > 1
    SubjectPose = varargin{1};
    % Turn on/off debugging statements
    verbose = 0;
elseif nargin > 2
    SubjectPose = varargin{1};
    verbose = varargin{2};
else
    % Turn on/off debugging statements
    verbose = 0;
    SubjectPose = [];
end

m2mm = 1e3;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Get a frame
if verbose
    fprintf( 'Waiting for new frame...' );
end

while MyClient.GetFrame().Result.Value ~= Result.Success
    if verbose
        fprintf( '.' );
    end
end% while

if verbose
    fprintf( '\n' );
end


% Get the frame number
Output_GetFrameNumber = MyClient.GetFrameNumber();
if verbose
    fprintf( 'Frame Number: %d\n', Output_GetFrameNumber.FrameNumber );
end

% Get the frame rate
Output_GetFrameRate = MyClient.GetFrameRate();
if verbose
    fprintf( 'Frame rate: %g\n', Output_GetFrameRate.FrameRateHz );
    for FrameRateIndex = 1:MyClient.GetFrameRateCount().Count
        FrameRateName  = MyClient.GetFrameRateName( FrameRateIndex ).Name;
        FrameRateValue = MyClient.GetFrameRateValue( FrameRateName ).Value;
        
        fprintf( '%s: %gHz\n', FrameRateName, FrameRateValue );
    end% for
    fprintf( '\n' );
end

% Get the timecode
Output_GetTimecode = MyClient.GetTimecode();
if verbose
    fprintf( 'Timecode: %dh %dm %ds %df %dsf %s %d %d %d\n\n',    ...
        Output_GetTimecode.Hours,                  ...
        Output_GetTimecode.Minutes,                ...
        Output_GetTimecode.Seconds,                ...
        Output_GetTimecode.Frames,                 ...
        Output_GetTimecode.SubFrame,               ...
        AdaptBool( Output_GetTimecode.FieldFlag ), ...
        Output_GetTimecode.Standard.Value,         ...
        Output_GetTimecode.SubFramesPerFrame,      ...
        Output_GetTimecode.UserBits );
end
% Get the latency
Output_LatencyTotal = MyClient.GetLatencyTotal().Total;
if verbose
    fprintf( 'Latency: %gs\n',  Output_LatencyTotal);
    for LatencySampleIndex = 1:MyClient.GetLatencySampleCount().Count
        SampleName  = MyClient.GetLatencySampleName( LatencySampleIndex ).Name;
        SampleValue = MyClient.GetLatencySampleValue( SampleName ).Value;
        
        fprintf( '  %s %gs\n', SampleName, SampleValue );
        
    end% for
    
    fprintf( '\n' );
end
% Count the number of subjects
SubjectCount = MyClient.GetSubjectCount().SubjectCount;
if verbose
    fprintf( 'Subjects (%d):\n', SubjectCount );
end

timestr = sprintf('%s %d:%d:%d',char(datetime('today')),Output_GetTimecode.Hours,Output_GetTimecode.Minutes,Output_GetTimecode.Seconds);

if isempty(SubjectPose)
    clear SubjectPose
    SubjectPose(1:SubjectCount) = struct('positionXYZ',zeros(3,1),'anglesXYZ',zeros(3,1),'time',timestr,'name','','occluded',false);
end

for SubjectIndex = 1:SubjectCount
    if verbose
        fprintf( '  Subject #%d\n', SubjectIndex - 1 );
    end
    % Get the subject name
    SubjectName = MyClient.GetSubjectName( SubjectIndex ).SubjectName;
    if verbose
        fprintf( '    Name: %s\n', SubjectName );
    end
    % Set the name
    SubjectPose(SubjectIndex).name = SubjectName;
    
    % Get the global segment translation
    Output_GetSegmentGlobalTranslation = MyClient.GetSegmentGlobalTranslation( SubjectName, SubjectName );
    if verbose
        fprintf( '        Global Translation: (%g, %g, %g) %s\n',               ...
            Output_GetSegmentGlobalTranslation.Translation( 1 ), ...
            Output_GetSegmentGlobalTranslation.Translation( 2 ), ...
            Output_GetSegmentGlobalTranslation.Translation( 3 ), ...
            AdaptBool( Output_GetSegmentGlobalTranslation.Occluded ) );
    end
    if ~Output_GetSegmentGlobalTranslation.Occluded
        SubjectPose(SubjectIndex).positionXYZ = Output_GetSegmentGlobalTranslation.Translation/m2mm;
        SubjectPose(SubjectIndex).occluded = false;
    else
        SubjectPose(SubjectIndex).occluded = true;
    end
    % % Get the global segment rotation in helical co-ordinates
    % Output_GetSegmentGlobalRotationHelical = MyClient.GetSegmentGlobalRotationHelical( SubjectName, SubjectName );
    % if verbose
    %     fprintf( '        Global Rotation Helical: (%g, %g, %g) %s\n',           ...
    %         Output_GetSegmentGlobalRotationHelical.Rotation( 1 ), ...
    %         Output_GetSegmentGlobalRotationHelical.Rotation( 2 ), ...
    %         Output_GetSegmentGlobalRotationHelical.Rotation( 3 ), ...
    %         AdaptBool( Output_GetSegmentGlobalRotationHelical.Occluded ) );
    % end

    % % Get the global segment rotation as a matrix
    % Output_GetSegmentGlobalRotationMatrix = MyClient.GetSegmentGlobalRotationMatrix( SubjectName, SubjectName );
    % if verbose
    %     fprintf( '        Global Rotation Matrix: (%g, %g, %g, %g, %g, %g, %g, %g, %g) %s\n', ...
    %         Output_GetSegmentGlobalRotationMatrix.Rotation( 1 ),               ...
    %         Output_GetSegmentGlobalRotationMatrix.Rotation( 2 ),               ...
    %         Output_GetSegmentGlobalRotationMatrix.Rotation( 3 ),               ...
    %         Output_GetSegmentGlobalRotationMatrix.Rotation( 4 ),               ...
    %         Output_GetSegmentGlobalRotationMatrix.Rotation( 5 ),               ...
    %         Output_GetSegmentGlobalRotationMatrix.Rotation( 6 ),               ...
    %         Output_GetSegmentGlobalRotationMatrix.Rotation( 7 ),               ...
    %         Output_GetSegmentGlobalRotationMatrix.Rotation( 8 ),               ...
    %         Output_GetSegmentGlobalRotationMatrix.Rotation( 9 ),               ...
    %         AdaptBool( Output_GetSegmentGlobalRotationMatrix.Occluded ) );
    % end
    
    % % Get the global segment rotation in quaternion co-ordinates
    % Output_GetSegmentGlobalRotationQuaternion = MyClient.GetSegmentGlobalRotationQuaternion( SubjectName, SubjectName );
    % if verbose
    %     fprintf( '        Global Rotation Quaternion: (%g, %g, %g, %g) %s\n',             ...
    %         Output_GetSegmentGlobalRotationQuaternion.Rotation( 1 ),       ...
    %         Output_GetSegmentGlobalRotationQuaternion.Rotation( 2 ),       ...
    %         Output_GetSegmentGlobalRotationQuaternion.Rotation( 3 ),       ...
    %         Output_GetSegmentGlobalRotationQuaternion.Rotation( 4 ),       ...
    %         AdaptBool( Output_GetSegmentGlobalRotationQuaternion.Occluded ) );
    % end

    % Get the global segment rotation in EulerXYZ co-ordinates
    Output_GetSegmentGlobalRotationEulerXYZ = MyClient.GetSegmentGlobalRotationEulerXYZ( SubjectName, SubjectName );
    if verbose
        fprintf( '        Global Rotation EulerXYZ: (%g, %g, %g) %s\n',                 ...
            Output_GetSegmentGlobalRotationEulerXYZ.Rotation( 1 ),       ...
            Output_GetSegmentGlobalRotationEulerXYZ.Rotation( 2 ),       ...
            Output_GetSegmentGlobalRotationEulerXYZ.Rotation( 3 ),       ...
            AdaptBool( Output_GetSegmentGlobalRotationEulerXYZ.Occluded ) );
    end
    if ~Output_GetSegmentGlobalRotationEulerXYZ.Occluded
        SubjectPose(SubjectIndex).anglesXYZ = Output_GetSegmentGlobalRotationEulerXYZ.Rotation;
        SubjectPose(SubjectIndex).occluded = false;
    else
        SubjectPose(SubjectIndex).occluded = true;
    end
    
end% SubjectIndex


if nargout < 2
    varargout{1} = SubjectPose;
elseif nargout == 2
    varargout{1} = SubjectPose;
    varargout{2} = Output_GetFrameRate;
elseif nargout == 3
    varargout{1} = SubjectPose;
    varargout{2} = Output_GetFrameRate;
    varargout{3} = Output_LatencyTotal;
end
end