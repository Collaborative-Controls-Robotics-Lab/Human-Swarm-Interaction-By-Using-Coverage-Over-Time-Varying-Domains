% Vicon class should have methods for initialization (connect), get data
% (pose), and closing (disconnect)
function [ varargout ] = ViconClientInit( varargin )

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

if nargin > 0
    verbose = varargin{1};
else
    % Turn on/off debugging statements
    verbose = 0;
end

% A dialog to stop the loop
% MessageBox = msgbox( 'Stop DataStream Client', 'Vicon DataStream SDK' );

% Add path to SDK
addpath('C:\Users\CCRL\Documents\CCRL\Utilities\ViconClient\Libraries\','C:\Users\CCRL\Documents\CCRL\Utilities\ViconClient\Utilities\')

% Load the SDK
if verbose
    fprintf( 'Loading SDK...' );
end
Client.LoadViconDataStreamSDK();
if verbose
    fprintf( 'done\n' );
end

% Program options
HostName = 'localhost:801';

% Make a new client
MyClient = Client();

% Connect to a server
if verbose
    fprintf( 'Connecting to %s ...', HostName );
end
while ~MyClient.IsConnected().Connected
    % Direct connection
    MyClient.Connect( HostName );
    
    % Multicast connection
    % MyClient.ConnectToMulticast( HostName, '224.0.0.0' );
    if verbose
        fprintf( '.' );
    end
end
if verbose
    fprintf( '\n' );
end
% Enable some different data types
MyClient.EnableSegmentData();
% MyClient.EnableMarkerData();
% MyClient.EnableUnlabeledMarkerData();
% MyClient.EnableDeviceData();


if verbose
    fprintf( 'Segment Data Enabled: %s\n',          AdaptBool( MyClient.IsSegmentDataEnabled().Enabled ) );
    % fprintf( 'Marker Data Enabled: %s\n',           AdaptBool( MyClient.IsMarkerDataEnabled().Enabled ) );
    % fprintf( 'Unlabeled Marker Data Enabled: %s\n', AdaptBool( MyClient.IsUnlabeledMarkerDataEnabled().Enabled ) );
    % fprintf( 'Device Data Enabled: %s\n',           AdaptBool( MyClient.IsDeviceDataEnabled().Enabled ) );
    % fprintf( 'Centroid Data Enabled: %s\n',         AdaptBool( MyClient.IsCentroidDataEnabled().Enabled ) );
    % fprintf( 'Marker Ray Data Enabled: %s\n',       AdaptBool( MyClient.IsMarkerRayDataEnabled().Enabled ) );
end
% Set the streaming mode
MyClient.SetStreamMode( StreamMode.ClientPull );
% MyClient.SetStreamMode( StreamMode.ClientPullPreFetch );
% MyClient.SetStreamMode( StreamMode.ServerPush );

% Set the global up axis
MyClient.SetAxisMapping( Direction.Forward, ...
    Direction.Left,    ...
    Direction.Up );    % Z-up
% MyClient.SetAxisMapping( Direction.Forward, ...
%                          Direction.Up,      ...
%                          Direction.Right ); % Y-up

Output_GetAxisMapping = MyClient.GetAxisMapping();
if verbose
    fprintf( 'Axis Mapping: X-%s Y-%s Z-%s\n', Output_GetAxisMapping.XAxis.ToString(), ...
        Output_GetAxisMapping.YAxis.ToString(), ...
        Output_GetAxisMapping.ZAxis.ToString() );
end

% Discover the version number
Output_GetVersion = MyClient.GetVersion();
if verbose
    fprintf( 'Version: %d.%d.%d\n', Output_GetVersion.Major, ...
        Output_GetVersion.Minor, ...
        Output_GetVersion.Point );
end

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

% Determine number of subjects
SubjectCount = MyClient.GetSubjectCount().SubjectCount;


if nargout < 2
    varargout{1} = MyClient;
elseif nargout == 2
    varargout{1} = MyClient;
    varargout{2} = SubjectCount;
elseif nargout == 3
    varargout{1} = MyClient;
    varargout{2} = SubjectCount;
    varargout{3} = Output_GetAxisMapping;
else
    varargout{1} = MyClient;
    varargout{2} = SubjectCount;
    varargout{3} = Output_GetAxisMapping;
    varargout{4} = Output_GetVersion;
end


end