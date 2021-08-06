% Vicon class should have methods for initialization (connect), get data
% (pose), and closing (disconnect)
function  ViconClientDisconnect( MyClient, varargin )

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
    verbose = varargin{1};
else
    % Turn on/off debugging statements
    verbose = 0;
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% Disconnect and dispose
MyClient.Disconnect();

% Unload the SDK
if verbose
    fprintf( 'Unloading SDK...' );
end

Client.UnloadViconDataStreamSDK();

if verbose
    fprintf( 'done\n' );
end

rmpath('C:\Users\CCRL\Documents\CCRL\Utilities\ViconClient\Libraries\','C:\Users\CCRL\Documents\CCRL\Utilities\ViconClient\Utilities\')

end