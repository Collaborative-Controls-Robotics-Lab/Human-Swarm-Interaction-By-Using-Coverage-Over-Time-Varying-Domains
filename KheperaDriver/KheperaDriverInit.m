% Function for initializing handle to send velocity commands to robot:
% Establishes connection and sends initialization command.
function [K4Drv, varargout] = KheperaDriverInit(robotIP,varargin)
nargoutchk(1,2)
narginchk(1,3)
% Determine number of robots used
numRobots = size(robotIP,1);
% Check if a port option was provided
if nargin > 1
    if ~isempty(varargin{1})
        portArg = varargin{1};
        if length(portArg) == 1
            portVector = portArg*ones(numRobots,1);
        elseif length(portArg) == numRobots
            portVector = portArg;
        else
            error('The port optional argument must be a scalar or match the length of the IP argument.')
        end
    else
        portVector = 4555*ones(numRobots,1);
    end
    if nargin > 2
        verbose = varargin{2};
    end
else
    % Default verbosity
    verbose = 0;
    % Use default port
    portVector = 4555*ones(numRobots,1);
end
% Define initialization messages
initReqMsg = '$K4DRV,REQ,INIT';
initResMsg = '$K4DRV,RES,INIT';
% Number of times to send and read messages
numReq = 2;
% Allocate outputs
K4Drv = cell(numRobots,2);
robotResponse = cell(numRobots,1);
errorList = cell(numRobots,1);
% Loop through and establish connections
for ii = 1:numRobots
    if verbose
        disp(['Creating udp object for robot ''',robotIP(ii,:),'''...'])
    end
    % Create upd object
    K4Drv{ii,1} = udp(['192.168.2.',robotIP(ii,:)],portVector(ii),'Terminator',''); %#ok<TNMLP>
    K4Drv{ii,2} = robotIP(ii,:);
    % Open udp connection
    fopen(K4Drv{ii,1});
    for jj = 1:numReq
        if verbose
            disp(['Sending INIT message to robot ''',robotIP(ii,:),'''...'])
        end
        % Send initialization message to the robots
        fprintf(K4Drv{ii,1},initReqMsg);
        if verbose
            disp(['Waiting for response from robot ''',robotIP(ii,:),'''...'])
        end
        % Listen for response
        robotResponse{ii} = fscanf(K4Drv{ii,1},'%s');
    end
    % Check if the response was received
    if ~strcmp(robotResponse{ii},initResMsg)
        if verbose
            disp(['No or incorrect response from robot ''',robotIP(ii,:),'''...'])
        end
        % No or incorrect response from robot. Add to error list.
        errorList{ii} = robotIP(ii,:);
    end
end
% If there are robots that this not initialize, send an error message
errorList = [errorList{:}];
if ~isempty(errorList)
    errorList = reshape(errorList,2,[]).';
    errorString1 = 'Unable to establish communication with the following robots: [';
    errorString2 = sprintf('%d ',str2num(errorList)); %#ok<ST2NM>
    errorString3 = '\b]';
    keyboard
    error([errorString1,errorString2,errorString3]);
end
% Send received response as optional output argument if requested
if nargout > 1
    varargout{1} = robotResponse;
end
end