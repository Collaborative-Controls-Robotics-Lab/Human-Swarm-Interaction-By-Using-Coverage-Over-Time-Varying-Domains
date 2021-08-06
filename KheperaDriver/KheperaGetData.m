% Function for sending velocity commands to robot: Transform linear and
% angular velocity to (saturated) wheel commands and, send commands
% wirelessly via udp.
function [D] = KheperaGetData(K4Drv)
nargoutchk(0,1)
% Get number of robots
numberRobots = size(K4Drv,1);
% Check if the arguments are the right dimension
% if length(v) == 1
%     v = v*ones(numberRobots,1);
% elseif length(v) ~= numberRobots
%     error('The argument for linear velocities must be a scalar or an array the length of the number of robots')
% end
% if length(w) == 1
%     w = w*ones(numberRobots,1);
% elseif length(w) ~= numberRobots
%     error('The argument for angular velocities must be a scalar or an array the length of the number of robots')
% end
% Define initialization messages
dataReqMsg = '$K4DRV,REQ,DATA';
% ctrlResMsg = '$K4DRV,RES,CTRL';
% Number of times to send and read messages
numReq = 2;
% Allocate outputs
robotResponse = cell(numberRobots,1);
warningList = cell(numberRobots,1);
warningMessage = '';
% Loop through the handle for every robot
for ii = 1:numberRobots
    for jj = 1:numReq
        % Send control request message to the robots
        fprintf(K4Drv{ii,1},dataReqMsg);
        % Listen for response
        robotResponse{ii} = fscanf(K4Drv{ii,1},'%s');
    end
    %%%%%%%%%%%%%%% Parse %%%%%%%%%%%%%%%%%%%
    D = parsefunctiontobewritten(robotResponse{ii});
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % % Check if the response was received
    % if ~strcmp(robotResponse{ii},ctrlResMsg)
    %     % No or incorrect response from robot. Add to warning list.
    %     warningList{ii} = K4Drv{ii,2};
    % end
end
% % If there are robots that did not reply, send a warning message
% warningList = [warningList{:}];
% if ~isempty(warningList)
%     warningList = reshape(warningList,2,[]).';
%     errorString1 = 'The following robots did not reply as expected: [';
%     errorString2 = sprintf('%d ',str2num(warningList)); %#ok<ST2NM>
%     errorString3 = '\b]';
%     warningMessage = sprintf([errorString1,errorString2,errorString3]);
% end
% % Check if there are output arguments requested
% if nargout > 0
%     varargout{1} = warningMessage;
%     if nargout > 1
%         varargout{2} = robotResponse;
%     end
% end
end