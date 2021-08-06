% Function for sending velocity commands to robot: Transform linear and
% angular velocity to (saturated) wheel commands and, send commands
% wirelessly via udp.
function [] = KheperaSetSpeed(K4Drv, v, w)
nargoutchk(0,0)
% Get number of robots
numberRobots = size(K4Drv,1);
% Check if the arguments are the right dimension
if length(v) == 1
    v = v*ones(numberRobots,1);
elseif length(v) ~= numberRobots
    error('The argument for linear velocities must be a scalar or an array the length of the number of robots')
end
if length(w) == 1
    w = w*ones(numberRobots,1);
elseif length(w) ~= numberRobots
    error('The argument for angular velocities must be a scalar or an array the length of the number of robots')
end
% Define initialization messages
ctrlReqMsg = '$K4DRV,REQ,CTRL,%f,%f';
% ctrlResMsg = '$K4DRV,RES,CTRL';
% Number of times to send and read messages
numReq = 2;
% Allocate outputs
robotResponse = cell(numberRobots,1);
warningList = cell(numberRobots,1);
warningMessage = '';
% Define robot parameters
m2mm = 1000; % Conversion from m to mm
rev2rad = 2*pi; % Conversion from revolutions to radians
% pulse2si = 0.678181/m2mm; % Conversion from pulses to m/s
rad2pulses = (19456*0.01)/rev2rad; % Conversion from pulses by revolutions to pulses by rads
R = 21/m2mm; % wheel radius (m)
L = 105.4/m2mm; % wheel base length (m)
maxPulses = 1200; % pulses/
% Transform velocites to wheel velocities
vel_r = (2*v - w*L)/(2*R); % rad/s
vel_l = (2*v + w*L)/(2*R); % rad/s
% Transform wheel velocities to pulses
vel_r_pulses = floor(vel_r*rad2pulses); % pulses/s
vel_l_pulses = floor(vel_l*rad2pulses); % pulses/s
% Saturate the velocites to recommended max pulses
vel_r_pulses = max(min(vel_r_pulses,maxPulses),-maxPulses);
vel_l_pulses = max(min(vel_l_pulses,maxPulses),-maxPulses);
% Loop through the handle for every robot
for ii = 1:numberRobots
    % Saturate the velocites to recommended max pulses
    vel_r_pulses(ii) = max(min(vel_r_pulses(ii),maxPulses),-maxPulses);
    vel_l_pulses(ii) = max(min(vel_l_pulses(ii),maxPulses),-maxPulses);
    for jj = 1:numReq
        tempMsg = sprintf(ctrlReqMsg,vel_l_pulses(ii),vel_r_pulses(ii));
        % Send control request message to the robots
        fprintf(K4Drv{ii,1},tempMsg);
        % % Listen for response
        % robotResponse{ii} = fscanf(K4Drv{ii,1},'%s');
    end
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