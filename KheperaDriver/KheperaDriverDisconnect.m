% Function for disconnecting the robot driver
% Closes connection from the khepera driver udp objects
function KheperaDriverDisconnect(K4Drv)
% Get number of robots
numberRobots = size(K4Drv,1);
% Loop through the handle for every robot
for ii = 1:numberRobots
    % Close the connection
    fclose(K4Drv{ii,1});
end
end
