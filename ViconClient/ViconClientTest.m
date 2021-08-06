%% Test of Vicon Client Function
% Tests the capacity to establish a connection with the Vicon client and read
% data before disconnecting.
[ViconClient,N] = ViconClientInit;
try
    %%
    % Define how many times we wish to fetch data.
    maxIter = 2;
    robotXYZ = zeros(3,N,maxIter);
    robotAngles = zeros(3,N,maxIter);
    %%
    % Loop through and grab data.
    robotPose = ViconClientFetch(ViconClient);
    for ii = 1:maxIter
        robotPose = ViconClientFetch(ViconClient,robotPose,1);
        robotXYZ(:,:,ii) = [robotPose.positionXYZ];
        robotAngles(:,:,ii) = [robotPose.anglesXYZ]*180/pi;
    end
    robotXY = robotXYZ(1:2,:,:);
    robotTheta = robotAngles(3,:,:);
    %%
    % Disconnect the client.
    ViconClientDisconnect(ViconClient);
catch e
    ViconClientDisconnect(ViconClient);
    e.rethrow()
end