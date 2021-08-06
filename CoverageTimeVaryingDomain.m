%% Demo of Coverage Control over Teleoperated Time-Varying Domains
% Executes coverage control laws over a time-varying domain provided by
% user input
%
% Authors: Xiaotian Xu & Yancy Diaz-Mercado
%
% Date Created: 06/17/2019
%
% Copyright (c) 2019
% Collaborative Controls and Robotics Laboratory
% University of Maryland, College Park
%
% All rights reserved.

close all, clear all, clc %#ok<CLALL,DUALC>
% Set constant random number generator seed for reproduceability of results
rng(7)
% Flag to visualize debugging of dcdt
debug_dcdt = 0;
simulate_flag = 0;
%#ok<*UNRCH> <-- No warning from flags
%% Get Robotarium object used to communicate with the robots/simulator
% Set path to Robotarium API
run 'C:\Users\CCRL\Documents\CCRL\Utilities\Robotarium\init'
load('projectiveTransform.mat','H')
%% Add plotting utility to path
% Should now be part of permanent path
% addpath('C:\Users\CCRL\Documents\CCRL\Utilities')
%% Update real robot data
% Set the maximum speed for the computed controller (saturation)
hFigStopLoop = stopLoop;
if simulate_flag
    % Number of robots
    numRobots = 6;
    % Define robot object
    r = Robotarium('NumberOfRobots', numRobots, 'ShowFigure', true);
    controlMaxSpeed = 1e-2;
    controlScaleFactor = 1;
else
    controlMaxSpeed = 1e-2;%0.01;
    % Scale factor reduces magnitude of commands
    controlScaleFactor = 1;%0.002;
    % Add path of utilities to control robots
    Vicon_path = 'C:\Users\CCRL\Documents\CCRL\Utilities\ViconClient';
    K4Driver_path = 'C:\Users\CCRL\Documents\CCRL\Utilities\KheperaDriver';
    addpath(Vicon_path,K4Driver_path)
    %
    disp('Establishing connections with Vicon')
    % Initialize the Vicon Client
    [ViconClient,numTrackables] = ViconClientInit;
    % Fetch initial data to create pose object
    viconPose = ViconClientFetch(ViconClient);
    % Get the names of the trackable objects
    viconNames = {viconPose.name};
    % Determine which objects are named K**
    robotIndicesVicon = contains(viconNames,'K','IgnoreCase',true);
    % Determine the IP address of the robots
    robotIP = cell2mat(viconNames(robotIndicesVicon).');
    robotIP = robotIP(:,2:3);
    % Determine number of robots
    numRobots = size(robotIP,1);
    % Initialize the robots
    verbosity = 0;
    disp('Establishing connections with the robots')
    K4Drv = KheperaDriverInit(robotIP,[],verbosity);
    % Get initial positions and headings
    robotPose = viconPose(robotIndicesVicon);
    robotPositionXYZ0 = [robotPose.positionXYZ];
    robotAnglesXYZ0 = [robotPose.anglesXYZ];
    % robotY = -motion_capture_data(3,:)';
    % Stack them for convenience
    robotXY0 = robotPositionXYZ0(1:2,:);
    robotTheta0 = robotAnglesXYZ0(3,:); % Yaw
    % Store heading angles after transforming from quaternions
    % robotTheta = QuatToEuler(...
    % motion_capture_data(4,:),motion_capture_data(5,:),...
    % motion_capture_data(6,:),motion_capture_data(7,:));
    % Correcting for Matlab/OptiTrack different coordinate sys
    robotHeading0 = ...
        [cos(robotTheta0);
        sin(robotTheta0)];
    % Send a stop to the robots
    KheperaSetSpeed(K4Drv, 0, 0)
    pause(0.1)
    disp('Determining heading bias')
    % Recalibrate to rule out bias
    % Make the robot move for a short time
    KheperaSetSpeed(K4Drv, controlMaxSpeed/2, 0)
    pause(0.5)
    KheperaSetSpeed(K4Drv, 0, 0)
    % Get updated data
    viconPose = ViconClientFetch(ViconClient,viconPose,1);
    robotPose = viconPose(robotIndicesVicon);
    robotPositionXYZ = [robotPose.positionXYZ];
    robotAnglesXYZ = [robotPose.anglesXYZ];
    % robotY = -motion_capture_data(3,:)';
    % Stack them for convenience
    robotXY = robotPositionXYZ(1:2,:);
    robotX = robotXY(1,:).';
    robotY = robotXY(2,:).';
    robotTheta = atan2(robotXY(2,:)-robotXY0(2,:),robotXY(1,:)-robotXY0(1,:));
    disp('Robot vicon heading (degrees):')
    disp(robotTheta0*180/pi)
    disp('Robot actual heading (degrees):')
    disp(robotTheta*180/pi)
    robotThetaBias = atan2(sin(robotTheta0-robotTheta),cos(robotTheta0-robotTheta));
    disp('Robot heading bias (degrees):')
    disp(robotThetaBias*180/pi)
    % Define robot object
    r = Robotarium('NumberOfRobots', numRobots, 'ShowFigure', true,'InitialConditions',[robotXY;robotTheta]);
end
% Select the number of iterations for the experiment.  This value is
% arbitrary
iterations = 1e7;
% Ordered id's for robots
indxRobots = 1:numRobots;
% Define the robot diameter
robotDiameter = r.robot_diameter;
%% Control Parameters and Utilities
% Safety distance (barrier ceritificates)
barrierSafetyDist = 1.5*robotDiameter;
% Gain for coverage control law
controlGain = 6;
% Gain for single integrator to unicycle conversion
si2uniGain = 10;
% Set actuator limits
angularVelLimit = 2*pi;%3*controlMaxSpeed;
% linearVelLimit = 0.49;
linearVelLimit = 0.3;%0.6*controlMaxSpeed; % 0.8 ORIGINAL
% Geneterate a go-to-goal controller utility
positionController = create_si_position_controller;
% Generate a barrier certificate safety wrap around controller utility
barrierCertificate = create_si_barrier_certificate('SafetyRadius', barrierSafetyDist);
% barrierCertificate = @(x,y) x;
% Generate a single integrator model to unicycle model control conversion
si2uni = ...
    create_si_to_uni_mapping('ProjectionDistance', 1*robotDiameter);
...create_si_to_uni_mapping2('LinearVelocityGain', si2uniGain, 'AngularVelocityLimit', pi/4);
% Area integral grids
[x_mesh,y_mesh] = meshgrid(linspace(0,1,50));
locCostIntegrand = @(q1,q2,p1,p2) bsxfun(@minus,q1,p1).^2+bsxfun(@minus,q2,p2).^2;
%% Define the subdomain function
% Define the time-varying coverage subdomain
domainBoundaries = r.boundaries;
domLength = (domainBoundaries(2) - domainBoundaries(1))/2;
domHeight = (domainBoundaries(4) - domainBoundaries(3))/2;
% Obstacle and Goal Definition
Obstacle1 = [domainBoundaries(1) domainBoundaries(1) -0.5;
            0.25 domainBoundaries(4) domainBoundaries(4)];
Obstacle2 = [-0.325 -0.325 domainBoundaries(2)-1.2*[1 1] 0.3250 0.3250;
            domainBoundaries(3) 0.05  0.05  -0.5 -0.5 domainBoundaries(3)];
Obstacle3 = [domainBoundaries([2 1 1 2 2])*1.1, nan, domainBoundaries([2 2 1 1 2]);
             domainBoundaries([4 4 3 3 4])*1.1, nan, domainBoundaries([4 3 3 4 4])];
Goal1 =  [-2.5,-2.5,-1.5,-1.5;-1.7,-0.7,-0.7,-1.7];
Goal2 = [[-2.5,-2.5,-1.5,-1.5]+3;-1.7,-0.7,-0.7,-1.7];
% Define subdomain handle
domRadiusX0 = 0.5; % domLength/2; %domHeight/2;
domRadiusY0 = 0.5; % domHeight/2; %domLength/2;
domainControlGain = 5/diff(domainBoundaries(1:2));
domainMaxSpeed = 9/10*linearVelLimit/controlScaleFactor;
domainCentroid = [0.75;0.75];
referenceCursor = domainCentroid;
% domainCentroidVel = @(x,r) saturate_speed(domainControlGain*(r-x),domainMaxSpeed);
[domainVel,collision_flag] = domainCentroidVel2(domainCentroid,referenceCursor,domainMaxSpeed,domainControlGain,{Obstacle1,Obstacle2,Obstacle3},[domRadiusX0,domRadiusY0]);
hSubdom = @(x) [x(1) + domRadiusX0*[-1 1], x(2) + domRadiusY0*[-1 1]];
hDSubdom = @(v) [v(1)*[1 1],v(2)*[1 1]];
subdomInit = hSubdom(domainCentroid);
% Define the initial configuration on subdomain
posXY0 = [...
    (diff(subdomInit(1:2))/2-robotDiameter)*(2*rand(1,numRobots)-1)+sum(subdomInit(1:2))/2;
    (diff(subdomInit(3:4))/2-robotDiameter)*(2*rand(1,numRobots)-1)+sum(subdomInit(3:4))/2];

%% Visualization Elements
% Load projector calibration matrix
load('projectiveTransform.mat','H')
% Construct an extended display object
extDisp = extendedDisplay(H,r.figure_handle);
% Recover handle for the robots to bring them to the top of the
% visualization stack later
% hRobots = r.figure_handle.Children.Children(1:end-1);
hRobots = gobjects(numRobots,1);
for ii = 1:numRobots
    hRobots(ii) = r.robot_handle{ii};
end
% Set the window to full screen
% set(hFig,'Position',[1921 41 1920 963]);%[1 41 1536 748.8]);%
% Plot the domain
% hSubdomPlot = rectangle('Position',[subdomInit([1 3]) subdomInit([2 4])-subdomInit([1 3])],'EdgeColor','k','LineWidth',2);
hSubdomPlot = extDisp.patch(subdomInit([1 1 2 2]),subdomInit([3 4 4 3]),'EdgeColor','k','LineWidth',2,'FaceColor','none');
% hObstacle1 = rectangle('Position',[-2.55 0.4 2 1.4],'FaceColor',[1 0 0],'EdgeColor','k','LineWidth',2);
% hObstacle2 = rectangle('Position',[-0.325 -1.8 0.65 1.4],'FaceColor',[1 0 0],'EdgeColor','k','LineWidth',2);
% hObstacle3 = rectangle('Position',[1 0.45 0.65 1.4],'FaceColor',[1 0 0],'EdgeColor','k','LineWidth',2);
hObstacle1 = extDisp.patch(Obstacle1(1,:),Obstacle1(2,:),'FaceColor',[1 0 0],'EdgeColor','k','LineWidth',2,'FaceAlpha',0.75);
hObstacle2 = extDisp.patch(Obstacle2(1,:),Obstacle2(2,:),'FaceColor',[1 0 0],'EdgeColor','k','LineWidth',2,'FaceAlpha',0.75);
% hInitialbox = rectangle('Position',[-2.3317 -1.2954 1.5545 2.5908],'FaceColor',[0.25 0.5 1],'EdgeColor','k','LineWidth',2);
% hFinalbox = rectangle('Position',[1.6283 -1.6954 0.5945 3.3908],'FaceColor',[0 1 0.5],'EdgeColor','k','LineWidth',2);
hBlueBox = extDisp.patch(Goal1(1,:),Goal1(2,:),'FaceColor',[0.25 0.5 1],'EdgeColor','k','LineWidth',2,'FaceAlpha',0.2);
hGreenBox = extDisp.patch(Goal2(1,:),Goal2(2,:),'FaceColor',[0 1 0.5],'EdgeColor','k','LineWidth',2,'FaceAlpha',0.2);
% hLinePhi0 = plot(domCenterX+domLength/2*[-1  1],phi0+domCenterY+0.05*domHeight*[0 0],'r','LineWidth',1);
% hLinePhiMag = plot(domCenterX+domLength/2*[-1 1],phiMag+domCenterY+0.05*domHeight*[0 0],'b','LineWidth',1);
% Plot initial positions in 1-d domain
% hInitPos = plot(posXY0(1,:),posXY0(2,:),'.','MarkerSize',30,'LineWidth',3);
hRefCursor = extDisp.plot(referenceCursor(1),referenceCursor(2),'rx','LineWidth',2);
hDomainCentroid = extDisp.plot(domainCentroid(1),domainCentroid(2),'ro','LineWidth',2);
hRefCursor2DomCentroid = extDisp.plot(referenceCursor(1)-domainCentroid(1),referenceCursor(2)-domainCentroid(2),'r--','LineWidth',2);
% Plot tessellation as patches of area under the density curve
colorMat = 0.75*hsv(numRobots);
hVoronoi = gobjects(2,numRobots);
for ii = 1:numRobots
    hVoronoi(:,ii) = extDisp.patch(nan,nan,'FaceColor',colorMat(ii,:),'FaceAlpha',0.25);
end
% Plot the density curve
% hDensity = plot(domLineX,phiMag*hPhi((domLineX-domLineX(1))/domLength)+domCenterY,'Color',0.5*[1 1 1],'LineWidth',2);
% Plot the center of mass for each cell (target location)
hCM = extDisp.plot(nan,nan,'x','MarkerSize',10,'LineWidth',3);
% Bring robot visualization to top of ui stack
uistack(hRobots,'top')
hFigLocCost = figure('Position',[769.8 41.8 766.4 328.8]);
yyaxis left
hAggregateError = plot(nan,nan,'LineWidth',2);
grid minor
xlabel('$$t$$ [s]','FontSize',14,'Interpreter','latex')
ylabel('$$\|p-c\|$$ [m]','FontSize',14,'Interpreter','latex')
hLocAxes = gca;
xlim(hLocAxes,[0 100])
yyaxis right
hLocCost = plot(nan,nan,'LineWidth',2);
ylabel('$$\mathcal{H}(p,t)$$/Total Area [m/m]','FontSize',14,'Interpreter','latex')
if debug_dcdt
    figure(hFig)
    hDebugDcdt = extDisp.quiver(nan,nan,nan,nan,0,'LineWidth',3);
end
% %% Plot projector content
% hFigProjector = figure('units','normalized','position',[-1 0 1 1],'MenuBar','none','color','w','WindowState','fullscreen');
% % Create axes
% hAxesProjector = axes('Units', 'normalized', 'Position', [0 0 1 1]);
% axis(hAxesProjector,[0 1 0 1])
% axis manual
% axis off
% hold on
% % Transform objects
% subdomInitProjector = projectiveTransform([subdomInit([1 1 2 2]);subdomInit([3 4 4 3])],H);
% Obstacle1Projector = projectiveTransform(Obstacle1,H);
% Obstacle2Projector = projectiveTransform(Obstacle2,H);
% Goal1Projector = projectiveTransform(Goal1,H);
% Goal2Projector = projectiveTransform(Goal2,H);
% 
% hSubdomPlotProjector = patch('XData',subdomInitProjector(1,:),'YData',subdomInitProjector(2,:),'EdgeColor','k','LineWidth',2','FaceColor','none');
% hObstacle1Projector = patch('XData',Obstacle1Projector(1,:),'YData',Obstacle1Projector(2,:),'FaceColor',[1 0 0],'EdgeColor','k','LineWidth',2,'FaceAlpha',0.75);
% hObstacle2Projector = patch('XData',Obstacle2Projector(1,:),'YData',Obstacle2Projector(2,:),'FaceColor',[1 0 0],'EdgeColor','k','LineWidth',2,'FaceAlpha',0.75);
% hBlueBoxProjector = patch('XData',Goal1Projector(1,:),'YData',Goal1Projector(2,:),'FaceColor',[0.25 0.5 1],'EdgeColor','k','LineWidth',2,'FaceAlpha',0.2);
% hGreenBoxProjector = patch('XData',Goal2Projector(1,:),'YData',Goal2Projector(2,:),'FaceColor',[0 1 0.5],'EdgeColor','k','LineWidth',2,'FaceAlpha',0.2);
% 
% referenceCursorProjector = projectiveTransform(referenceCursor(:),H);
% domainCentroidProjector = projectiveTransform(domainCentroid(:),H);
% 
% hRefCursorProjector = plot(referenceCursorProjector(1),referenceCursorProjector(2),'rx','LineWidth',2);
% hDomainCentroidProjector = plot(domainCentroidProjector(1),domainCentroidProjector(2),'ro','LineWidth',2);
% hRefCursor2DomCentroidProjector = plot([referenceCursorProjector(1) domainCentroidProjector(1)],[referenceCursorProjector(2) domainCentroidProjector(2)],'r--','LineWidth',2);
% 
% 
% hVoronoiProjector = gobjects(numRobots,1);
% for ii = 1:numRobots
%     hVoronoiProjector(ii) = patch(nan,nan,colorMat(ii,:),'FaceAlpha',0.25);
% end
% hCMProjector = plot(nan,nan,'x','MarkerSize',10,'LineWidth',3);


%% TUIO Variables
% Allocate memory
% Number of reference cursors stored
nMemory = 1;
% Available and transitioning slot ID hardcoded constants values
AVAILABLE = 200; TRANSITIONING = 500;
% Id number of currently plotted objects
currentIds = AVAILABLE*ones(1,nMemory);
 
%% Java Libraries
% Add the java libraries to the dynamic class path if not already on it
if ~any(strcmp([pwd,'\javaosc.jar'],javaclasspath('-dynamic')))
    disp('Adding ''javaosc.jar'' to class path...')
    % Library for OSC message handling. (TUIO messages use OSC messages)
    javaaddpath javaosc.jar
end
if ~any(strcmp([pwd,'\yajtl.jar'],javaclasspath('-dynamic')))
    disp('Adding ''yajtl.jar'' to class path...')
    % Library for creating TUIO client/servers and even handling. (Cursors)
    javaaddpath yajtl.jar
end
if ~any(strcmp([pwd,'\DynamicDensityTUIO.jar'],javaclasspath('-dynamic')))
    disp('Adding ''DynamicDensityTUIO.jar'' to class path...')
    % Library for running event listener on separate thread with public
    % methods for getting cursor information (e.g., (x,y), id, start time)
    javaaddpath DynamicDensityTUIO.jar
end
% Create a new TUIO thread to listen for TUIO events (e.g., new cursor)
javaThreadObject = javaObject('dynamic.density.tuio.NewTuioThread');
% Start a new thread
newThread = javaMethod('startTh', javaThreadObject);

%% Loop
% Set flag to demo phases
%       -Phase 1: go to initial position in 1-d domain (LED: red)
%       -Phase 2: set the heading of the robot to the right (LED: yellow)
%       -Phase 3: run coverage control on 1-d domain (LED: green)
%       -Phase 4: display sucess metrics (TODO)
algorithmPhase = 1;
tPhase1 = 1;
% Set LED for phase 1 (red)
r.set_left_leds(1:numRobots,[255;0;0]*ones(1,numRobots));
r.set_right_leds(1:numRobots,[255;0;0]*ones(1,numRobots));
% Allocate position matrix and vector of locational cost
positions = nan(2,numRobots,iterations);
aggregateError = nan(iterations, 1);
VoronoiCellInfoOverTime = cell(numRobots,iterations);
centroidsOverTime = nan(2,numRobots, iterations);
locCost = nan(iterations, 1);
timeVector = nan(iterations, 1);
% Voronoi tessellation parameters
powerDiagramWeights = zeros(numRobots, 1); % For power diagram (zeros -> Standard Voronoi cells)
% Iterate for the previously specified number of iterations
for t = 1:iterations
    if hFigStopLoop.Stop()
        break
    end
    
    if simulate_flag
        % Retrieve the most recent poses from the Robotarium.  The time delay is
        % approximately 0.033 seconds
        x = r.get_poses();
        % Rename for convenience
        robotXY = x(1:2,:);
        robotTheta = x(3,:);
    else
        % Grabbing new data
        viconPose = ViconClientFetch(ViconClient,viconPose,1);
        robotPose = viconPose(robotIndicesVicon);
        robotPositionXYZ = [robotPose.positionXYZ];
        robotAnglesXYZ = [robotPose.anglesXYZ];
        % robotY = -motion_capture_data(3,:)';
        % Stack them for convenience
        robotXY = robotPositionXYZ(1:2,:);
        robotX = robotXY(1,:).';
        robotY = robotXY(2,:).';
        robotTheta = robotAnglesXYZ(3,:)-robotThetaBias; % Yaw
        % Set the updated pose for the robots and visualize
        r.set_poses([robotXY;robotTheta]);
        r.draw_robots();
    end
    % Go through algorithm phases
    if algorithmPhase == 1 % Go to initial positions
        % Go to goal
        dxi = controlScaleFactor*positionController(robotXY,posXY0);
        % Add collision avoidance
        dxi = barrierCertificate(dxi,[robotXY;robotTheta]);
        % Convert to unicycle model
        dxu = si2uni(dxi,[robotXY;robotTheta]);
        % Check if they've converged
        robotsDone = hypot(robotXY(1,:)-posXY0(1,:),robotXY(2,:)-posXY0(2,:))<5e-1;
        % robotsDone = vecnorm(dxi)<5e-1;
        % Set LED's to yellow if they've converged and red if the have not
        r.set_left_leds(indxRobots(robotsDone),[255;255;0]*ones(1,nnz(robotsDone)));
        r.set_right_leds(indxRobots(robotsDone),[255;255;0]*ones(1,nnz(robotsDone)));
        r.set_left_leds(indxRobots(~robotsDone),[255;0;0]*ones(1,nnz(~robotsDone)));
        r.set_right_leds(indxRobots(~robotsDone),[255;0;0]*ones(1,nnz(~robotsDone)));
        if all(robotsDone)
            % Move to phase 2 if they are all done
            algorithmPhase = 2;
            tPhase2 = t + 1;
            % Define current bounding box
            boundingBox = [...
                subdomInit([1 3]);
                subdomInit([1 4]);
                subdomInit([2 4]);
                subdomInit([2 3])]; % the bounding box in clockwise order
            
            % Get the Voronoi tessellation (from Matlab File Exchange: Fast Voronoi)
            [V,C,polyXY] = power_bounded(robotXY(1,:).',robotXY(2,:).', powerDiagramWeights, boundingBox);
            % Compute Voronoi cell area integrals
            polyV = polyshape(polyXY(1,:),polyXY(2,:),'Simplify',false);
            m = area(polyV,1:numRobots);
            m = abs(m);
            c = zeros(2,numRobots);
            [c(1,:),c(2,:)] = centroid(polyV,1:numRobots);
            extDisp.set(hCM,c(1,:),c(2,:))
            for ii = 1:numRobots
                extDisp.set(hVoronoi(:,ii),V(C{ii},1),V(C{ii},2))
            end
            % Start a clock for the domain function
            timer = tic;
            timerNewTime = toc(timer);
            drawnow
        end
    elseif algorithmPhase == 2 % Coverage control
        % Update TUIO cursor information
        newCursorIds = javaThreadObject.getId()';
        if any(newCursorIds>=AVAILABLE)
            disp('Excedeed maximum ID allocation, please reset TUIO client app...')
        end
        % Positions
        newCursorX =  (domainBoundaries(1)+(domainBoundaries(2)-domainBoundaries(1)).*javaThreadObject.getX()');
        newCursorY =  (domainBoundaries(4)-(domainBoundaries(4)-domainBoundaries(3)).*javaThreadObject.getY()');
        %% Implement TUIO cursor logic %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Check if a new cursor was added.
        unmatchedCursorIds = setdiff(newCursorIds,currentIds);
        if ~isempty(unmatchedCursorIds)
            % There are unaccounted for cursors being broadcasted, store
            % them in a new vector for ease of handling.
            availableIdSlots = ...
                find((currentIds>=AVAILABLE)&(currentIds<TRANSITIONING));
            % Assign all unmatched cursors
            if length(availableIdSlots)<length(unmatchedCursorIds)
                disp(['More cursors active than Gaussians available. ',...
                    'Discarding some...'])
            end
            maxIter = min(length(unmatchedCursorIds),length(availableIdSlots));
            for iAdd = 1:maxIter
                % Cursor not accounted for. Add to pool.
                currentIds(availableIdSlots(iAdd)) = unmatchedCursorIds(iAdd);
                referenceCursor(:,availableIdSlots(iAdd)) = [...
                    newCursorX(iAdd);
                    newCursorY(iAdd)];
            end
        end
        %
        %------------------------------------------------------------------
        % Check if a TUIO cursor is transtitioning out of the pool
        if any(currentIds<AVAILABLE)
            % Determine if any and which TUIO cursors are obsolete
            oldCursorIds = setdiff(currentIds,...
                [newCursorIds,AVAILABLE,TRANSITIONING]);
            [~,transitionIndices] = ismember(oldCursorIds,currentIds);
            if ~isempty(oldCursorIds)
                % keyboard
                for kTransition = 1:length(transitionIndices)
                    % Change ID to transitioning and removal transition
                    % time, if transition set to final time (not already
                    % transitioning)
                    currentIds(transitionIndices(kTransition)) = TRANSITIONING;
                end
            end
        end
        %
        %------------------------------------------------------------------
        % Check if any transitioning TUIO cursor needs to be removed
        if any(currentIds==TRANSITIONING)
            % keyboard
            % If transition more than few seconds, it doesn't contribute,
            % remove it from the pool of Gaussians and set to final time
            transitioningIds = (currentIds==TRANSITIONING);
            currentIds(transitioningIds) = AVAILABLE;
        end
        %
        %------------------------------------------------------------------
        % Change reference centroid of all active Gaussians
        if any(currentIds<AVAILABLE)
            % Get the matching Ids from the current pool of Gaussians
            [matchedIds,kMove] = ismember(currentIds,newCursorIds);
            % Sanity check
            if length(matchedIds(matchedIds==1))~=length(newCursorIds)
                disp(['Something wrong in the TUIO logic! ',...
                    'Did not trim/add the right Ids'])
            end
            % Update the position of the centroids
            for iMove = 1:nMemory
                if kMove(iMove) ~= 0 ...
                        && iMove<=numel(newCursorX) && iMove<=numel(newCursorY)
                    referenceCursor(:,iMove) = [...
                        newCursorX(kMove(iMove));
                        newCursorY(kMove(iMove))];
                end
                % Check if we must continue this loop
                if all(kMove(iMove+1:end)==0)
                    break
                end
            end
        end
        %% End TUIO cursor logic %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Timer
        tau = (t-tPhase2+1);
        timerPrevTime = timerNewTime;
        timerNewTime = toc(timer);
        % Update the subdomain position
        subDom = hSubdom(domainCentroid);
        % subdomProjector = projectiveTransform([subDom([1 1 2 2]);subDom([3 4 4 3])],H);
        % domainVel = domainCentroidVel(domainCentroid,referenceCursor);
        [domainVel,collision_flag] = domainCentroidVel2(domainCentroid,referenceCursor,domainMaxSpeed,domainControlGain,{Obstacle1,Obstacle2,Obstacle3},[domRadiusX0,domRadiusY0]);
        dSubDom = hDSubdom(domainVel);
        % Update subdomain visualization
        if collision_flag
            extDisp.set(hSubdomPlot,subDom([1 1 2 2]),subDom([3 4 4 3]),'EdgeColor','r')
            % extDisp.set(hSubdomPlotProjector,'XData',subdomProjector(1,:),'YData',subdomProjector(2,:),'EdgeColor','r')
        else
            extDisp.set(hSubdomPlot,subDom([1 1 2 2]),subDom([3 4 4 3]),'EdgeColor','k')
            % extDisp.set(hSubdomPlotProjector,'XData',subdomProjector(1,:),'YData',subdomProjector(2,:),'EdgeColor','k')
        end
        if simulate_flag
            dt = r.time_step;
            timeVector(tau) = timerNewTime;
        else
            dt = timerNewTime - timerPrevTime;
            timeVector(tau) = timerNewTime;
        end
        extDisp.set(hDomainCentroid,domainCentroid(1),domainCentroid(2))
        extDisp.set(hRefCursor,referenceCursor(1),referenceCursor(2))
        extDisp.set(hRefCursor2DomCentroid,[referenceCursor(1) domainCentroid(1)],[referenceCursor(2) domainCentroid(2)])
        
        % domainCentroidProjector = projectiveTransform(domainCentroid(:),H);
        % referenceCursorProjector = projectiveTransform(referenceCursor(:),H);
        % set(hDomainCentroidProjector,'XData',domainCentroidProjector(1),'YData',domainCentroidProjector(2))
        % set(hRefCursorProjector,'XData',referenceCursorProjector(1),'YData',referenceCursorProjector(2))
        % set(hRefCursor2DomCentroidProjector,'XData',[referenceCursorProjector(1) domainCentroidProjector(1)],'YData',[referenceCursorProjector(2) domainCentroidProjector(2)])
        % Integrate position numerically
        domainCentroid = domainCentroid + dt*domainVel;
        % Store the positions at the current time
        positions(:,:,tau) = robotXY;
        % Compute the Voronoi tessellation
        boundingBox = [...
            subDom([1 3]);
            subDom([1 4]);
            subDom([2 4]);
            subDom([2 3])]; % the bounding box in clockwise order
        % Get the Voronoi tessellation (from Matlab File Exchange: Fast Voronoi)
        [V,C,polyXY,AdjacencyXY] = power_bounded(robotXY(1,:).',robotXY(2,:).', powerDiagramWeights, boundingBox);
        % Get the Voronoi cells as polyshapes
        polyV = polyshape(polyXY(1,:),polyXY(2,:),'Simplify',false);
        % Compute Voronoi cell area integrals using polyshape
        m = area(polyV,1:numRobots);
        m = abs(m);
        c = zeros(2,numRobots);
        [c(1,:),c(2,:)] = centroid(polyV,1:numRobots);
        % Determine adjacency and find the neighbor boundary vertex
        % indices. Also determine Voronoi cells in contact with the
        % subdomain and find the vertex indices
        [adjacency, iDVij, boundCells, iDSi] = voronoiAdjacency(AdjacencyXY ,subDom);
        % Allocate partial derivatives, normal array, and cost variables
        dcdt = zeros(2,numRobots);
        dcdp = zeros(2*numRobots);
        normalVectors = zeros(2,numRobots,numRobots);
        interiorCellBoundary = zeros(2,numRobots,numRobots);
        aggregateError(tau) = 0;
        locCost(tau) = 0;
        VoronoiCellInfoOverTime(:,tau) = AdjacencyXY;
        centroidsOverTime(:,:,tau) = c;
        for ii = 1:numRobots
            % Relabel for readability
            pii = robotXY(:,ii);
            cii = c(:,ii);
            mii = m(ii);
            if ii < numRobots
                for jj = ii+1:numRobots
                    if adjacency(ii,jj) ~= 0
                        % Relabel for readability
                        pjj = robotXY(:,jj);
                        cjj = c(:,jj);
                        mjj = m(jj);
                        normpij = norm(pii-pjj);
                        normalOutward = (pjj-pii)/normpij;
                        normalVectors(:,jj,ii) =  normalOutward; % i's Neighbor j
                        normalVectors(:,ii,jj) = -normalOutward; % j's Neighbor i
                        Nij = normalOutward*normalOutward.';
                        % Get the vertices for the boundary
                        V1 = AdjacencyXY{ii}(iDVij(ii,jj,1),:).';
                        V2 = AdjacencyXY{ii}(iDVij(ii,jj,2),:).';
                        V2_minus_V1 = V2 - V1;
                        interiorCellBoundary(:,jj,ii) = V2_minus_V1;
                        interiorCellBoundary(:,ii,jj) = V2_minus_V1;
                        normV1V2 = norm(V2_minus_V1);
                        % for dcidpj
                        V1_minus_cii = V1 - cii;
                        V1_minus_pjj = V1 - pjj;
                        % for dcjdpi
                        V1_minus_cjj = V1 - cjj;
                        V1_minus_pii = V1 - pii;
                        % dcidpj
                        dcidpj = -normV1V2/(mii*normpij)*...
                            (V1_minus_cii*V1_minus_pjj.'   ...
                            +0.5*(V1_minus_cii*V2_minus_V1.'+...
                            V2_minus_V1*V1_minus_pjj.') ...
                            +1/3*(V2_minus_V1*V2_minus_V1.') ...
                            );
                        % Specular reflection
                        dcidpi = dcidpj*(2*Nij-eye(2));
                        % Store
                        dcdp(2*ii-1:2*ii,2*jj-1:2*jj) = dcidpj;
                        dcdp(2*ii-1:2*ii,2*ii-1:2*ii) = ...
                            dcdp(2*ii-1:2*ii,2*ii-1:2*ii) + dcidpi;
                        % dcjdpi
                        dcjdpi = -normV1V2/(mjj*normpij)*...
                            (V1_minus_cjj*V1_minus_pii.'   ...
                            +0.5*(V1_minus_cjj*V2_minus_V1.'+...
                            V2_minus_V1*V1_minus_pii.') ...
                            +1/3*(V2_minus_V1*V2_minus_V1.') ...
                            );
                        % Specular reflection
                        dcjdpj = dcjdpi*(2*Nij-eye(2));
                        % Store
                        dcdp(2*jj-1:2*jj,2*ii-1:2*ii) = dcjdpi;
                        dcdp(2*jj-1:2*jj,2*jj-1:2*jj) = ...
                            dcdp(2*jj-1:2*jj,2*jj-1:2*jj) + dcjdpj;
                    end
                end
            end
            % Compute dcidt
            if boundCells(ii) > 0
                jj = find(iDSi{ii},1,'first');
                while jj <= length(iDSi{ii})
                    if iDSi{ii}(jj) ~= 0
                        V1 = AdjacencyXY{ii}(jj,:).';
                        if jj < length(iDSi{ii})
                            if iDSi{ii}(jj+1) ~=0
                                V2 = AdjacencyXY{ii}(jj+1,:).';
                            else
                                jj = jj + 1;
                                continue
                            end
                        else
                            if iDSi{ii}(1) ~=0
                                V2 = AdjacencyXY{ii}(1,:).';
                            else
                                jj = jj + 1;
                                continue
                            end
                        end
                        % Vertices are provided in cw orientation. Use
                        % vector orientation to determine which boundary
                        % derivative to use, within boundAngTol degrees
                        boundAngTol = 1;
                        exteriorBoundary = V2-V1;
                        normExteriorBoundary = norm(exteriorBoundary);
                        boundaryDir = exteriorBoundary/normExteriorBoundary;
                        boundAngle = atan2d(boundaryDir(2),boundaryDir(1));
                        interiorV1s = AdjacencyXY{ii}(nonzeros(iDVij(ii,:,1)),:);
                        interiorV2s = AdjacencyXY{ii}(nonzeros(iDVij(ii,:,2)),:);
                        matchingBoundary = ...
                            (ismembertol(interiorV1s,V1','ByRows',true)&ismembertol(interiorV2s,V2','ByRows',true))...
                            |(ismembertol(interiorV1s,V2','ByRows',true)&ismembertol(interiorV2s,V1','ByRows',true));
                        % Check that the boudary vertices dont belong to an
                        % interior dVij
                        if ~any(matchingBoundary)
                            % Apply unit outward normal to derivative
                            if abs(boundAngle-90) < boundAngTol % Left boundary
                                dqdt = -dSubDom(1);
                            elseif abs(boundAngle+90) < boundAngTol % Right boundary
                                dqdt = dSubDom(2);
                            elseif abs(boundAngle-180) < boundAngTol || abs(boundAngle+180) < boundAngTol % Bottom boundary
                                dqdt = -dSubDom(3);
                            elseif abs(boundAngle) < boundAngTol % Top boundary
                                dqdt = dSubDom(4);
                            else
                                dqdt = 0;
                                warning('Voronoi cell ''Subdomain'' boundary not aligned with subdomain')
                            end
                        else % Boundary aligned with interior boundary
                            dqdt = 0;
                        end
                        %
                        dcidt_analy = ...
                            0.5*dqdt/mii*((V1-cii)+(V2-cii))*norm(V1-V2);
                        % int (q-ci) dq/dt dq / mi
                        dcdt(:,ii) = dcdt(:,ii) + dcidt_analy;%
                        if debug_dcdt
                            set(hDebugDcdt(1),'XData',V1(1),'YData',V1(2),'UData',V2(1)-V1(1),'VData',V2(2)-V1(2))
                            XYUVData = extDisp.projectiveTransform([V1,V2]);
                            set(hDebugDcdt(2),'XData',XYUVData(1,1),'YData',XYUVData(2,1),'UData',XYUVData(1,2)-XYUVData(1,1),'VData',XYUVData(2,2)-XYUVData(2,1))
                            %disp('Paused. Press any key...')
                            %pause
                            q_line = [linspace(V1(1),V2(1));
                                linspace(V1(2),V2(2))];
                            norm_dq = norm(q_line(:,1)-q_line(:,2));
                            q_minus_cii = bsxfun(@minus,q_line,cii);
                            dcidt_num = sum(q_minus_cii,2)*dqdt*norm_dq / mii;
                            disp(['dqdt = ',num2str(dqdt)])
                            disp(['norm(dcidt_num-dcidt_analy) = ',...
                                num2str(norm(dcidt_num-dcidt_analy))])
                            keyboard
                        end
                    end
                    jj = jj + 1;
                end
            end
            % Integrate for locational cost
            aggregateError(tau) = norm(robotXY(:)-c(:));% + trapz(q_line, abs(q_line - p(ii)).*phi_line);
            % 
            locCost(tau) = locCost(tau) + polygonIntegral(@(x,y)locCostIntegrand(x,y,pii(1),pii(2)),AdjacencyXY{ii},x_mesh,y_mesh)/(diff(subDom(1:2))*diff(subDom(3:4)));% + trapz(q_line, abs(q_line - p(ii)).*phi_line);
            if tau*r.time_step < 100
                tauIndLoc = (1:tau);
                tauLocCost = timeVector(tauIndLoc);%tauIndLoc*r.time_step;
                set(hAggregateError, 'XData', tauLocCost, 'YData',aggregateError(tauIndLoc));
                set(hLocCost, 'XData', tauLocCost, 'YData',locCost(tauIndLoc));
            else
                tauIndLoc = (tau-floor(100/r.time_step):tau);
                tauLocCost = timeVector(tauIndLoc);%tauIndLoc*r.time_step;
                xlim(hLocAxes,tauLocCost([1 end]))
                set(hAggregateError, 'XData', tauLocCost, 'YData',aggregateError(tauIndLoc));
                set(hLocCost, 'XData', tauLocCost, 'YData',locCost(tauIndLoc));
            end
            % Update Voronoi tessellation visualization
            extDisp.set(hVoronoi(:,ii),V(C{ii},1),V(C{ii},2))
            % VProjector = projectiveTransform([V(C{ii},1).';V(C{ii},2).'],H);
            % set(hVoronoiProjector(ii),'XData',VProjector(1,:),'YData',VProjector(2,:))
        end
        % Update center of mass visualization
        extDisp.set(hCM,c(1,:),c(2,:))
        % CMProjector = projectiveTransform(c,H);
        % set(hCMProjector,'XData',CMProjector(1,:),'YData',CMProjector(2,:))
        % Compute coverage control law (note: it's transposed)
        % dp = ((eye(2*numRobots)))*(controlGain*(c(:)-robotXY(:))+dcdt(:));%);
        dp = ((eye(2*numRobots)+dcdp))*(controlGain*(c(:)-robotXY(:))+dcdt(:));
        % dp = ((eye(2*numRobots)+dcdp))*(controlGain*(c(:)-robotXY(:)));%);
        % dp = ((eye(2*numRobots)+dcdp+dcdp^2))*(controlGain*(c(:)-robotXY(:))+dcdt(:));%);
        % dp = ((eye(2*numRobots)+dcdp+dcdp^2+dcdp^3))*(controlGain*(c(:)-robotXY(:))+dcdt(:));%);
        % dp = ((eye(2*numRobots)+dcdp+dcdp^2+dcdp^3+dcdp^4+dcdp^5+dcdp^6))*(controlGain*(c(:)-robotXY(:))+dcdt(:));%);
        % dp = ((eye(2*numRobots)-dcdp))\(controlGain*(c(:)-robotXY(:)));
        % dp = (eye(2*numRobots)-dcdp)\(controlGain*(c(:)-robotXY(:))+dcdt(:));
        dxi = controlScaleFactor*reshape(dp,2,[]);
        % Scale and Stack control commands to be sent
        dxu = si2uni(dxi,[robotXY;robotTheta]);
        % Set LEDs to green if they've converged and yellow if the have not
        robotsDone = hypot(c(1,:)-robotXY(1,:),c(2,:)-robotXY(2,:))<1e-2;
        r.set_left_leds(indxRobots(robotsDone),[0;255;0]*ones(1,nnz(robotsDone)));
        r.set_right_leds(indxRobots(robotsDone),[0;255;0]*ones(1,nnz(robotsDone)));
        r.set_left_leds(indxRobots(~robotsDone),[255;255;0]*ones(1,nnz(~robotsDone)));
        r.set_right_leds(indxRobots(~robotsDone),[255;255;0]*ones(1,nnz(~robotsDone)));
    end
    %% Send velocities to agents
    % Saturate the control to avoid actuator limits
    dxu(1,dxu(1,:)>linearVelLimit) = linearVelLimit;
    dxu(1,dxu(1,:)<-linearVelLimit) = -linearVelLimit;
    dxu(2,dxu(2,:)>angularVelLimit) = angularVelLimit;
    dxu(2,dxu(2,:)<-angularVelLimit) = -angularVelLimit;
    if simulate_flag
        % Set velocities of agents 1,...,N
        r.set_velocities(1:numRobots, dxu);
        % Send the previously set velocities to the agents.  This function must be called!
        r.step();
    else
        % Send velocity updates to robots
        KheperaSetSpeed(K4Drv, dxu(1,:), dxu(2,:))
        % KheperaSetSpeed(K4Drv,0,0)
    end
end
%% Terminate
javaMethod('stopTh', javaThreadObject,newThread)
clear javaThreadObject newThread
% Remove libraries from dynamic path
if any(strcmp([pwd,'\javaosc.jar'],javaclasspath('-dynamic')))
    disp('Removing ''javaosc.jar'' from class path...')
    javarmpath javaosc.jar
end
if any(strcmp([pwd,'\yajtl.jar'],javaclasspath('-dynamic')))
    disp('Removing ''yajtl.jar'' from class path...')
    javarmpath yajtl.jar
end
if any(strcmp([pwd,'\DynamicDensityTUIO.jar'],javaclasspath('-dynamic')))
    disp('Removing ''DynamicDensityTUIO.jar'' from class path...')
    javarmpath DynamicDensityTUIO.jar
end
hFigStopLoop.Clear();
clear hFigStopLoop;
if ~simulate_flag
    % Stop all robots at the end
    KheperaSetSpeed(K4Drv, 0, 0)
    % Disconnect the Vicon client
    ViconClientDisconnect(ViconClient)
    % Disconnect the Khepera driver client
    KheperaDriverDisconnect(K4Drv)
    % Close the "stop" window it is still open

    % fullpathdir = [pwd,'\data\'];
    % filename = unique_filename([fullpathdir,domainType,'Coverage_4Robots']);
    % filenameFig1 = unique_filename_fig([fullpathdir,domainType,'Coverage_4Robots_RobotFig']);
    % filenameFig2 = unique_filename_fig([fullpathdir,domainType,'Coverage_4Robots_LocCostFig']);
    % hasBeenRecorded = ~isnan(timeVector);
    % positions = positions(:,:,hasBeenRecorded);
    % VoronoiCellInfoOverTime = VoronoiCellInfoOverTime(:,hasBeenRecorded);
    % centroidsOverTime = centroidsOverTime(:,:,hasBeenRecorded);
    % timeVector = timeVector(hasBeenRecorded);
    % locCost = locCost(hasBeenRecorded);
    % aggregateError = aggregateError(hasBeenRecorded);
    % save(filename,'positions','timeVector','locCost','aggregateError','VoronoiCellInfoOverTime','centroidsOverTime');
    % savefig(hFig,filenameFig1)
    % savefig(hFigLocCost,filenameFig2)
end
% We should call r.call_at_scripts_end() after our experiment is over!
r.debug();