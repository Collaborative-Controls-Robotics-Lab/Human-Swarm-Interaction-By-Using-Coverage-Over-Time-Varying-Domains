% Class/function for sending velocity commands to robot: establish
% connection, transform linear and angular velocity to (saturated) wheel
% commands, send commands wirelessly (udp?).

clear all

% %% get data from vicon
% [ViconClient,N] = ViconClientInit;
% % Define how many times we wish to fetch data.
% maxIter = 100;
% 
% robotXYZ = zeros(3,N,maxIter);
% angleXYZ = zeros(3,N,maxIter);
% %%
% % Loop through and grab data.
% robotPose = ViconClientFetch(ViconClient);
% for ii = 1:maxIter
%     robotPose = ViconClientFetch(ViconClient,robotPose,1);
%     robotXYZ(:,:,ii) = [robotPose.positionXYZ];
%     angleXYZ(:,:,ii) = [robotpose.anglesXYZ];
% end
% %     robotXY = eye(2,3)*reshape(robotXYZ,3*N,[]);
% %%
% % Disconnect the client.
% ViconClientDisconnect(ViconClient);
% robotXY = eye(2,3)*robotXYZ(:,:,100);
% angleXY = eye(2,3)*angleXYZ(:,:,100);
% %% plug your algorithm
% addpath(genpath('Voronoipolyhedrons'))
% 
% xmin = 0;
% xmax = 1;
% ymin = 0;
% ymax = 1;
% K = 2;
% dt=0.02;
% % T=0;
% boundary = [xmin xmax ymin ymax];
% % P = rand(2,N);
% P = robotXY;
% Q=[0.95 0.05;0.95 0.05];
% 
% [Vtempx,Vtempy,vert,Mass,Cx,Cy,dcdp] = f_TVDC_components(boundary,P',Q);
% 
% % dp = (eye(2*N)-dcdp)\(K*reshape(([Cx.';Cy.']-P),[],1));
% dp = (eye(2*N)+dcdp)*(K*reshape(([Cx.';Cy.']-P),[],1));
% % dp = (eye(2*N)+dcdp+dcdp^2)*(K*reshape(([Cx.';Cy.']-P),[],1));
% dp = reshape(dp,2,[]);
% 
% %% Transform to wheel speed
% %   distance = P_new-P;
% anglediff = atan2((P_new(1,:)-P(1,:)),(P_new(2,:)-P(2,:)))- angleXY(1,:);
% v = dp;
% w = anglediff/dt;
% 
m2mm = 1000;

v = 0; % m/s
w = 0.1; % rads/s

R = 21/m2mm; %wheel radius (mm)
L = 105.4/m2mm; %wheel base length (mm)
% inputs: v, w, forward velocity and angular velocity
vel_r = (2*v + w*L)/(2*R);% angular velocity of right wheel
vel_l = (2*v - w*L)/(2*R);
%% Send data to Robots
% obj = tcpclient('Adress',0000);
% obj = tcpip('localhost',0000,'NetworkRole','Client');

obj = udp('192.168.2.97',4555,'Terminator','');
% obj.Terminator = '';
fopen(obj);
%%
%data = [vel_r;vel_l];
pulse2si = 0.678181/m2mm;
vel_r_pulses = floor(vel_r/pulse2si);
vel_l_pulses = floor(vel_l/pulse2si);
formatSpecRead = '%s';
formatSpec0 = '$K4DRV,REQ,INIT';
for ii = 1:5
    fprintf(obj,formatSpec0);
    feedback0 = fscanf(obj,formatSpecRead);
end
formatSpec = '$K4DRV,REQ,CTRL,%f,%f';
str = sprintf(formatSpec,vel_r_pulses,vel_l_pulses);
for ii = 1:5
    fprintf(obj,str);
end
%%
% feedback = 	(obj,formatSpecRead);
formatSpecIR = '$K4DRV,REQ,IR-P';
for ii = 1:5
    fprintf(obj,formatSpecIR);
    for jj = 1:5
        feedbackIR = fscanf(obj,formatSpecRead);
    end
end
[~,pos] = textscan(feedbackIR,'%s',3,'Delimiter',',');
C = cell2mat(textscan(feedbackIR(16+1:end),'%f',12,'Delimiter',',')).'
%%
fclose(obj);
% fprintf(obj,'B');
%should give back K4 version information, use for testing connection
% fprintf(obj,'D,200,-200');
% send left_motor_speed and right_motor_speed, maximum are 1200 and -1200
% fprintf(obj,'E');
% Request the speed of motors, left and rignt
% fprintf(obj,'G');
% Request Ultrasonic values,1000 means nothing in range, 0 means in 25cm
% fprintf(obj,'H,P,I,D');
% Set gain of PID controller
% fprintf(obj,'K,63,0,0,0,63,0,0,0,63');
% Set LEDs left-red green blue, right-red green blue, back-red green blue
% fprintf(obj,'N');
% Request the vlue of infra-red sensors
% fprintf(obj,'V,arguement');
% Get battery status

