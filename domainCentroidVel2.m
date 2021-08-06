function [v,collision_flag] = domainCentroidVel2(x,r,domainMaxSpeed,domainControlGain,Obstacles,domainDims)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% Determine velocity as saturated first order response
v = saturate_speed(domainControlGain*(r-x),domainMaxSpeed);

% Determine direction of velocity vector
% vtheta = atan2(v(2),v(1));

% Determine boundary sensor locations
% sensorAngles = linspace(0,2*pi,numberSensors+1)+pi/4;
% sensorAngles = sensorAngles(1:end-1);
% cossa = cos(sensorAngles); sinsa = sin(sensorAngles);
% sensorX = domainDims(1)*(abs(cossa).*cossa + abs(sinsa).*sinsa)+x(1);
% sensorY = domainDims(2)*(abs(cossa).*cossa - abs(sinsa).*sinsa)+x(2);
sensorX = domainDims(1)*[1  1  0 -1 -1 -1  0  1]+x(1);
sensorY = domainDims(2)*[0  1  1  1  0 -1 -1 -1]+x(2);
numberSensors = length(sensorX);

collision_flag = false;

for ii = 1:numberSensors
    for jj = 1:length(Obstacles)
        [dist2poly,inpoly_x,inpoly_y] = p_poly_dist(sensorX(ii),sensorY(ii),Obstacles{jj}(1,:),Obstacles{jj}(2,:));
        if (dist2poly<0)
            sensor = [sensorX(ii);sensorY(ii)];
            inpoly = [inpoly_x;inpoly_y];
            u = (sensor-inpoly)/norm(sensor-inpoly);
            v = v-max(dot(u,v),0)*u;
            collision_flag = true;
        end
    end
end
% obst1 = Obstacles{1};
% obst2 = Obstacles{2};
%
% activeSensors = inpolygon(sensorX,sensorY,[obst1(1,:),nan,obst2(1,:)],[obst1(2,:),nan,obst2(2,:)]);
%
% for dir = sensorAngles(activeSensors)
%     u = [cos(dir-pi/2);sin(dir-pi/2)];
%     % v = v-max(dot(u,v),0)*u;
%     v = v-u*u.'*v;
% end

end

