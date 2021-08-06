function [saturated_velocity] = saturate_speed(velocity,limit)
%SATURATE_SPEED Saturates velocity vector so speed < limit
saturated_velocity = velocity;
speed = vecnorm(velocity);
indices =  speed > limit ;
if any(indices)
    saturated_velocity(:,indices) = limit*bsxfun(@rdivide,velocity(:,indices),speed(indices));
end
end