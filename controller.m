function [ u ] = pd_controller(~, s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters

Kp = 20;
%5.706 - 5.8
%Kv = 6;
if s(1) > 0.8 * s_des(1)
    Kv = 8;
else
    Kv = 2;
end
e = s_des(1) - s(1);
edot = (s_des(2)) - (s(2));

u = params.mass * (params.gravity + (Kp*e) + (Kv*edot) + s_des(2));


% FILL IN YOUR CODE HERE


end

