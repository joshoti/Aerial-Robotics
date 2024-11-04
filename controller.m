function [ u1, u2 ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls

%   U1 Parameters
KvZ = 20; % 2
KpZ = 80; % 8
ze = des_state.pos(2) - state.pos(2);
zedot = des_state.vel(2) - state.vel(2);
zeddot = des_state.acc(2);

u1 = params.mass * (params.gravity + zeddot + (KvZ*zedot) + (KpZ*ze));

%   PhiC Parameters
KvY = 10; % 0.005 * 8
KpY = 0; % 0.0015
ye = des_state.pos(1) - state.pos(1);
yedot = des_state.vel(1) - state.vel(1);
yeddot = des_state.acc(1);

phic = -(yeddot + (KvY*yedot) + (KpY*ye)) / params.gravity ;

%   U2 Parameters
KvPhi = 20; % 2.3 * 1.3;
KpPhi = 4000; % 2;
phie = phic - state.rot(1);
phiedot = 0 - state.omega(1);
phieddot = 0;

u2 = params.Ixx * (phieddot + (KvPhi*phiedot) + (KpPhi*phie));

end

