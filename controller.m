function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls

% Force = 0;
% Moment = zeros(3,1);

% =================== Your code goes here ===================

% Thrust
Kd_3 = 20; 
Kp_3 = 80;
R3_error = des_state.pos(3) - state.pos(3);
R3_doterror = des_state.vel(3) - state.vel(3);
R3_ddoterror = des_state.acc(3);
F = params.mass * ( params.gravity + R3_ddoterror + (Kd_3*R3_doterror) + (Kp_3*R3_error) );

% Moment
Kd_1 = 2; 
Kp_1 = 30;
R1_error = des_state.pos(1) - state.pos(1);
R1_doterror = des_state.vel(1) - state.vel(1);
R1_ddoterror = des_state.acc(1);
R1_des_ddot = R1_ddoterror + (Kd_1*R1_doterror) + (Kp_1*R1_error);

Kd_2 = 2; 
Kp_2 = 30;
R2_error = des_state.pos(2) - state.pos(2);
R2_doterror = des_state.vel(2) - state.vel(2);
R2_ddoterror = des_state.acc(2);
R2_des_ddot = R2_ddoterror + (Kd_2*R2_doterror) + (Kp_2*R2_error);

Psi_des = des_state.yaw;
Phi_des = (R1_des_ddot*sin(Psi_des) - R2_des_ddot*cos(Psi_des) ) / params.gravity ;
Theta_des = (R1_des_ddot*cos(Psi_des) + R2_des_ddot*sin(Psi_des) ) / params.gravity ;

r_des = des_state.yawdot;
p_des = 0;
q_des = 0;

Kp_Psi = 4000;
Kd_Psi = 20;
Kp_Phi = 6000;
Kd_Phi = 20;
Kp_Theta = 6000;
Kd_Theta = 20;
Psi_error = Psi_des - state.rot(3);
Psi_doterror = r_des - state.omega(3);
Phi_error = Phi_des - state.rot(1);
Phi_doterror = p_des - state.omega(1);
Theta_error = Theta_des - state.rot(2);
Theta_doterror = q_des - state.omega(2);

Phi_M = (Kd_Phi*Phi_doterror) + (Kp_Phi*Phi_error);
Theta_M = (Kd_Theta*Theta_doterror) + (Kp_Theta*Theta_error);
Psi_M = (Kd_Psi*Psi_doterror) + (Kp_Psi*Psi_error);
M = params.I * [Phi_M ; Theta_M ; Psi_M];

% =================== Your code ends here ===================

end
