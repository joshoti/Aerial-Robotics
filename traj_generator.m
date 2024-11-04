function [ desired_state ] = traj_generator(t, state, waypoints)
Ax = [[ 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ];
     [ 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ];
     [ 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 ];
     [ 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 ];
     [ 1 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ];
     [ 0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ];
     [ 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 ];
     [ 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 ];
     [ 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ];
     [ 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ];
     [ 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ];
     [ 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 7 6 5 4 3 2 1 0 ];
     [ 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 42 30 20 12 6 2 0 0 ];
     [ 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 210 120 60 24 6 0 0 0 ];
     [ 7 6 5 4 3 2 1 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ];
     [ 0 0 0 0 0 0 0 0 7 6 5 4 3 2 1 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 ];
     [ 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 7 6 5 4 3 2 1 0 0 0 0 0 0 0 -1 0 ];
     [ 42 30 20 12 6 2 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ];
     [ 0 0 0 0 0 0 0 0 42 30 20 12 6 2 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 ];
     [ 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 42 30 20 12 6 2 0 0 0 0 0 0 0 -1 0 0 ];
     [ 210 120 60 24 6 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ];
     [ 0 0 0 0 0 0 0 0 210 120 60 24 6 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 ];
     [ 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 210 120 60 24 6 0 0 0 0 0 0 0 -1 0 0 0 ];
     [ 840 360 120 24 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ];
     [ 0 0 0 0 0 0 0 0 840 360 120 24 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 ];
     [ 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 840 360 120 24 0 0 0 0 0 0 0 -1 0 0 0 0 ];
     [ 2520 720 120 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ];
     [ 0 0 0 0 0 0 0 0 2520 720 120 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 ];
     [ 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 2520 720 120 0 0 0 0 0 0 0 -1 0 0 0 0 0 ];
     [ 5040 720 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ];
     [ 0 0 0 0 0 0 0 0 5040 720 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ];
     [ 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 5040 720 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 ]
     ];
bx = [0;1;2;3;1;2;3;4;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0];
Ay = [[ 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ];
     [ 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ];
     [ 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 ];
     [ 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 ];
     [ 1 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ];
     [ 0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ];
     [ 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 ];
     [ 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 ];
     [ 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ];
     [ 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ];
     [ 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ];
     [ 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 7 6 5 4 3 2 1 0 ];
     [ 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 42 30 20 12 6 2 0 0 ];
     [ 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 210 120 60 24 6 0 0 0 ];
     [ 7 6 5 4 3 2 1 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ];
     [ 0 0 0 0 0 0 0 0 7 6 5 4 3 2 1 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 ];
     [ 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 7 6 5 4 3 2 1 0 0 0 0 0 0 0 -1 0 ];
     [ 42 30 20 12 6 2 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ];
     [ 0 0 0 0 0 0 0 0 42 30 20 12 6 2 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 ];
     [ 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 42 30 20 12 6 2 0 0 0 0 0 0 0 -1 0 0 ];
     [ 210 120 60 24 6 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ];
     [ 0 0 0 0 0 0 0 0 210 120 60 24 6 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 ];
     [ 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 210 120 60 24 6 0 0 0 0 0 0 0 -1 0 0 0 ];
     [ 840 360 120 24 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ];
     [ 0 0 0 0 0 0 0 0 840 360 120 24 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 ];
     [ 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 840 360 120 24 0 0 0 0 0 0 0 -1 0 0 0 0 ];
     [ 2520 720 120 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ];
     [ 0 0 0 0 0 0 0 0 2520 720 120 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 ];
     [ 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 2520 720 120 0 0 0 0 0 0 0 -1 0 0 0 0 0 ];
     [ 5040 720 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ];
     [ 0 0 0 0 0 0 0 0 5040 720 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ];
     [ 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 5040 720 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 ]
     ];
by = [0;1;0;-1;1;0;-1;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0];
Az = [[ 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ];
     [ 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ];
     [ 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 ];
     [ 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 ];
     [ 1 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ];
     [ 0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ];
     [ 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 ];
     [ 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 ];
     [ 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ];
     [ 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ];
     [ 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ];
     [ 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 7 6 5 4 3 2 1 0 ];
     [ 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 42 30 20 12 6 2 0 0 ];
     [ 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 210 120 60 24 6 0 0 0 ];
     [ 7 6 5 4 3 2 1 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ];
     [ 0 0 0 0 0 0 0 0 7 6 5 4 3 2 1 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 ];
     [ 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 7 6 5 4 3 2 1 0 0 0 0 0 0 0 -1 0 ];
     [ 42 30 20 12 6 2 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ];
     [ 0 0 0 0 0 0 0 0 42 30 20 12 6 2 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 ];
     [ 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 42 30 20 12 6 2 0 0 0 0 0 0 0 -1 0 0 ];
     [ 210 120 60 24 6 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ];
     [ 0 0 0 0 0 0 0 0 210 120 60 24 6 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 ];
     [ 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 210 120 60 24 6 0 0 0 0 0 0 0 -1 0 0 0 ];
     [ 840 360 120 24 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ];
     [ 0 0 0 0 0 0 0 0 840 360 120 24 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 ];
     [ 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 840 360 120 24 0 0 0 0 0 0 0 -1 0 0 0 0 ];
     [ 2520 720 120 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ];
     [ 0 0 0 0 0 0 0 0 2520 720 120 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 ];
     [ 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 2520 720 120 0 0 0 0 0 0 0 -1 0 0 0 0 0 ];
     [ 5040 720 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ];
     [ 0 0 0 0 0 0 0 0 5040 720 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ];
     [ 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 5040 720 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 ]
     ];
bz = [0;1;2;1;1;2;1;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0];
% TRAJ_GENERATOR: Generate the trajectory passing through all
% positions listed in the waypoints list
%
% NOTE: This function would be called with variable number of input arguments.
% During initialization, it will be called with arguments
% trajectory_generator([], [], waypoints) and later, while testing, it will be
% called with only t and state as arguments, so your code should be able to
% handle that. This can be done by checking the number of arguments to the
% function using the "nargin" variable, check the MATLAB documentation for more
% information.
%
% t,state: time and current state (same variable as "state" in controller)
% that you may use for computing desired_state
%
% waypoints: The 3xP matrix listing all the points you much visited in order
% along the generated trajectory
%
% desired_state: Contains all the information that is passed to the
% controller for generating inputs for the quadrotor
%
% It is suggested to use "persistent" variables to store the waypoints during
% the initialization call of trajectory_generator.


%% Example code:
% Note that this is an example of naive trajectory generator that simply moves
% the quadrotor along a stright line between each pair of consecutive waypoints
% using a constant velocity of 0.5 m/s. Note that this is only a sample, and you
% should write your own trajectory generator for the submission.
    function [ T ] = polyT( n, k, t)
        % n is the polynom number of coefficients, k is the requested derivative and t is the actual value of t (this can be anything, not just 0 or 1).
        T = zeros(n,1);
        D = zeros(n,1);

        % Init:
        for i=1:n
            D(i) = i-1;
            T(i) = 1;
        end

        % Derivative:
        for j=1:k
            for i=1:n    
                T(i) = T(i) * D(i);
                if D(i) > 0
                    D(i) = D(i) - 1;
                end
            end
        end

        % put t value
        for i=1:n
            T(i) = T(i) * t^D(i);
        end
        T = T';
    end

persistent waypoints0 traj_time d0 coeffx coeffy coeffz
if nargin > 2
    d = waypoints(:,2:end) - waypoints(:,1:end-1);
    d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
    traj_time = [0, cumsum(d0)];
    waypoints0 = waypoints;
    coeffx = Ax\bx;    
    coeffy = Ay\by;    
    coeffz = Az\bz;
else
    if(t > traj_time(end))
        t = traj_time(end);
    end
    t_index = find(traj_time >= t,1) - 1;
    t_index = max(t_index,1);
    scale = (t-traj_time(t_index)) / d0(t_index);
    if(t == 0)
        desired_state.pos = waypoints0(:,1);
        desired_state.vel = zeros(3,1);
        desired_state.acc = zeros(3,1);
    else
%         scale = t/d0(t_index-1);        
%         desired_state.pos = (1 - scale) * waypoints0(:,t_index-1) + scale * waypoints0(:,t_index);       
        
        index = [(t_index-1)*8+1:t_index*8];
        
        t0 = polyT(8,0,scale)';
        t1 = polyT(8,1,scale)';        
        t2 = polyT(8,2,scale)';               
        
%         desired_state.pos = [coeffx(index)'*t0;coeffy(index)'*t0;coeffz(index)'*t0 ];
%         desired_state.vel=[coeffx(index)'*t1.*(1/d0(t_index));coeffy(index)'*t1.*(1/d0(t_index));coeffz(index)'*t1.*(1/d0(t_index)) ];
%         desired_state.acc=[coeffx(index)'*t2.*(1/d0(t_index)^2);coeffy(index)'*t2.*(1/d0(t_index)^2);coeffz(index)'*t2.*(1/d0(t_index)^2) ];
    end        
    
%% Method 2: spline
    k = [0 0; 0 0; 0 0];
    pp = spline(traj_time,[k(:,1) waypoints0 k(:,2)]);
%     disp('waypoints0') ; disp(waypoints0) ; disp('')
%     disp('traj_time') ; disp(traj_time) ; disp('')
%     disp('k(:,1)') ; disp(k(:,1)) ; disp('')
%     disp('pp') ; disp(pp) ; disp('')
    desired_state.pos = ppval(pp,t);
    desired_state.vel=(ppval(pp,t+.001)-ppval(pp,t))/(.001);
    desired_state.acc = zeros(3,1);
    desired_state.yaw = 0;
    desired_state.yawdot = 0;
end

%% Fill in your code here

% desired_state.pos = zeros(3,1);
% desired_state.vel = zeros(3,1);
% desired_state.acc = zeros(3,1);
% desired_state.yaw = 0;

end

