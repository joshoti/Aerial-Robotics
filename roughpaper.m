  X coefficients
 32 x 32 matrix containing 32 constraint equations for X values
[ 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ] [ w1_x ] % position p1 - start pos
[ 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ] [ w2_x ] % position p2 - start pos
[ 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 ] [ w3_x ] % position p3 - start pos
[ 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 ] [ w4_x ] % position p3 - start pos

[ 1 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ] [ w2_x ] % position p1 - end pos => pos_p2 - start pos
[ 0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ] [ w3_x ] % position p2 - end pos => pos_p3 - start pos
[ 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 ] [ w4_x ] % position p3 - end pos => pos_p4 - start pos
[ 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 ] [ w5_x ] % position p4 - end pos => pos_p5 - start pos

[ 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ] [ 0 ] % velocity p1 - start vel
[ 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ] [ 0 ] % acceleration p1 - start acc
[ 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ] [ 0 ] % jerk p1 - start jerk

[ 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 7 6 5 4 3 2 1 0 ] [ 0 ] % velocity p4 - end vel
[ 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 42 30 20 12 6 2 0 0 ] [ 0 ] % acceleration p4 - end acc
[ 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 210 120 60 24 6 0 0 0 ] [ 0 ] % jerk p4 - end jerk

[ 7 6 5 4 3 2 1 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ] [ 0 ] % velocity p1 - end vel => v_p2 - start vel
[ 0 0 0 0 0 0 0 0 7 6 5 4 3 2 1 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 ] [ 0 ] % velocity p2 - end vel => v_p3 - start vel
[ 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 7 6 5 4 3 2 1 0 0 0 0 0 0 0 -1 0 ] [ 0 ] % velocity p3 - end vel => v_p4 - start vel

[ 42 30 20 12 6 2 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ] [ 0 ] % acceleration p1 - end acc => a_p2 - start acc
[ 0 0 0 0 0 0 0 0 42 30 20 12 6 2 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 ] [ 0 ] % acceleration p2 - end acc => a_p3 - start acc
[ 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 42 30 20 12 6 2 0 0 0 0 0 0 0 -1 0 0 ] [ 0 ] % acceleration p3 - end acc => a_p4 - start acc

[ 210 120 60 24 6 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ] [ 0 ] % jerk p1 - end jerk => j_p2 - start jerk
[ 0 0 0 0 0 0 0 0 210 120 60 24 6 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 ] [ 0 ] % jerk p2 - end jerk => j_p3 - start jerk
[ 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 210 120 60 24 6 0 0 0 0 0 0 0 -1 0 0 0 ] [ 0 ] % jerk p3 - end jerk => j_p4 - start jerk

[ 840 360 120 24 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ] [ 0 ] % snap p1 - end snap => s_p2 - start snap
[ 0 0 0 0 0 0 0 0 840 360 120 24 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 ] [ 0 ] % snap p2 - end snap => s_p3 - start snap
[ 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 840 360 120 24 0 0 0 0 0 0 0 -1 0 0 0 0 ] [ 0 ] % snap p3 - end snap => s_p4 - start snap

[ 2520 720 120 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ] [ 0 ] % x5 p1 - end => x5_p2 - start
[ 0 0 0 0 0 0 0 0 2520 720 120 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 ] [ 0 ] % x5 p2 - end => x5_p3 - start
[ 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 2520 720 120 0 0 0 0 0 0 0 -1 0 0 0 0 0 ] [ 0 ] % x5 p3 - end => x5_p4 - start

[ 5040 720 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ] [ 0 ] % x6 p1 - end => x6_p2 - start
[ 0 0 0 0 0 0 0 0 5040 720 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ] [ 0 ] % x6 p2 - end => x6_p3 - start
[ 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 5040 720 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 ] [ 0 ] % x6 p3 - end => x6_p4 - start


% function [ T ] = polyT( n, k, t)
% % n is the polynom number of coefficients, k is the requested derivative and t is the actual value of t (this can be anything, not just 0 or 1).
% T = zeros(n,1);
% D = zeros(n,1);
% 
% % Init:
% for i=1:n
%     D(i) = i-1;
%     T(i) = 1;
% end
% 
% % Derivative:
% for j=1:k
%     for i=1:n    
%         T(i) = T(i) * D(i);
%         if D(i) > 0
%             D(i) = D(i) - 1;
%         end
%     end
% end
% 
% % put t value
% for i=1:n
%     T(i) = T(i) * t^D(i);
% end
% T = T';
% end
% 
% t0 = polyT(8,0,scale)';
% t1 = polyT(8,1,scale)';
% t2 = polyT(8,2,scale)';

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
coeffx = Ax\bx;

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
coeffy = Ay\by;

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
coeffz = Az\bz;
