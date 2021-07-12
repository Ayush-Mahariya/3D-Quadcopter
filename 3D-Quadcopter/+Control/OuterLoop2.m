function [ out ] = OuterLoop2( in )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
Model.glob;

global x_int y_int dotx_int doty_int

Kp = 4;
Ki = 0.2;

Kp_dot = 0.3;
Ki_dot = 0.1;
Kd_dot = 1;

xd = in(1);
yd = in(2);
current_yaw = in(7);

%rotating coordinate system wrt yaw
x_rot = xd * cos(current_yaw) + yd * sin(current_yaw);
y_rot = -xd * sin(current_yaw) + yd * cos(current_yaw);

x_current = in(11) * cos(current_yaw) + in(13) * sin(current_yaw);
y_current = -in(11) * sin(current_yaw) + in(13) * cos(current_yaw);

%error
error_x = x_rot - x_current;
error_y = y_rot - y_current;

x_int = [error_x,x_int];
x_int = x_int(1:end-1);

y_int = [error_y,y_int];
y_int = y_int(1:end-1);

%velocity control
out(1) = deg2rad(Kp_dot * error_x + Ki_dot*sp*sum(x_int) + Kd_dot*(x_int(1)-x_int(2)));
out(2) = deg2rad(Kp_dot * error_y + Ki_dot*sp*sum(y_int) + Kd_dot*(y_int(1)-y_int(2)));

% dotx_r = in(12) * cos(current_yaw) + in(14) * sin(current_yaw);
% doty_r = -in(12) * sin(current_yaw) + in(14) * cos(current_yaw);
% 
% error_x_dot = dotx_d - dotx_r;
% error_y_dot = doty_d - doty_r;
% 
% % error_x_dot = dotx_d - in(12);
% % error_y_dot = doty_d - in(14);
% 
% dotx_int = [error_x_dot,dotx_int];
% dotx_int = dotx_int(1:end-1);
% 
% doty_int = [error_y_dot,doty_int];
% doty_int = doty_int(1:end-1);
% 
% out(1) = -deg2rad(Kp*error_y_dot + Ki*sp*sum(dotx_int));
% out(2) =  deg2rad(Kp*error_x_dot + Ki*sp*sum(doty_int));

end
