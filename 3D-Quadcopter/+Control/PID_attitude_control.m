% Rotations control function
% INPUT: state, (phi,theta,psi)(ref)
% OUTPUT: desired prop speeds

function out=PID_attitude_control(in)

% global file for parameters
Model.glob;
global Kp_r Ki_r Kd_r Kp_y Ki_y Kd_y 
global roll_int;
global pitch_int;
global yaw_int;

% ********* Operational Conditions *********
roll=in(1);     % [rad]
dotroll=in(2);  % [rad/s]
pitch=in(3);
dotpitch=in(4);
yaw=in(5);
dotyaw=in(6);
z=in(7);        % [m]
dotz=in(8);     % [m/s]
x=in(9);
dotx=in(10);
y=in(11);
doty=in(12);

Td=in(15);   % desired thrust [N]
rolld=in(13);   % desired ROLL angle [rad]
pitchd=in(14);   % desired PITCH angle [rad]
yawd=in(16);    % desired yaw angle [rad]
dotrolld = 0;
dotpitchd = 0;
dotyawd = 0;

% ******************************************************** CONTROL ******************************************************** 
% Little oscillating...optimize around this 
% Kp = 4;
% Ki = 0.1;
% Kd = 2;

roll_int = [(rolld-roll),roll_int];
roll_int = roll_int(1:end-1);

pitch_int = [(pitchd-pitch),pitch_int];
pitch_int = pitch_int(1:end-1);

yaw_int = [(yawd-yaw),yaw_int];
yaw_int = yaw_int(1:end-1);

U(1)=Td; % [N]
U(2)=(Ixx/L)*(Kp_r*(rolld-roll) + Ki_r*sum(roll_int)*sp+ Kd_r*(dotrolld-dotroll));
U(3)=(Iyy/L)*(Kp_r*(pitchd-pitch) + Ki_r*sum(pitch_int)*sp+ Kd_r*(dotpitchd-dotpitch));
U(4)=(Izz)* (Kp_y*(yawd-yaw) + Ki_y*sum(yaw_int)*sp+ Kd_y*(dotyawd-dotyaw));


% Outputs
out(1)=U(1);
out(2)=U(2);
out(3)=U(3);
out(4)=U(4);