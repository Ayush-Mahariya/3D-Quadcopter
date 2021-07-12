% Altitude control function
% INPUT: state, desired altitude
% OUTPUT: desired speeds

function out=alt_control(in)

% global file for parameters
Model.glob;
roll=in(1);     % [rad]
pitch=in(2);
z=in(3);        % [m]
zd=in(4);  % desired altitude [m]
dotz = in(5);
dotzd=0;

dotz=diff(z);
dotz(1)=0;

% ********** CONTROL 1 PD ************
%%YOUR CODE HERE TO CALCULATE THE OUTPUT SPEED OF MOTORS
a1=2;
a2=4;

r1=-a1*dotz-a2*(z-zd);

U1=(r1+m*g)/(cos(roll)*cos(pitch));

% Required Thrust
out(1)=U1; %[N]
