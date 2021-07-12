% Transformation to the prop rotation speeds in [rad/s] function
% INPUT: glob, image of prop speeds
% OUTPUT: prop speeds in [rad/s]

function out=bin2Om(in)

Model.glob;

bin(1)=in(1);   % binary setpoint from the PC
bin(2)=in(2);
bin(3)=in(3);
bin(4)=in(4);

% Relationship between binary setpoint and a [rad/s] out in ML motor module
% Om[rad/s]=f(x)

out(1)=slo*bin(1)-shi; % [rad/s]
out(2)=slo*bin(2)-shi;
out(3)=slo*bin(3)-shi;
out(4)=slo*bin(4)-shi;