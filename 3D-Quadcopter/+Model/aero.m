% Aerodynamics function
% INPUT: glob, state, prop speeds
% OUTPUT: Aerodynamic forces and torques

function out=aero(in)
Model.glob;

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

% ********* Actual rotor speeds *********
Omega=in(13:16);    % [rad/s]
P_active = in(17);


%%YOUR CODE HERE
% horizontal speed [m/s]
V=sqrt(dotx^2+doty^2);

% Inflow velocity [m/s]
v1=sqrt(-0.5*V^2+sqrt((0.5*V^2)^2+(w/(2*rho*A))^2));

% Inflow ratio
lambda=(v1+dotz)/(OmegaH*R);

% advance ratio
mu=V/(OmegaH*R);

% advance ratio in x axis
muX=dotx/(OmegaH*R);

% advance ratio in y axis
muY=doty/(OmegaH*R);

% Lift coefficients
%a) for Thrust force
Ct=sigma_*a*(((1/6)+(mu^2)/4)*theta0-((1+mu^2)*thetatw/8)-lambda/4);
%b) for Torque
Cq=sigma_*a*((1/(8*a))*(1+mu^2)*Cd+lambda*((theta0/6)-(thetatw/8)-(lambda/4)));
%c) for Hub force in X
ChX=sigma_*a*((muX*Cd/(4*a))+(0.25*lambda*muX*(theta0-0.5*thetatw)));
%d) for Hub force in Y
ChY=sigma_*a*((muY*Cd/(4*a))+(0.25*lambda*muY*(theta0-0.5*thetatw)));
%e) for Rolling moments in X
CrrX= - sigma_*a*(muX*(theta0/6-thetatw/8-lambda/8)); % ------> NEGATIVE
%f) for Rolling moments in Y
CrrY= - sigma_*a*(muY*(theta0/6-thetatw/8-lambda/8)); % ------> NEGATIVE

T = zeros(P_active,1);
Q = zeros(P_active,1);
HX = zeros(P_active,1);
HY = zeros(P_active,1);
RRX = zeros(P_active,1);
RRY = zeros(P_active,1);

thrustcoeff=Ct*rho*A*(R^2);
disp(thrustcoeff);
for i = 1:P_active
% *** Thrust force of individual motor ***
T(i)=thrustcoeff*((Omega(i))^2); %[N]

% *** Torque due to each motor ***
Q(i)=Cq*rho*A*(Omega(i)^2)*(R^3); %[Nm]

% *** Hub forces due to each motor ***
% Hub force in X
HX(i)=ChX*rho*A*((Omega(i)*R)^2); %[N]
% Hub force in Y
HY(i)=ChY*rho*A*((Omega(i)*R)^2); %[N]

% *** Rolling moment due to each motor ***
% The rolling moments in X
RRX(i)=CrrX*rho*A*(Omega(i)^2)*(R^3); %[Nm]
% The rolling moments in Y
RRY(i)=CrrY*rho*A*(Omega(i)^2)*(R^3); %[Nm]

end

%should calculate the outputs listed below.

out(1:4)=T;
out(5:8)=Q;
out(9:12)=HX;
out(13:16)=HY;
out(17:20)=RRX;
out(21:24)=RRY;