% Dynamics fnction
% INPUT: state, aerodynamic forces & torques, prop rot speed
% OUTPUT: system accelerations

function out=dinamica(in)

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

% from aero file   
T=in(13:16);    % The thrusts [N]
Q=in(17:20);    % The counter-torques [Nm]
HX=in(21:24);    % The hub forces in X [N]
HY=in(25:28);   % The hub forces in Y [N]
RRX=in(29:32);   % The rolling moments in X [N]
RRY=in(33:36);   % The rolling moments in Y [N]

% from motor dyna file
Omega=in(37:40);    % [rad/s]
Om=+Omega(1)-Omega(2)+Omega(3)-Omega(4); % Om residual propellers rot. speed [rad/s]

% *************** Rotations (in body fixed frame) *************** 
%YOUR CODE HERE

global Om_old;

%calculate
%1)Roll moments
%a) Body gyro effect
RgB = dotpitch*dotyaw*(Iyy-Izz); %[Nm]

%b) Propeller gyro effect
RgP = jr*dotpitch*Om; %[Nm]

%c) Roll actuator action
RaA = L*(-T(2)+T(4)); %[Nm]

%d) Hub force in y axis causes positive roll
RhF = (HY(1)+HY(2)+HY(3)+HY(4))*h; %[Nm]

%e) Rolling moment due to forward flight in X
RrM = +RRX(1)-RRX(2)+RRX(3)-RRX(4); %[Nm]

%f) Roll friction moment VOIR L'IMPORTANCE
RfM = 0.5*Cz*A*rho*dotroll*abs(dotroll)*L*(P/2)*L; %[Nm]

%2)Pitch moments
%a) Body gyro effect
PgB = dotroll*dotyaw*(Izz-Ixx); %[Nm]

%b) Propeller gyro effect
PgP = jr*dotroll*Om; %[Nm]

%c) Pitch actuator action
PaA = L*(-T(1)+T(3)); %[Nm]

%d) Hub force in y axis causes positive pitch
PhF = (HX(1)+HX(2)+HX(3)+HX(4))*h; %[Nm]

%e) Pitching moment due to sideward flight
PrM = +RRY(1)-RRY(2)+RRY(3)-RRY(4); %[Nm]

%f) Pitch friction moment VOIR L'IMPORTANCE
PfM = 0.5*Cz*A*rho*dotpitch*abs(dotpitch)*L*(P/2)*L; %[Nm]

%3)Yaw moments
%a) Body gyro effect
YgB = dotpitch*dotroll*(Ixx-Iyy); %[Nm]

%b) Inertial acceleration produces opposit yawing moment
YiA = jr*(Om-abs(Om_old))/sp; %[Nm]

%c) counter torques difference produces yawing
YawA = +Q(1)-Q(2)+Q(3)-Q(4); %[Nm]

%d) Yaw moment produced due to Hub force in X
YhFx = (-HX(2)+HX(4))*L; %[Nm]

%e) Yaw moment produced due to Hub force in Y
YhFy = (-HY(1)+HY(3))*L; %[Nm]

Om_old=Om; % [rad/s]


% *************** Translations (in earth fixed frame) *************** 
%YOUR CODE HERE
%calculate
%1)X forces
%a) Actuators action in X
XaA = (sin(yaw)*sin(roll)+cos(yaw)*sin(pitch)*cos(roll))*(T(1)+T(2)+T(3)+T(4)); %[N]

%b) Drag force acting in X
XdF = 0.5*Cx*Ac*rho*dotx*abs(dotx); %[N]

%c) Net Hub force in X
XhF = HX(1)+HX(2)+HX(3)+HX(4); %[N]

%2)Y forces
%a) Actuators action in Y
YaA = (-cos(yaw)*sin(roll)+sin(yaw)*sin(pitch)*cos(roll))*(T(1)+T(2)+T(3)+T(4)); %[N]

%b) Drag force acting in Y
YdF = 0.5*Cy*Ac*rho*doty*abs(doty); %[N]

%c) Net Hub force in Y
YhF = HY(1)+HY(2)+HY(3)+HY(4); %[N]

%3)Z forces
%a) Actuators action in Z
ZaA = (cos(pitch)*cos(roll))*(T(1)+T(2)+T(3)+T(4)); %[N]

%b) Archimedian force
ZaR = PArchim; %[N]

%c) Net friction force in Z
ZaF = 0.5*Cz*A*rho*dotz*abs(dotz)*P + 0.5*Cz*Ac*rho*dotz*abs(dotz); %[N]

% *************** OS4 equations of motion *************** 
%YOUR CODE HERE
%Use equations of motion to find rotations and translations and set them as
%output
%ROTATIONS
%1)roll rate [rad/s] 
out(1)=dotroll;

%2) roll accel [rad/s^2]
out(2)=(RgB + RgP + RaA + RhF + RrM - RfM) /Ixx;

%3) pitch rate [rad/s]
out(3)=dotpitch;

%4) pitch accel [rad/s^2]
out(4)=(PgB - PgP + PaA - PhF + PrM - PfM) /Iyy;

%5) yaw rate [rad/s]
out(5)=dotyaw;

%6) yaw accel [rad/s^2]
out(6)=(YgB + YiA + YawA + YhFx + YhFy) /Izz;


% TRANSLATIONS Z,X,Y
%1) z rate [m/s]
out(7)=dotz;

%2) z accel [m/s^2]
out(8)=-g + (ZaR + ZaA - ZaF)/m;

%3) x rate [m/s]
out(9)=dotx;

%4) x accel [m/s^2]
out(10)=(XaA - XdF - XhF)/m;

%5) y rate [m/s]
out(11)=doty;

%6) y accel [m/s^2]
out(12)=(YaA - YdF - YhF)/m;

% *** Other outputs ***
out(13)=ZaA;
out(14)=ZaR;
out(15)=ZaF;
out(16)=YhFx;
out(17)=YhFy;
out(18)=YhFy;