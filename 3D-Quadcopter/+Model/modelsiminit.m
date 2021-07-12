% SIMINIT Initialization of global variables
clear all;
clc;

Model.glob;

global traj_theta curr_theta
traj_theta = 0;
curr_theta=0;

global roll_samples dotroll_samples pitch_samples dotpitch_samples yaw_samples dotyaw_samples z_samples filter_dim;
filter_dim = 5;
roll_samples = zeros(1,filter_dim);
dotroll_samples = zeros(1,filter_dim);
pitch_samples = zeros(1,filter_dim);
dotpitch_samples = zeros(1,filter_dim);
yaw_samples = zeros(1,filter_dim);
dotyaw_samples = zeros(1,filter_dim);
z_samples = zeros(1,filter_dim);


global residue Timeframe;
Timeframe = 15;
residue = zeros(6,Timeframe/sp);

global Kp_alt Ki_alt Kd_alt Kp_r Ki_r Kd_r Kp_y Ki_y Kd_y
Kp_alt = 26.6;
Ki_alt = 11.1;
Kd_alt = 5.3;
Kp_r = 20.5;
Ki_r = 6;
Kd_r = 7;
Kp_y = 20.6;
Ki_y = 8.9;
Kd_y = 9.4;
% %%[26.6,11.1,5.3,20.5,6,7,20.6,8.9,9.4]


global alpha1 alpha2 alpha3 alpha4 alpha5 alpha6 alpha7 alpha8
alpha1 = 7.3;
alpha2 = 5.1;
alpha3 = 10.2;
alpha4 = 4.3;
alpha5 = 4.2;
alpha6 = 4.2;
alpha7 = 7.8;
alpha8 = 7.8;

global x_int y_int dotx_int doty_int
x_int = zeros(1,10);
y_int = zeros(1,10);
dotx_int = zeros(1,10);
doty_int = zeros(1,10);

global state n
state = 1;
n = 4;

global u1_prev u2_prev u3_prev u4_prev;
u1_prev = 0;
u2_prev = 0;
u3_prev = 0;
u4_prev = 0;

global alt_int;
alt_int = zeros(1,10);

global roll_int pitch_int yaw_int;
roll_int = zeros(1,10);
pitch_int = zeros(1,10);
yaw_int = zeros(1,10);

global x1_1;    % old integrator value
global x2_1;    % old integrator value
global x3_1;    % old integrator value

global Om_old; % old residual speed

x1_1=0;
x2_1=0;
x3_1=0;
Om_old=0;

global matA
Model.glob;
matB = [b b b b;0 -b 0 b; -b 0 b 0;d -d d -d];
matA = matB^-1;

disp('Done initializing')