function out = Trajectory2(in)
%  out(1) -> x desired
%  out(2) -> y desired
% state can vary from 1-n 
global curr_theta traj_theta
z = in(7);
x = in(9);
y = in(11);

[xd,yd] = Control.nextTarget2(curr_theta,z);
ex = xd - x;
ey = yd - y;

if(abs(ex)<0.3 && abs(ey)<0.3)
    curr_theta = traj_theta;
end

out(1) = xd;
out(2) = yd;

end
