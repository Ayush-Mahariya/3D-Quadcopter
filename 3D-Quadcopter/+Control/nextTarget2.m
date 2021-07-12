function [xd,yd] = nextTarget2(curr_theta,z)
global traj_theta

if(curr_theta == traj_theta)
    traj_theta = traj_theta + 0.6;
end

xd = (z)*sin(traj_theta);
yd = (z)*cos(traj_theta);

end