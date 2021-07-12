function out = Trajectory(in)
%  out(1) -> x desired
%  out(2) -> y desired
% state can vary from 1-n 
global state n 

x = in(9);
y = in(11);

[xd,yd] = Control.nextTarget(state);
ex = xd - x;
ey = yd - y;

if(abs(ex)<0.3 && abs(ey)<0.3)
    state = mod((state + 1),n+1);
end

if(state == 0)
    state = 1;
end

out(1) = xd;
out(2) = yd;

end
