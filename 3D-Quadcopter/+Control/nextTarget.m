function [xd,yd] = nextTarget(state)

if(state == 1)
    xd = 3;
    yd = 0;
end

if(state == 2)
    xd = 3;
    yd = 3;
end

if(state == 3)
    xd = 0;
    yd = 3;
end

if(state == 4)
    xd = 0;
    yd = 0;
end

end