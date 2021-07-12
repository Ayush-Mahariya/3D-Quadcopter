% Transformation from U commands to binary image of propellers speed
% INPUT: U vector (4 elements)
% OUTPUT: decimal value to be sent to motor module

function out=U2bin2(in)

Model.glob;

U(1)=in(1)/b;
U(2)=in(2)/b;
U(3)=in(3)/b;
U(4)=in(4)/d;

Omdsqr = -1*ones(4,1);
count = 0;
while(sum(Omdsqr<0)|| sum(Omdsqr>3600))
    if(count == 1)
        U(1) = U(1)/2;
    elseif(count == 2)
        U(1) = 2*U(1);
        U(4) = U(4)/2;
    elseif(count == 3)
        U(4) = 2*U(4);
        U(3) = U(3)/2;
    elseif(count == 4)
        U(3) = U(3)*2;
        U(2) = U(2)/2;
    elseif(count >=5)
        break;
    end
    uinvert = [1/4 0 -1/2 1/4;1/4 -1/2 0 1/4;1/4 0 1/2 1/4;1/4 1/2 0 -1/4];
    Omdsqr=uinvert*U';
    count = count+1;
end
Omdsqr
Omd = abs(sqrt(Omdsqr));
Omd
bin=(Omd+shi)/slo;  % setpoint to be sent to motor modules
% Only for test
out(1)=bin(1); % [dec]
out(2)=bin(2);
out(3)=bin(3);
out(4)=bin(4);


% FOR INFORMATION MAPPING T&Q to U (Xsens MTx)
% U(1)=+T(1)+T(2)+T(3)+T(4); % [N]
% U(2)=-T(2)+T(4); % [N]
% U(3)=-T(1)+T(3); % [N]
% U(4)=+Q(1)-Q(2)+Q(3)-Q(4); % [Nm]