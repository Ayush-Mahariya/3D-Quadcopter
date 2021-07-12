% priority wise speed reduction

function out=U2bin(in)

Model.glob;

% U to Omega file
U(1)=in(1);
U(2)=in(2);
U(3)=in(3);
U(4)=in(4);
global matA

Omd=sqrt(matA*U');   % desired prop speeds [rad/s]

case_var = 1;

while(isreal(Omd)==0)
    disp('omega going negative!!');
    
    if(case_var == 1)
        U(4) = U(4)/2;
        case_var = case_var+1;
    elseif(case_var == 2)
        U(3) = U(3)/2;
        case_var = case_var+1;
    elseif(case_var == 3)
        U(2) = U(2)/2;
        case_var = case_var+1;
    else
        U(1) = 2*U(1);
        case_var = 1;
    end
    
    Omd=sqrt(matA*U');
end
    
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