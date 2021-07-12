function [U] = Cap(in)

U(1)=in(1);
U(2)=in(2);
U(3)=in(3);
U(4)=in(4);

if(in(1)>48)
    U(1) = 48;
end

if(in(1)<1)
    U(1) = 1;
end

if(in(2)>2)
    U(2) = 2;
end

if(in(2)<-2)
    U(2) = -2;
end

if(in(3)>2)
    U(3) = 2;
end
if(in(3)<-2)
    U(3) = -2;
end
   

if(in(4)>1)
    U(4) = 1;
end
if(in(4)<-1)
    U(4) = -1;
end
Model.glob;

% U to Omega file

global matA

Omd=sqrt(matA*U');   % desired prop speeds [rad/s]
case_var = 1;
while(isreal(Omd)==0)
    disp('Omega negative')
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
end
 
end