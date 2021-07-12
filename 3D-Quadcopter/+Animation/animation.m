function animation

optsiminit;
open_system ('systema.mdl');
sim('systema.mdl');
save('sim_states','states');
clear all
load('sim_states');
close all

global A

figure('units','normalized','position',[.1 .1 .8 .8],'name','Quadrotor AUS','numbertitle','off','color','w');
axes('units','normalized','position',[.2 .1 .6 .8]);
axis equal

All_Variables;
quadmodel;

E1 = uicontrol('units','normalized','position',[.11 .85 .1 .07],'style','edit','fontsize',13,'string',0,'backgroundcolor','w');
E4 = uicontrol('units','normalized','position',[.11 .55 .1 .07],'style','edit','fontsize',13,'string',0,'backgroundcolor','w');
E5 = uicontrol('units','normalized','position',[.11 .45 .1 .07],'style','edit','fontsize',13,'string',0,'backgroundcolor','w');
E6 = uicontrol('units','normalized','position',[.11 .35 .1 .07],'style','edit','fontsize',13,'string',0,'backgroundcolor','w');

uicontrol('units','normalized','position',[.02 .83 .05 .07],'style','text','fontsize',13,'string','Altitude','backgroundcolor','w');
uicontrol('units','normalized','position',[.02 .53 .05 .07],'style','text','fontsize',13,'string','Yaw','backgroundcolor','w');
uicontrol('units','normalized','position',[.02 .43 .05 .07],'style','text','fontsize',13,'string','X','backgroundcolor','w');
uicontrol('units','normalized','position',[.02 .33 .05 .07],'style','text','fontsize',13,'string','Y','backgroundcolor','w');
uicontrol('units','normalized','position',[.11 .25 .1 .07],'style','pushbutton','fontsize',13,'string','Go','callback',@Go1);

uicontrol('units','normalized','position',[.85 .83 .05 .07],'style','text','fontsize',13,'string','Front M','backgroundcolor',[.5 .7 1]);
uicontrol('units','normalized','position',[.85 .73 .05 .07],'style','text','fontsize',13,'string','Right M','backgroundcolor',[.5 .7 1]);
uicontrol('units','normalized','position',[.85 .63 .05 .07],'style','text','fontsize',13,'string','Rear M','backgroundcolor',[.5 .7 1]);
uicontrol('units','normalized','position',[.85 .53 .05 .07],'style','text','fontsize',13,'string','Left M','backgroundcolor',[.5 .7 1]);

O1 = uicontrol('units','normalized','position',[.91 .86 .08 .05],'style','text','fontsize',13,'string','0','backgroundcolor','w');
O2 = uicontrol('units','normalized','position',[.91 .76 .08 .05],'style','text','fontsize',13,'string','0','backgroundcolor','w');
O3 = uicontrol('units','normalized','position',[.91 .66 .08 .05],'style','text','fontsize',13,'string','0','backgroundcolor','w');
O4 = uicontrol('units','normalized','position',[.91 .56 .08 .05],'style','text','fontsize',13,'string','0','backgroundcolor','w');

pop1 = uicontrol('units','normalized','position',[.02 .15 .19 .07],'style','popupmenu','fontsize',13,'string',{'3D view';'Camera view'},'callback',@view1,'value',1);

axis([-5 5 -5 5 0 2])
view(30,30)
grid on
hold on

camproj perspective 
camva(25)

hlight = camlight('headlight'); 

lighting gouraud
set(gcf,'Renderer','OpenGL')

line([-1 1],[0 0],[0 0])
line([0 0],[-.5 .5],[0 0],'color','r')

%---------- starting the loop ----------%
while 1
tic;  
%---------- measuring the parameters ----------%
   Z_meas;
   XY_meas;
   IMU_meas;
   
%---------- Kalman filters ----------%
    Kalman_phi2;
    Kalman_theta2;
    Kalman_psi2;
    Kalman_Z2;
    Kalman_X2;
    Kalman_Y2;

%---------- perform PID controllers ----------%
   PID_X;
   PID_Y;
   PID_Z;
   PID_roll;
   PID_pitch;
   PID_heading;

%----------- limit the motors speed ----------%   
   Motors_Speed;
   Forces;
   
%---------- apply the equations of motion -----------%
   quadmodel;
   
%---------- Calculating Motors speed and displaying it ------------%

   set(O1,'string',num2str(A.O1));
   set(O2,'string',num2str(A.O2));
   set(O3,'string',num2str(A.O3));
   set(O4,'string',num2str(A.O4));
   
%---------- PLotting the quadrotor new position ----------%
   if(A.flag==3)
       
      plot_quad_3D;  
      switch(A.Camera_View)
          case 0
      campos([A.X+2 A.Y+2 A.Z+2])
      camtarget([A.X A.Y A.Z])
      camroll(0);
          case 1
      campos([A.X A.Y A.Z])
      camtarget([A.X+A.Z*sin(A.theta) A.Y+A.Z*sin(A.phi) 0])
      camroll(A.psi*180/pi-A.psi_cam)
      A.psi_cam = A.psi*180/pi;
      end
    

      drawnow
      A.flag=0;
   toc
      while(toc<.03)
      end
   end
   A.flag = A.flag+1;
   
   A.init = 1;  % stop the initilization in my code
end


%---------- Subfunctions -----------%

    function Go1(varargin)
        A.Z_des = str2double(get(E1,'string'));
        A.psi_des = str2double(get(E4,'string'))*pi/180;
        A.X_des_EF = str2double(get(E5,'string'));
        A.Y_des_EF = str2double(get(E6,'string'));
    end

    function view1(varargin)
       A.var1 = get(pop1,'value');
       switch A.var1
           case 1
               A.Camera_View = 0;
           case 2
               A.Camera_View = 1;
       end
    end

end