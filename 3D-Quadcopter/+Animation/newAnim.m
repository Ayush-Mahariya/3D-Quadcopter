function newAnim

Model.modelsiminit;
cd +Model\
open_system ('systema.mdl');
cd ..
sim('systema');
save('sim_states','states');
clear all
load('sim_states');
close all

global A

figure('units','normalized','position',[.1 .1 .8 .8],'name','Quadrotor','numbertitle','off','color','w');
axes('units','normalized','position',[.2 .1 .6 .8]);
axis equal

Animation.copter_model_3D
A.flag = 0;

axis([-5 5 -5 5 0 5])
view(30,30)
grid on
hold on
O1 = uicontrol('units','normalized','position',[.91 .86 .08 .05],'style','text','fontsize',13,'string','0','backgroundcolor','w');
camproj perspective 
camva(25)

lighting gouraud
set(gcf,'Renderer','OpenGL')

line([-1 1],[0 0],[0 0])
line([0 0],[-.5 .5],[0 0],'color','r')

%checkpoint lines
line([3 3],[0 0],[0 1],'color','g')
line([0 0],[0 0],[0 1],'color','g')
line([0 0],[3 3],[0 1],'color','g')
line([3 3],[3 3],[0 1],'color','g')
line([3 3],[0 0],[1 2],'color','b')
line([0 0],[0 0],[1 2],'color','b')
line([0 0],[3 3],[1 2],'color','b')
line([3 3],[3 3],[1 2],'color','b')
line([3 3],[0 0],[2 3],'color','r')
line([0 0],[0 0],[2 3],'color','r')
line([0 0],[3 3],[2 3],'color','r')
line([3 3],[3 3],[2 3],'color','r')
%---------- starting the loop ----------%
for i=1:length(states.time)
tic;  
   
A.phi= states.signals.values(i,1);
A.theta=states.signals.values(i,3);
A.psi=states.signals.values(i,5);
A.X=states.signals.values(i,9);
A.Y=states.signals.values(i,11);
% A.X=0;
% A.Y=0;
A.Z=states.signals.values(i,7);
%---------- PLotting the quadrotor new position ----------%
   if(A.flag==3)       
      Animation.plot_quad_3D;  
      campos([A.X+3 A.Y+4 A.Z+2])
%       campos([-3 -1 5])
      camtarget([A.X A.Y A.Z])
      camroll(0);
      set(O1,'string',num2str(states.time(i)));
      drawnow
      A.flag=0;
   toc
      while(toc<.03)
      end
   end
   A.flag = A.flag+1;
end