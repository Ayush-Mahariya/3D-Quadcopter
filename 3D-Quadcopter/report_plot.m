% %% plot generator for reports:
% Model.modelsiminit;
% cd +Model\
% open_system ('systema.mdl');
% cd ..
% sim('systema');
% save('sim_states','states');
% clear all
% load('sim_states');
% close all
% %
% a = 1:50:length(states.signals.values(:,1));
% plot3(states.signals.values(a,9),states.signals.values(a,11),states.signals.values(a,7),'-x','MarkerSize',12);
% axis([-4 4 -4 4 0 3])
% grid('on')
% hold on
% plot3(zeros(3,1),zeros(3,1),0:1:2,'r-o','MarkerSize',20);
% plot3(3*ones(3,1),zeros(3,1),0:1:2,'r-o','MarkerSize',20);
% plot3(zeros(3,1),3*ones(3,1),0:1:2,'r-o','MarkerSize',20);
% plot3(3*ones(3,1),3*ones(3,1),0:1:2,'r-o','MarkerSize',20);
% hold off
%%
%  pidstates = states;
%  save('pidstates','pidstates');
clear all
load('bsstates');
load('pidstates');
subplot(3,1,1)
plot(bsstates.time(1:1000),(180/pi)*bsstates.signals.values(1:1000,1),'-r','LineWidth',3);
hold on
plot(pidstates.time(1:1000),(180/pi)*pidstates.signals.values(1:1000,1),'--','LineWidth',3);
hold off
title('Roll (\phi)','FontSize',15)
set(gca,'FontSize',15)
grid('on')
xlabel('time(s)','FontSize',15)
ylabel('degrees','FontSize',15)
legend('Backstepping','PID','FontSize',12)
% 
subplot(3,1,2)
plot(bsstates.time(1:1000),(180/pi)*bsstates.signals.values(1:1000,5),'-r','LineWidth',3);
hold on
plot(pidstates.time(1:1000),(180/pi)*pidstates.signals.values(1:1000,5),'--','LineWidth',3);
hold off
title('Yaw (\psi)','FontSize',15)
grid('on')
xlabel('time(s)','FontSize',15)
ylabel('degrees','FontSize',15)
legend('Backstepping','PID','FontSize',12)
set(gca,'FontSize',15)
% 
subplot(3,1,3)
plot(bsstates.time(1:1000),bsstates.signals.values(1:1000,7),'-r','LineWidth',3);
hold on
plot(pidstates.time(1:1000),pidstates.signals.values(1:1000,7),'--','LineWidth',3);
hold off
title('Altitude (z)','FontSize',15)
grid('on')
legend('Backstepping','PID','FontSize',12)
xlabel('time(s)','FontSize',15)
ylabel('altitude(m)','FontSize',15)
set(gca,'FontSize',15)
% 
% 
% %%X
% plot(states.time(1:end),states.signals.values(1:end,11),'-r','LineWidth',2);
% title('Altitude','FontSize',15)
% grid('on')
% xlabel('time(s)','FontSize',15)
% ylabel('X position(m)','FontSize',15)
% set(gca,'Fontsize',18)