clear; close all; clc

V_1 = importdata('V_05_0.mat');
V_2 = importdata('V_05_05.mat');
V_3 = importdata('V_05_1.mat');
g = importdata('g_05_0.mat');
load('traj.mat');
load('traj_V.mat');

dt = 0.1;
t = [0:dt:20];

% L = find(V_1 > 3);
% V_1(L) = NaN;
% Deriv_1 = computeGradients(g, V_1);
% L_1 = find(abs(Deriv_1{1}) >3);
% L_2 = find(abs(Deriv_1{2}) >3);
% V_1(L_1) = NaN;
% V_1(L_2) = NaN;
% 
% L = find(V_2 > 3);
% V_2(L) = NaN;
% Deriv_2 = computeGradients(g, V_2);
% L_1 = find(abs(Deriv_2{1}) >3);
% L_2 = find(abs(Deriv_2{2}) >3);
% V_2(L_1) = NaN;
% V_1(L_2) = NaN;
% 
% L = find(V_3 > 3);
% V_3(L) = NaN;
% Deriv_3 = computeGradients(g, V_3);
% L_1 = find(abs(Deriv_3{1}) >3);
% L_2 = find(abs(Deriv_3{2}) >3);
% V_3(L_1) = NaN;
% V_3(L_2) = NaN;

%%
% set(0,'DefaultFigureWindowStyle','docked')
close all
% figure
% set(gcf,'unit','normalized','position',[0.2,0.5,0.64,0.3]);
extra1.LineStyle = '--';
figure
set(gcf,'unit','normalized','position',[0.2,0.2,0.64,0.6]);
subplot(2,3,1)
visFuncIm(g, V_1,'black',.3);
hold on
visSetIm(g, V_1,'black',6,extra1);
zlim([0,6])
c = camlight;
c.Position = [-30 -30 -30];
grid off
view(30,15)
set(gca,'unit','normalized','position',[0.1,0.65,0.2,0.25])
set(gca,'zTick',[0:1.5:6]);
a = get(gca,'ZTickLabel');
set(gca,'ZTickLabel',a,'fontsize',15)
xlabel('$x_1$','interpreter','latex','FontSize',25);
ylabel('$x_2$','interpreter','latex','FontSize',25);
zlabel('$V_\gamma^\infty$','interpreter','latex','FontSize',25);
title('$\gamma = 0$','interpreter','latex','FontSize',25)

subplot(2,3,2)
visFuncIm(g, V_2,'blue',.3);
hold on
visSetIm(g, V_2,'blue',6,extra1);
c = camlight;
c.Position = [-30 -30 -30];
zlim([0,6])
grid off
view(30,15)
set(gca,'zTick',[0:1.5:6]);
set(gca,'unit','normalized','position',[0.4,0.65,0.2,0.25])
a = get(gca,'ZTickLabel');
set(gca,'ZTickLabel',a,'fontsize',15)
xlabel('$x_1$','interpreter','latex','FontSize',25);
ylabel('$x_2$','interpreter','latex','FontSize',25);
% zlabel('$V_\gamma^\infty(\gamma = 0.1)$','interpreter','latex','FontSize',25);
title('$\gamma = 0.5$','interpreter','latex','FontSize',25)

subplot(2,3,3)
visFuncIm(g, V_3,'red',.3);
c = camlight;
c.Position = [-30 -30 -30];
zlim([0,6])
hold on
visSetIm(g, V_3,'red',6,extra1);
grid off
view(30,15)
set(gca,'zTick',[0:1.5:6]);
a = get(gca,'ZTickLabel');
set(gca,'unit','normalized','position',[0.7,0.65,0.2,0.25])
set(gca,'ZTickLabel',a,'fontsize',15)
xlabel('$x_1$','interpreter','latex','FontSize',25);
ylabel('$x_2$','interpreter','latex','FontSize',25);
title('$\gamma = 1$','interpreter','latex','FontSize',20)


subplot(2,3,4)
plot(x(1,:,1),x(2,:,1),'black','linewidth',2);
hold on
plot(x(1,:,2),x(2,:,2),'blue','linewidth',2);
hold on
plot(x(1,:,3),x(2,:,3),'red','linewidth',2);
hold on
plot(x(1,1,1),x(2,1,1),'mo','linewidth',2);
hold on
plot(0,0,'go','linewidth',2);
extra.LineWidth = 1;
extra.LineStyle = '--';
visSetIm(g, V_1,'black',6,extra);
hold on
visSetIm(g, V_2,'blue',6,extra);
hold on
visSetIm(g, V_3,'red',6,extra);
hold on
set(gca,'zTick',[0:1.5:6]);
a = get(gca,'ZTickLabel');
set(gca,'ZTickLabel',a,'fontsize',18)
set(gca,'unit','normalized','position',[0.1 , 0.15 , 0.3 , 0.35 ])
% xlim([-1.3,1.3])
% ylim([-1.3,1.3])
xlabel('$x_1$','interpreter','latex','FontSize',25);
ylabel('$x_2$','interpreter','latex','FontSize',25);
grid off
lgd = legend('$\gamma = 0$','$\gamma = 0.5$','$\gamma = 1$',...
      'Initial State','Final State',' $\partial \mathcal D_{\gamma=0}$',...
      '$\partial \mathcal D_{\gamma=0.5}$','$\partial \mathcal D_{\gamma=1}$',...
      'interpreter','latex','FontSize',20);
% lgd.NumColumns = 8;
title('Trajectory and ROES','interpreter','latex','FontSize',20)

subplot(2,3,5)
plot(t(1:length(V(:,1))),V(:,1),'black','linewidth',1.5)
hold on
plot(t(1:length(V(:,2))),V(:,2),'blue','linewidth',1.5)
hold on
plot(t(1:length(V(:,3))),V(:,3),'red','linewidth',1.5)
grid off
set(gca,'zTick',[0:1.5:6]);
set(gca,'unit','normalized','position',[0.47,0.15,0.25,0.35])
a = get(gca,'ZTickLabel');
set(gca,'ZTickLabel',a,'fontsize',18)
% legend('$\gamma = 0$','$\gamma = 0.5$','$\gamma = 1$',...
%     'interpreter','latex','FontSize',18)


% set(gca,'unit','normalized','position',[0.7,0.7,0.2,0.25])
xlabel('$t$','interpreter','latex','FontSize',25);
ylabel('$V$','interpreter','latex','FontSize',25);
title('Value along the Trajectory','interpreter','latex','FontSize',20);
% ylim([-.3,1.3])
xlim([0,6])


% 
% subplot(3,2,3)
% plot(x(1,:,2),x(2,:,2),'b.');
% hold on
% plot(x(1,1,2),x(2,1,2),'ro');
% hold on
% visSetIm(g, V_2,'green',6);
% % set(gca,'unit','normalized','position',[0.4,0.37,0.2,0.25])
% % xlim([-1.3,1.3])
% % ylim([-1.3,1.3])
% xlabel('$x_1$','interpreter','latex','FontSize',14);
% ylabel('$x_2$','interpreter','latex','FontSize',14);
% legend('','','ROC')
% grid on
% 
% subplot(3,2,4)
% plot(t(1:length(V(:,2))),V(:,2),'b','linewidth',1)
% grid on
% % set(gca,'unit','normalized','position',[0.7,0.37,0.2,0.25])
% xlabel('$t$','interpreter','latex','FontSize',14);
% ylabel('$V$','interpreter','latex','FontSize',14);
% % ylim([-.3,1.3])
% xlim([0,6])
% 
% 
% 
% subplot(3,2,5)
% plot(x(1,:,3),x(2,:,3),'b.');
% hold on
% plot(x(1,1,3),x(2,1,3),'ro');
% hold on
% visSetIm(g, V_3,'cyan',6);
% % set(gca,'unit','normalized','position',[0.4,0.05,0.2,0.25])
% % xlim([-1.3,1.3])
% % ylim([-1.3,1.3])
% xlabel('$x_1$','interpreter','latex','FontSize',14);
% ylabel('$x_2$','interpreter','latex','FontSize',14);
% legend('','','ROC')
% grid on
% 
% subplot(3,2,6)
% plot(t(1:length(V(:,3))),V(:,3),'b','linewidth',1)
% % set(gca,'unit','normalized','position',[0.7,0.05,0.2,0.25])
% grid on
% xlabel('$t$','interpreter','latex','FontSize',14);
% ylabel('$V$','interpreter','latex','FontSize',14);
% % ylim([-.3,1.3])
% xlim([0,6])
% 

%%
figure
set(gcf,'unit','normalized','position',[0.2,0.2,0.64,0.3]);
subplot(2,3,1)
visFuncIm(g, V_1,'magenta',.3);
hold on
visSetIm(g, V_1,'magenta',6);
zlim([0,6])
c = camlight;
c.Position = [-30 -30 -30];
grid off
view(30,15)
set(gca,'zTick',[0:1.5:6]);
a = get(gca,'ZTickLabel');
set(gca,'ZTickLabel',a,'fontsize',18)
xlabel('$x_1$','interpreter','latex','FontSize',25);
ylabel('$x_2$','interpreter','latex','FontSize',25);
zlabel('$V_\gamma^\infty$','interpreter','latex','FontSize',25);
title('$\gamma = 0$','interpreter','latex','FontSize',25)

subplot(2,3,2)
visFuncIm(g, V_2,'green',.3);
hold on
visSetIm(g, V_2,'green',6);
c = camlight;
c.Position = [-30 -30 -30];
zlim([0,6])
grid off
view(30,15)
set(gca,'zTick',[0:1.5:6]);
a = get(gca,'ZTickLabel');
set(gca,'ZTickLabel',a,'fontsize',18)
xlabel('$x_1$','interpreter','latex','FontSize',25);
ylabel('$x_2$','interpreter','latex','FontSize',25);
% zlabel('$V_\gamma^\infty(\gamma = 0.1)$','interpreter','latex','FontSize',25);
title('$\gamma = 0.5$','interpreter','latex','FontSize',25)

subplot(2,3,3)
visFuncIm(g, V_3,'cyan',.3);
c = camlight;
c.Position = [-30 -30 -30];
zlim([0,6])
hold on
visSetIm(g, V_3,'cyan',6);
grid off
view(30,15)
set(gca,'zTick',[0:1.5:6]);
a = get(gca,'ZTickLabel');
set(gca,'ZTickLabel',a,'fontsize',18)
xlabel('$x_1$','interpreter','latex','FontSize',25);
ylabel('$x_2$','interpreter','latex','FontSize',25);
% zlabel('$V_\gamma^\infty(\gamma = 0.1)$','interpreter','latex','FontSize',25);
title('$\gamma = 1$','interpreter','latex','FontSize',25)
% figure
% h2 = visSetIm(g, V_1,'blue',1.3);
% hold on
% h2 = visSetIm(g, V_2,'green',1.3);
% hold on
% h2 = visSetIm(g, V_3,'red',1.3);
% grid on
% xlabel('x','interpreter','latex');
% ylabel('y','interpreter','latex');
% legend({'$\gamma = 0$','$\gamma = 0.5$','$\gamma = 1$'},'interpreter','latex')



%%
% 
% L = find(V_1 > 3);
% V_1(L) = NaN;
% Deriv_1 = computeGradients(g, V_1);
% L_1 = find(abs(Deriv_1{1}) >5);
% L_2 = find(abs(Deriv_1{2}) >5);
% V_1(L_1) = NaN;
% V_1(L_2) = NaN;
% 
% M = max(V_1,[],'all');
% figure
% h = visFuncIm(g, V_1,'blue',.3);
% hold on
% h = visSetIm(g,V_1,'b',M)
% 
% 
% 
% 
% 
