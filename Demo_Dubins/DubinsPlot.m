close all; clear; clc
      
data = importdata('V_gamma=03_c=05.mat');
data1 = importdata('V_gamma=0_2.mat');
g2 = importdata(['g_fine_2.mat']);
load('traj.mat');
load('t.mat');
load('traj_V.mat');
x2 = importdata('traj2.mat');
V2 = importdata('traj_V2.mat');

grid_min = [-3; -3; -pi]; % Lower corner of computation domain
grid_max = [3; 3; pi];    % Upper corner of computation domain
N = [101; 101; 51];         % Number of grid points per dimension
pdDims = 3;               % 3rd dimension is periodic

g1 = createGrid(grid_min, grid_max, N, pdDims);

[g2D, data2D] = proj(g1,data,[0 0 1],'min');
[g2D2, data2D2] = proj(g2,data1,[0 0 1],'min');

%%
figure
set(gcf,'unit','normalized','position',[0.2,0.2,0.64,0.6]);

subplot(2,3,1)
visFuncIm(g2D2,data2D2,'green',.3);
% hold on
% visSetIm(g2D2,data2D2,'green',0.5);
zlim([0,1.5])
set(gca,'zTick',0:0.5:1.6);
az = 30;
el = 10;
view(az,el);
c = camlight;
c.Position = [-30 -30 -30];
grid off
set(gca,'unit','normalized','position',[0.1,0.6,0.2,0.3])
a = get(gca,'ZTickLabel');
set(gca,'ZTickLabel',a,'fontsize',18);
xlabel('$x_1$','interpreter','latex','FontSize',25);
ylabel('$x_2$','interpreter','latex','FontSize',25);
zlabel('$V^\infty$','interpreter','latex','FontSize',25);
title('Projected Value Function','interpreter','latex','FontSize',20)


subplot(2,3,2)
visFuncIm(g2D,data2D,'blue',.3);
% hold on
% visSetIm(g2D,data2D,'blue',0);
set(gca,'unit','normalized','position',[0.4,0.6,0.2,0.3])

set(gca,'zTick',0.1:0.4:1.5);
grid off
a = get(gca,'ZTickLabel');
set(gca,'ZTickLabel',a,'fontsize',18);
zlim([-0,1])
xlim([-1,1])
ylim([-1,1])
az = 30;
el = 10;
view(az,el);
c = camlight;
c.Position = [-30 -30 -30];
xlabel('$x_1$','interpreter','latex','FontSize',25);
ylabel('$x_2$','interpreter','latex','FontSize',25);
zlabel('$V^\infty _\gamma (\gamma = 0.3)$','interpreter','latex','FontSize',25);
title('Projected CLVF','interpreter','latex','FontSize',20)

subplot(2,3,3)
set(gca,'unit','normalized','position',[0.7,0.6,0.2,0.3])
az = 30;
el = 10;
view(az,el);
visSetIm(g1,data,'blue',0);
hold on
a = get(gca,'ZTickLabel');
set(gca,'ZTickLabel',a,'fontsize',18);
% plot3(x(1,:),x(2,:),x(3,:),'m','linewidth',2);
xlabel('$x_1$','interpreter','latex','FontSize',25);
ylabel('$x_2$','interpreter','latex','FontSize',25);
zlabel('$x_3$','interpreter','latex','FontSize',25);
title('$\mathcal I_m^\infty$','interpreter','latex','FontSize',20)
% legend('$ \mathcal I^\infty_m$','interpreter','latex','FontSize',18)


subplot(2,3,4)
set(gca,'unit','normalized','position',[0.17 , 0.15 , 0.2 , 0.3 ])
[g2D_1, data2D_1] = proj(g1,data,[0 0 1],0);
[g2D_2, data2D_2] = proj(g1,data,[0 0 1],pi/2);
[g2D_3, data2D_3] = proj(g1,data,[0 0 1],-pi/2);

visSetIm(g2D_1,data2D_1,'blue',0);
hold on
visSetIm(g2D_2,data2D_2,'black',0);
hold on
visSetIm(g2D_3,data2D_3,'red',0);
xlim([-0.7,0.7])
ylim([-0.7,0.7])
a = get(gca,'ZTickLabel');
set(gca,'ZTickLabel',a,'fontsize',18);
xlabel('$x_1$','interpreter','latex','FontSize',25);
ylabel('$x_2$','interpreter','latex','FontSize',25);
title('Slices of $\mathcal I_m^\infty$','interpreter','latex','FontSize',20)
legend('$x_3 = 0$','$x_3 = 2\pi/3$','$x_3 = -\pi/2$','interpreter','latex','FontSize',20)

subplot(2,3,5)
set(gca,'unit','normalized','position',[0.6,0.15,0.2,0.3])

t = 0:0.1:2.1*pi;
c1 = 0.51*sin(t);
c2 = 0.51*cos(t);
c3 = 0.28*c1;
c4 = 0.28*c2;


h1 =plot(x(1,:),x(2,:),'m','linewidth',2);
hold on
% h2 =plot(x2(1,:),x2(2,:),'r');
extra.LineWidth = 2;
extra.LineStyle = '-.';
h3 = visSetIm(g2D2,data2D2,'blue',0.51,extra);
hold on
h4 = fill(c1,c2,'b');
h4.FaceAlpha = 0.3;
hold on
h5 = fill(c3,c4,'w');

legend([h3,h4],{'$\partial \mathcal I^\infty_m$','$ \mathcal I^\infty_m$'},...
    'interpreter','latex','FontSize',18)
grid off
a = get(gca,'ZTickLabel');
set(gca,'ZTickLabel',a,'fontsize',18);
xlabel('$x_1$','interpreter','latex','FontSize',25);
ylabel('$x_2$','interpreter','latex','FontSize',25);
title('Trajectrory Projected on $x_1 -x_2$ ','interpreter','latex','FontSize',20)


% subplot(2,3,6)
% set(gca,'unit','normalized','position',[0.7,0.15,0.2,0.3])
% 
% plot(sim_t(1:1501),V(1:1501),'m','linewidth',2)
% grid off
% a = get(gca,'ZTickLabel');
% set(gca,'ZTickLabel',a,'fontsize',18);
% % legend('Value along Trajectory','FontSize',18)
% xlabel('t','interpreter','latex','FontSize',25)
% ylabel('Value','interpreter','latex','FontSize',25)
% title('Value along the Trajectrory','interpreter','latex','FontSize',20)

%%
% [g2D, data2D] = proj(g1,data,[0 0 1],'min');

figure
set(gcf,'unit','normalized','position',[0.2,0.2,0.64,0.44]);

subplot(1,2,1)
az = 30;
el = 10;
view(az,el);
visSetIm(g2,data1,'blue',0.5);
hold on
% plot3(x(1,1500:2002),x(2,1500:2002),x(3,1500:2002),'m','linewidth',2);
xlabel('$x_1$','interpreter','latex','FontSize',25);
ylabel('$x_2$','interpreter','latex','FontSize',25);
zlabel('$x_3$','interpreter','latex','FontSize',25);
title('The Smallest Control Invariant Set $\mathcal I_m^\infty$','interpreter','latex','FontSize',20)

subplot(1,2,2)
[g2D_1, data2D_1] = proj(g1,data,[0 0 1],0);
[g2D_2, data2D_2] = proj(g1,data,[0 0 1],pi/2);
[g2D_3, data2D_3] = proj(g1,data,[0 0 1],-pi/2);

visSetIm(g2D_1,data2D_1,'blue',0);
hold on
visSetIm(g2D_2,data2D_2,'green',0);
hold on
visSetIm(g2D_3,data2D_3,'red',0);
xlim([-1,1])
ylim([-1,1])
xlabel('$x_1$','interpreter','latex','FontSize',25);
ylabel('$x_2$','interpreter','latex','FontSize',25);
title('Slices of $\mathcal I_m^\infty$ ','interpreter','latex','FontSize',20)
legend('$x_3 = 0$','$x_3 = 2\pi/3$','$x_3 = -\pi/2$','interpreter','latex','FontSize',20)