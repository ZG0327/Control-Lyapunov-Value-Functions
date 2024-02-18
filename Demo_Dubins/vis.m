%This is a test file to see different functions
clear; close all; clc

%% import the grid and value functions
g = importdata('g.mat');

V_2norm = importdata('V_2norm.mat');
V_2min = min(V_2norm,[],'all');
V_2norm = V_2norm - V_2min;


V_infnorm = importdata('V_infnorm.mat');
V_imin = min(V_infnorm,[],'all');
V_infnorm = V_infnorm - V_imin;

V_quadcost = importdata('V_Quacost.mat');
V_qmin = min(V_quadcost,[],'all');
V_quadcost = V_quadcost - V_qmin;

%% Proj the value functions
[g_p,V_2p] = proj(g,V_2norm,[0 0 1],'min');
[~,V_ip] = proj(g,V_infnorm,[0 0 1],'min');
[~,V_qp] = proj(g,V_quadcost,[0 0 1],'min');

%% figures
az = 40;
el = 30;

figure
set(gcf,'unit','normalized','position',[0.2,0.2,0.64,0.6]);

subplot(2,3,1)
c = camlight;
c.Position = [-30 -30 -30];
view(az,el);
visFuncIm(g_p,V_2p,'b',0.6)
hold on 
visSetIm(g_p,V_2p,'k',0.01)
grid off
xlabel('$x_1$','interpreter','latex','FontSize',25);
ylabel('$x_2$','interpreter','latex','FontSize',25);
zlabel('$V^\infty$','interpreter','latex','FontSize',25);
title('Using 2-norm','interpreter','latex','FontSize',20)

subplot(2,3,2)
c = camlight;
c.Position = [-30 -30 -30];
view(az,el);
visFuncIm(g_p,V_ip,'r',0.6)
hold on
visSetIm(g_p,V_ip,'k',0.02)
grid off
xlabel('$x_1$','interpreter','latex','FontSize',25);
ylabel('$x_2$','interpreter','latex','FontSize',25);
% zlabel('$V^\infty$','interpreter','latex','FontSize',25);
title('Using infinity-norm','interpreter','latex','FontSize',20)

subplot(2,3,3)
c = camlight;
c.Position = [-30 -30 -30];
view(az,el);
visFuncIm(g_p,V_qp,'c',0.6)
hold on
visSetIm(g_p,V_qp,'k',0.01)
grid off
xlabel('$x_1$','interpreter','latex','FontSize',25);
ylabel('$x_2$','interpreter','latex','FontSize',25);
% zlabel('$V^\infty$','interpreter','latex','FontSize',25);
title('Using quadratic-cost','interpreter','latex','FontSize',20)

subplot(2,3,4)
view(az,el);
visSetIm(g,V_2norm,'b',0.01)
xlabel('$x_1$','interpreter','latex','FontSize',25);
ylabel('$x_2$','interpreter','latex','FontSize',25);
zlabel('$x_3$','interpreter','latex','FontSize',25);

subplot(2,3,5)
view(az,el);
visSetIm(g,V_infnorm,'r',0.01)
xlabel('$x_1$','interpreter','latex','FontSize',25);
ylabel('$x_2$','interpreter','latex','FontSize',25);
zlabel('$x_3$','interpreter','latex','FontSize',25);

subplot(2,3,6)
view(az,el);
visSetIm(g,V_quadcost,'c',0.01)
xlabel('$x_1$','interpreter','latex','FontSize',25);
ylabel('$x_2$','interpreter','latex','FontSize',25);
zlabel('$x_3$','interpreter','latex','FontSize',25);



%%
figure
set(gcf,'unit','normalized','position',[0.2,0.2,0.64,0.4]);

subplot(1,3,1)
view(az,el);
visSetIm(g,V_2norm,'b',0.01)

subplot(1,3,2)
view(az,el);
visSetIm(g,V_infnorm,'r',0.01)

subplot(1,3,3)
view(az,el);
visSetIm(g,V_quadcost,'c',0.01)