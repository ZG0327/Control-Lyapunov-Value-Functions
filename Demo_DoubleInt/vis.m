%This is a test file to see different functions
clear; close all; clc

%% import the grid and value functions
g = importdata('g.mat');

V_2norm = importdata('V0_2norm.mat');
V_2min = min(V_2norm,[],'all');
V_2norm = V_2norm - V_2min;
x_2n = importdata('traj_2n.mat');

V_infnorm = importdata('V0_infnorm.mat');
V_imin = min(V_infnorm,[],'all');
V_infnorm = V_infnorm - V_imin;
x_in = importdata('traj_in.mat');

V_quadcost = importdata('V0_qc.mat');
V_qmin = min(V_quadcost,[],'all');
V_quadcost = V_quadcost - V_qmin;
x_qc = importdata('traj_qc.mat');


%% figures
az = 40;
el = 30;
a = 0.007;

figure
set(gcf,'unit','normalized','position',[0.2,0.2,0.64,0.6]);

subplot(2,3,1)
c = camlight;
c.Position = [-30 -30 -30];
view(az,el);
visFuncIm(g,V_2norm,'b',0.6)
hold on 
visSetIm(g,V_2norm,'k',a)
grid off
xlabel('$x_1$','interpreter','latex','FontSize',25);
ylabel('$x_2$','interpreter','latex','FontSize',25);
zlabel('$V^\infty$','interpreter','latex','FontSize',25);
title('Using 2-norm','interpreter','latex','FontSize',20)

subplot(2,3,2)
c = camlight;
c.Position = [-30 -30 -30];
view(az,el);
visFuncIm(g,V_infnorm,'r',0.6)
hold on
visSetIm(g,V_infnorm,'k',a)
grid off
xlabel('$x_1$','interpreter','latex','FontSize',25);
ylabel('$x_2$','interpreter','latex','FontSize',25);
% zlabel('$V^\infty$','interpreter','latex','FontSize',25);
title('Using infinity-norm','interpreter','latex','FontSize',20)

subplot(2,3,3)
c = camlight;
c.Position = [-30 -30 -30];
view(az,el);
visFuncIm(g,V_quadcost,'c',0.6)
hold on
visSetIm(g,V_quadcost,'k',a)
grid off
xlabel('$x_1$','interpreter','latex','FontSize',25);
ylabel('$x_2$','interpreter','latex','FontSize',25);
% zlabel('$V^\infty$','interpreter','latex','FontSize',25);
title('Using quadratic cost','interpreter','latex','FontSize',20)

subplot(2,3,4)
view(az,el);
visSetIm(g,V_2norm,'b',0.01)
hold on
plot(x_2n(1,:),x_2n(2,:));
xlabel('$x_1$','interpreter','latex','FontSize',25);
ylabel('$x_2$','interpreter','latex','FontSize',25);
zlabel('$x_3$','interpreter','latex','FontSize',25);

subplot(2,3,5)
view(az,el);
visSetIm(g,V_infnorm,'r',0.01)
hold on
plot(x_in(1,:),x_in(2,:));

xlabel('$x_1$','interpreter','latex','FontSize',25);
ylabel('$x_2$','interpreter','latex','FontSize',25);
zlabel('$x_3$','interpreter','latex','FontSize',25);

subplot(2,3,6)
view(az,el);
visSetIm(g,V_quadcost,'c',0.01)
hold on
plot(x_qc(1,:),x_qc(2,:));
xlabel('$x_1$','interpreter','latex','FontSize',25);
ylabel('$x_2$','interpreter','latex','FontSize',25);
zlabel('$x_3$','interpreter','latex','FontSize',25);



%%
figure

view(az,el);
visSetIm(g,V_2norm,'b',a)
hold on

visSetIm(g,V_infnorm,'r',a)
hold on

visSetIm(g,V_quadcost,'c',a)
