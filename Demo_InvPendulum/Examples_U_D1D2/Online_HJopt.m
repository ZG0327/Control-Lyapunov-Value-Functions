close all; clear; clc

% This is a toy example of CLVF-QP 
% \dot x1 = x2 + d1
% \dot x2 = -a*x2 + b*sin(x1) + u + d2, 
% u \in [-2,2], d1 = 0, d2 \in [-0.5,0.5]. 
% Note this is equavalent to u \in [-1.5,1.5] and no d.
% where a=b=1. 

% V = importdata('value_gamma=0.mat');
% V_min = min(V,[],'all');
% V = V-V_min;
% V = importdata('value_gamma=0.1.mat');
V = importdata('value_g=01_601.mat');

g = importdata('grid_large.mat');

% V_s = importdata('value_small_2.mat');
% g_s = importdata('grid_small.mat');

params = importdata('params.mat');

% Problem setup
dt = 0.1;
sim_t = [0:dt:300];
delta = 1e-3;

x0 = [0;2];
x = nan(2,length(sim_t));
u = nan(1,length(sim_t));
d = nan(2,length(sim_t));

x(:,1) = x0;
x_opt = x;
u_opt = u;
d_opt = d;
gamma = 0;
t = 0;

% data2 = data2 - min(data2,[],'all');
Deriv = computeGradients(g, V);
grad1 = Deriv{1};
grad2 = Deriv{2};

% Deriv_s = computeGradients(g_s, V_s);
% grad1_s = Deriv_s{1};
% grad2_s = Deriv_s{2};

D_g1 = computeGradients(g, grad1);
D_g2 = computeGradients(g, grad2);

G1_g1 = D_g1{1};
G2_g1 = D_g1{2};
G1_g2 = D_g2{1};
G2_g2 = D_g2{2};

I_11 = find(abs(G1_g1)>=5);
I_12 = find(abs(G2_g1)>=5);
I_1 = union(I_11,I_12);

I_21 = find(abs(G1_g2)>=3);
I_22 = find(abs(G2_g2)>=3);
I_2 = union(I_21,I_22);

xs1 = g.xs{1};
xs2 = g.xs{2};



%%
% using value function
for i = 1 : length(sim_t) 
       % Optimal controller
%        if abs(x_opt(1,i)) <= 0.8 && abs(x_opt(2,i))<=0.8
%            V_opt(i) = eval_u(g_s,V_s,x_opt(:,i));
%            deriv1_opt = eval_u(g_s,grad1_s,x_opt(:,i));
%            deriv2_opt = eval_u(g_s,grad2_s,x_opt(:,i));
%        else
%            V_opt(i) = eval_u(g,V,x_opt(:,i));
%            deriv1_opt = eval_u(g,grad1,x_opt(:,i));
%            deriv2_opt = eval_u(g,grad2,x_opt(:,i));
%        end
    V_opt(i) = eval_u(g,V,x_opt(:,i));
    deriv1_opt = eval_u(g,grad1,x_opt(:,i));
    deriv2_opt = eval_u(g,grad2,x_opt(:,i));
    u_opt(i) = (deriv2_opt>=0)*params.u_min +...
       (deriv2_opt<0)* params.u_max;
    d_opt(1,i) = (deriv1_opt>=0)*params.d_max(1) +...
        (deriv1_opt<0)* params.d_min(1);
    d_opt(2,i) = (deriv2_opt>=0)*params.d_max(2) +...
       (deriv2_opt<0)* params.d_min(2);
    [~, xs_opt] = ode45(@(t, s) ...
      sys(t, s, u_opt(i),d_opt(:,i)), [t t+dt], x_opt(:,i));

    x_opt(:,i+1) = xs_opt(end,:);
    t = t+dt;
end



%% Figures and Videos

% figure
% visSetIm(g,V,'k',0.01)
% hold on
% visFuncIm(g,V,'b',0.3)
% c = camlight;
% c.Position = [-30 -30 -30];
% grid off
% view(75,15)
% xlabel('$x_1$','interpreter','latex','FontSize',25);
% ylabel('$x_2$','interpreter','latex','FontSize',25);
% zlabel('R-CLVF value','interpreter','latex','FontSize',25);
% title('R-CLVF','interpreter','latex','FontSize',20)


figure
set(gcf,'unit','normalized','position',[0.1,0.25,0.8,0.55]);
subplot(1,2,1)
visSetIm(g,V,'m',V_opt(1))
hold on
plot(xs1(I_1),xs2(I_1),'r.')
hold on
plot(xs1(I_2),xs2(I_2),'g.')
hold on
plot(x0(1),x0(2),'k*')
hold on
plot(x_opt(1,1:length(x_opt)-1),x_opt(2,1:length(x_opt)-1),'k')
% set(gca,'unit','normalized','position',[0.1,0.6,0.2,0.3])
hold on
visSetIm(g,V,'k',0.02);
xlabel('$x_1$','interpreter','latex','FontSize',25);
ylabel('$x_2$','interpreter','latex','FontSize',25);
legend('Initial Level Set','non diff wrt $x_1$','non diff wrt $x_2$','IC point','opt', 'smallest CIS' ,...
    'interpreter','latex','FontSize',18)
title('value function','interpreter','latex','FontSize',20)

subplot(1,2,2)
axis([0 15 ,-0.1 3])
grid on
plot(sim_t(1:length(V_opt)),V_opt,'r')
xlabel('$t$','interpreter','latex','FontSize',25);
ylabel('value','interpreter','latex','FontSize',25);
legend('opt','interpreter','latex','FontSize',18)
title('Decay of value function','interpreter','latex','FontSize',20)

% Plot the value function and control
figure
set(gcf,'unit','normalized','position',[0.1,0.25,0.8,0.55]);

subplot(3,1,1)
grid on
axis([0 15,-2.2 2.2])
plot(sim_t(1:length(sim_t)),u_opt,'r')
title('opt controller','interpreter','latex','FontSize',20)

subplot(3,1,2)
grid on
axis([0 15,-2.2 2.2])
plot(sim_t(1:length(sim_t)),d_opt(1,:),'r')
title('opt dstb 1','interpreter','latex','FontSize',20)


subplot(3,1,3)
grid on
axis([0 15,-2.2 2.2])
plot(sim_t(1:length(sim_t)),d_opt(2,:),'r')
title('opt dstb 2','interpreter','latex','FontSize',20)

%%
sys(0,x_opt(:,end),u_opt(end),d_opt(:,end))

function dydt = sys(t,s,u,d)
dydt = [s(2)+d(1);(-s(2)+sin(s(1)))+u+d(2)];
end
