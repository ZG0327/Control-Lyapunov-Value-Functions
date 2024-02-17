close all; clear; clc

% This is a toy example of CLVF-QP 
% \dot x1 = x2 + d1
% \dot x2 = -a*x2 + b*sin(x1) + u + d2, 
% u \in [-2,2], d1 = 0, d2 \in [-0.5,0.5]. 
% Note this is equavalent to u \in [-1.5,1.5] and no d.
% where a=b=1. 

V = importdata('value_gamma=0.mat')-0.14;

g = importdata('grid.mat');
params = importdata('params.mat');
% params.d_min = [-0;-0];
% params.d_max = [0;0];
% params.u_min = -1.5;
% params.u_max = 1.5;
% Problem setup
dt = 0.2;
sim_t = [0:dt:30];
delta = 1e-3;

x0 = [-1;2];
x = nan(2,length(sim_t));
u = nan(1,length(sim_t));
d = nan(2,length(sim_t));

x(:,1) = x0;
x_opt = x;
x_qp = x;
x_sontag = x;
u_opt = u;
u_qp = u;
u_sontag = u;
d_opt = d;
d_qp = d;
d_sontag = d;

H = 1;
H_delta = [ 0 , 0 ; 0 , 1];
f_delta = [0,0];
lb_delta = [params.u_min;0];
ub_delta = [params.u_max;inf];

gamma = 0;

options = optimoptions('quadprog', 'ConstraintTolerance', 1e-6, 'StepTolerance', 1e-12, 'Display','iter');
t = 0;
lb = params.u_min;
ub = params.u_max;

% data2 = data2 - min(data2,[],'all');
Deriv = computeGradients(g, V);
grad1 = Deriv{1};
grad2 = Deriv{2};



%%
% using value function
for i = 1 : length(sim_t) 
    % QP Controller 
    V_qp(i) = eval_u(g,V,x_qp(:,i));
    deriv1_qp = eval_u(g,grad1,x_qp(:,i));
    deriv2_qp = eval_u(g,grad2,x_qp(:,i));
    LgV_qp = deriv2_qp;
    LfV_qp = x_qp(2,i)*deriv1_qp + ...
        deriv2_qp * (-x_qp(2,i)+sin(x_qp(1,i)));

    d_qp(1,i) = (deriv1_qp>=0)*params.d_max(1) +...
              (deriv1_qp<0)* params.d_min(1);
    d_qp(2,i) = (deriv2_qp>=0)*params.d_max(2) +...
              (deriv2_qp<0)* params.d_min(2);
    max_D = d_qp(1,i)*deriv1_qp+d_qp(2,i)*deriv2_qp;
    A_delta = [ LgV_qp , -1 ; 0 , -1 ];
    b_delta = [ -LfV_qp-gamma*V_qp(i)-max_D ; 0];
    [u_delta,~,] = quadprog(H_delta,f_delta,A_delta,b_delta,[],[],lb_delta,ub_delta);

    A1 = LgV_qp;
    b1 = -LfV_qp - gamma*V_qp(i)+u_delta(2)-max_D; % The 0.01 here accounts for the 
                                              % convergence threshold of
                                              % CLVF

    [u_qp(i),~,flag] = quadprog(H,0,A1,b1,[],[],lb,ub);

    [~, xs_qp] = ode45(@(t, s) ...
        sys(t, s, u_qp(i),d_qp(:,i)), [t t+dt], x_qp(:,i));
    
    x_qp(:,i+1) = xs_qp(end,:);


    % Optimal controller
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

    % sontag controller
    V_sontag(i) = eval_u(g,V,x_sontag(:,i));
    deriv1_sontag = eval_u(g,grad1,x_sontag(:,i));
    deriv2_sontag = eval_u(g,grad2,x_sontag(:,i));
    LgV_sontag = deriv2_sontag;
    LfV_sontag = x_sontag(2,i)*deriv1_sontag + ...
        deriv2_sontag * (-x_sontag(2,i)+sin(x_sontag(1,i)));

    d_sontag(1,i) = (deriv1_sontag>=0)*params.d_max(1) +...
                  (deriv1_sontag<0)* params.d_min(1);
    d_sontag(2,i) = (deriv2_sontag>=0)*params.d_max(2) +...
                  (deriv2_sontag<0)* params.d_min(2);
    u_sontag(i) = sontag(LfV_sontag,LgV_sontag,params);
    [~, xs_sontag] = ode45(@(t, s) ...
        sys(t, s, u_sontag(i),d_sontag(:,i)), [t t+dt], x_sontag(:,i));

    x_sontag(:,i+1) = xs_sontag(end,:);

    t = t+dt;
    %     norm_x_qp(i) = norm(x_qp(:,i)-[0;0]);
% %     if norm_x(i) <= 0.01
% %         break
% %     end
end



%% Figures and Videos

figure
visSetIm(g,V,'k',0.01)
hold on
visFuncIm(g,V,'b',0.3)
c = camlight;
c.Position = [-30 -30 -30];
grid off
view(75,15)
xlabel('$x_1$','interpreter','latex','FontSize',25);
ylabel('$x_2$','interpreter','latex','FontSize',25);
zlabel('R-CLVF value','interpreter','latex','FontSize',25);
title('R-CLVF','interpreter','latex','FontSize',20)


figure
set(gcf,'unit','normalized','position',[0.1,0.25,0.8,0.55]);
subplot(1,2,1)
plot(x0(1),x0(2),'r*')
hold on
plot(x_qp(1,1:length(x_qp)-1),x_qp(2,1:length(x_qp)-1),'b')
hold on
plot(x_opt(1,1:length(x_opt)-1),x_opt(2,1:length(x_opt)-1),'r')
hold on
plot(x_sontag(1,1:length(x_opt)-1),x_sontag(2,1:length(x_sontag)-1),'g')
% set(gca,'unit','normalized','position',[0.1,0.6,0.2,0.3])
hold on
visSetIm(g,V,'k',0.02);
xlabel('$x_1$','interpreter','latex','FontSize',25);
ylabel('$x_2$','interpreter','latex','FontSize',25);
legend('IC point','qp','opt','sontag', 'smallest CIS' ,...
    'interpreter','latex','FontSize',18)
title('value function','interpreter','latex','FontSize',20)

subplot(1,2,2)
axis([0 15 ,-0.1 3])
grid on
% plot(sim_t(1:length(V_qp)),V_qp(1)*exp(-gamma*sim_t(1:length(V_qp))))
% hold on
plot(sim_t(1:length(V_qp)),V_qp,'b')
hold on
plot(sim_t(1:length(V_opt)),V_opt,'r')
hold on
plot(sim_t(1:length(V_sontag)),V_sontag,'g')
xlabel('$t$','interpreter','latex','FontSize',25);
ylabel('value','interpreter','latex','FontSize',25);
legend('qp','opt','sontag','interpreter','latex','FontSize',18)
title('Decay of value function','interpreter','latex','FontSize',20)


% figure % 3D
% plot3(x0(1),x0(2),eval_u(g,V,[x0(1),x0(2)]),'r*')
% hold on
% plot3(x_qp(1,1:length(x_qp)-1),x_qp(2,1:length(x_qp)-1),V_qp)
% hold on
% plot3(x_opt(1,1:length(x_opt)-1),x_opt(2,1:length(x_opt)-1),V_opt)
% hold on
% plot3(x_sontag(1,1:length(x_opt)-1),x_sontag(2,1:length(x_sontag)-1),V_sontag)
% hold on
% visFuncIm(g,V,'blue',0.2);
% c = camlight;
% c.Position = [-30 -30 -30];
% grid off
% view(45,45)
% set(gcf,'unit','normalized','position',[0.1,0.1,0.8,0.8]);
% xlabel('x1')
% ylabel('x2')
% zlabel('value')
% legend('IC point','qp','opt','sontag')
% title('value function')

% Plot the value function and control
figure
set(gcf,'unit','normalized','position',[0.1,0.25,0.8,0.55]);

subplot(3,2,1)
grid on
axis([0 15,-2.2 2.2])
plot(sim_t(1:length(sim_t)),u_qp,'b')
title('qp controller','interpreter','latex','FontSize',20)

subplot(3,2,3)
grid on
axis([0 15,-2.2 2.2])
plot(sim_t(1:length(sim_t)),u_opt,'r')
title('opt controller','interpreter','latex','FontSize',20)

subplot(3,2,5)
grid on
axis([0 15,-2.2 2.2])
xlabel('$t$','interpreter','latex','FontSize',25);
ylabel('$u$','interpreter','latex','FontSize',25);
plot(sim_t(1:length(sim_t)),u_sontag,'g')
title('sontag controller','interpreter','latex','FontSize',20)
  
subplot(3,2,2)
grid on
axis([0 15,-2.2 2.2])
plot(sim_t(1:length(sim_t)),d_qp,'b')
title('qp dstb','interpreter','latex','FontSize',20)

subplot(3,2,4)
grid on
axis([0 15,-2.2 2.2])
plot(sim_t(1:length(sim_t)),d_opt,'r')
title('opt ditb','interpreter','latex','FontSize',20)

subplot(3,2,6)
grid on
axis([0 15,-2.2 2.2])
xlabel('$t$','interpreter','latex','FontSize',25);
ylabel('$u$','interpreter','latex','FontSize',25);
plot(sim_t(1:length(sim_t)),d_sontag,'g')
title('sontag dstb','interpreter','latex','FontSize',20)



%%
function dydt = sys(t,s,u,d)
dydt = [s(2)+d(1);(-s(2)+sin(s(1)))+u+d(2)];
end

function u = sontag(LfV,LgV,params)
    umin = params.u_min;
    umax = params.u_max;
    u_size = size(LgV,2);
    u = nan(u_size,1);
    for i = 1 : u_size
        if LgV(i) == 0
            u(i) = 0;
        else
            u(i) = -(LfV + sqrt(LfV^2+LgV(i)^4))/LgV(i);
            if u(i)>umax
                u(i) = umax;
            elseif u(i)<umin
                u(i) = umin;
            end
        end
    end
end