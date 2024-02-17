close all; clear; clc

% This is a toy example of CLVF-QP 
% \dot x1 = x2 + d1
% \dot x2 = -a*x2 + b*sin(x1) + u + d2, 
% u \in [-2,2], d1 = 0, d2 \in [-0.5,0.5]. 
% Note this is equavalent to u \in [-1.5,1.5] and no d.
% where a=b=1. 

% V = importdata('value_gamma=0.1.mat');
% g = importdata('grid.mat');
V = importdata('value_g=01_601.mat');
g = importdata('grid_large.mat');
% V_s = importdata('value_small_2.mat');
% g_s = importdata('grid_small.mat');

params = importdata('params.mat');
% params.d_min = [-0;-0];
% params.d_max = [0;0];
% params.u_min = -1.5;
% params.u_max = 1.5;
% Problem setup
dt = 0.01;
sim_t = [0:dt:20];
delta = 1e-3;

x0 = [0;-1];
dyn = InvertedPendulum(x0, params);

x = nan(2,length(sim_t));
u = nan(1,length(sim_t));
d = nan(2,length(sim_t));

x(:,1) = x0;
x_qp = x;
u_qp = u;
d_qp = d;

H = 1;
H_delta = [ 0 , 0 ; 0 , 1];
f_delta = [0,0];
lb_delta = [params.u_min;0];
ub_delta = [params.u_max;inf];

gamma = 0.1;

options = optimoptions('quadprog', 'ConstraintTolerance', 1e-6, 'StepTolerance', 1e-12, 'Display','iter');
t = 0;
lb = params.u_min;
ub = params.u_max;

% data2 = data2 - min(data2,[],'all');
[Deriv , DerivL, DerivR] = computeGradients(g, V);
grad1 = Deriv{1};
grad2 = Deriv{2};
grad1_L = DerivL{1};
grad2_L = DerivL{2};
grad1_R = DerivR{1};
grad2_R = DerivR{2};

D_g1 = computeGradients(g, grad1);
D_g2 = computeGradients(g, grad2);

G1_g1 = D_g1{1};
G2_g1 = D_g1{2};
G1_g2 = D_g2{1};
G2_g2 = D_g2{2};

I_11 = find(abs(G1_g1)>=2);
I_12 = find(abs(G2_g1)>=2);
I_1 = union(I_11,I_12);

I_21 = find(abs(G1_g2)>=3);
I_22 = find(abs(G2_g2)>=3);
I_2 = union(I_21,I_22);

% Deriv_s = computeGradients(g_s, V_s);
% grad1_s = Deriv_s{1};
% grad2_s = Deriv_s{2};


xs1 = g.xs{1};
xs2 = g.xs{2};

%%
% using value function
for i = 1 : length(sim_t) 
    i
%     if abs(x_qp(1,i)) <= 0.8 && abs(x_qp(2,i))<=0.8
%            V_qp(i) = eval_u(g_s,V_s,x_qp(:,i));
%            deriv1_qp = eval_u(g_s,grad1_s,x_qp(:,i));
%            deriv2_qp = eval_u(g_s,grad2_s,x_qp(:,i));
%        else
%            V_qp(i) = eval_u(g,V,x_qp(:,i));
%            deriv1_qp = eval_u(g,grad1,x_qp(:,i));
%            deriv2_qp = eval_u(g,grad2,x_qp(:,i));
%     end

    V_qp(i) = eval_u(g,V,x_qp(:,i));
    deriv1_qp = eval_u(g,grad1,x_qp(:,i));
    deriv2_qp = eval_u(g,grad2,x_qp(:,i));
    deriv1_qp_L = eval_u(g,grad1_L,x_qp(:,i));
    deriv2_qp_L = eval_u(g,grad2_L,x_qp(:,i));
    deriv1_qp_R = eval_u(g,grad1_R,x_qp(:,i));
    deriv2_qp_R = eval_u(g,grad2_R,x_qp(:,i));
   
    d_qp(1,i) = (deriv1_qp>=0)*params.d_max(1) +...
            (deriv1_qp<0)* params.d_min(1);
    d_qp(2,i) = (deriv2_qp>=0)*params.d_max(2) +...
            (deriv2_qp<0)* params.d_min(2);
    max_D = d_qp(1,i)*deriv1_qp+d_qp(2,i)*deriv2_qp;

    d_qp_L(1,i) = (deriv1_qp_L>=0)*params.d_max(1) +...
            (deriv1_qp_L<0)* params.d_min(1);
    d_qp_L(2,i) = (deriv2_qp_L>=0)*params.d_max(2) +...
            (deriv2_qp_L<0)* params.d_min(2);
    max_D_L = d_qp_L(1,i)*deriv1_qp_L+d_qp_L(2,i)*deriv2_qp_L;

    d_qp_R(1,i) = (deriv1_qp_R>=0)*params.d_max(1) +...
            (deriv1_qp_R<0)* params.d_min(1);
    d_qp_R(2,i) = (deriv2_qp_R>=0)*params.d_max(2) +...
            (deriv2_qp_R<0)* params.d_min(2);
    max_D_R = d_qp_R(1,i)*deriv1_qp_R+d_qp_R(2,i)*deriv2_qp_R;

    qpind(i) = abs(deriv1_qp_L - deriv1_qp_R);

    LgV_qp = deriv2_qp;
    LfV_qp = x_qp(2,i)*deriv1_qp + ...
            deriv2_qp * (-x_qp(2,i)+sin(x_qp(1,i)));
   
    LgV_qp_L = deriv2_qp_L;
    LfV_qp_L = x_qp(2,i)*deriv1_qp_L + ...
            deriv2_qp_L * (-x_qp(2,i)+sin(x_qp(1,i)));
    A1_L = LgV_qp_L;
    b1_L = -LfV_qp_L - gamma*V_qp(i)+ 0.0001 -max_D_L;

    LgV_qp_R = deriv2_qp_R;
    LfV_qp_R = x_qp(2,i)*deriv1_qp_R + ...
            deriv2_qp_R * (-x_qp(2,i)+sin(x_qp(1,i)));
    A1_R = LgV_qp_R;
    b1_R = -LfV_qp_R - gamma*V_qp(i)+ 0.0001 -max_D_R;

    if qpind(i) < 0.1
        
        A_delta = [ LgV_qp , -1 ; 0 , -1 ];
        b_delta = [ -LfV_qp-gamma*V_qp(i)-max_D ; 0];
        [u_delta(:,i),~,] = quadprog(H_delta,f_delta,A_delta,b_delta,[],[],lb_delta,ub_delta);
        A1 = LgV_qp;
        b1 = -LfV_qp - gamma*V_qp(i)+ u_delta(2,i) -max_D; % The 0.01 here accounts for the
        % convergence threshold of
        % CLVF
    else
        if abs(b1_L/A1_L) <= 2
            A1 = A1_L;
            b1 = b1_L;
        elseif abs(b1_R/A1_R) <= 2
            A1 = A1_R;
            b1 = b1_R;
        end
    end
    [u_qp(i),~,flag] = quadprog(H,0,A1,b1,[],[],lb,ub);

    [~, xs_qp] = ode45(@(t, s) ...
        sys(t, s, u_qp(i),d_qp(:,i)), [t t+dt], x_qp(:,i));
    
    x_qp(:,i+1) = xs_qp(end,:);

    t = t+dt;

end



%% Figures and Videos

figure
set(gcf,'unit','normalized','position',[0.1,0.25,0.8,0.55]);
subplot(1,2,1)
plot(x0(1),x0(2),'r*')
% set(gca,'unit','normalized','position',[0.1,0.6,0.2,0.3])
hold on
visSetIm(g,V,'k',0.05);
hold on
plot(xs1(I_1),xs2(I_1),'r.')
hold on
plot(xs1(I_2),xs2(I_2),'g.')
hold on
plot(x_qp(1,1:length(x_qp)-1),x_qp(2,1:length(x_qp)-1),'b')
xlabel('$x_1$','interpreter','latex','FontSize',25);
ylabel('$x_2$','interpreter','latex','FontSize',25);
legend('IC point', 'smallest CIS','$\frac{dV}{dx_1}=0$', '$\frac{dV}{dx_2}=0$',...
    'qp','interpreter','latex','FontSize',18)
title('value function','interpreter','latex','FontSize',20)

subplot(1,2,2)
axis([0 15 ,-0.1 3])
grid on
% plot(sim_t(1:length(V_qp)),V_qp(1)*exp(-gamma*sim_t(1:length(V_qp))))
% hold on
plot(sim_t(1:length(V_qp)),V_qp,'b')
xlabel('$t$','interpreter','latex','FontSize',25);
ylabel('value','interpreter','latex','FontSize',25);
legend('qp','interpreter','latex','FontSize',18)
title('Decay of value function','interpreter','latex','FontSize',20)


% Plot the value function and control
figure
set(gcf,'unit','normalized','position',[0.1,0.25,0.8,0.55]);

subplot(2,2,1)
grid on
axis([0 15,-2.2 2.2])
plot(sim_t(1:length(sim_t)),u_qp,'b')
title('qp controller','interpreter','latex','FontSize',20)

subplot(2,2,2)
grid on
axis([0 15,-2.2 2.2])
plot(sim_t(1:length(sim_t)),u_delta(2,:),'r')
hold on
plot(sim_t(1:length(V_qp)),0.0001*ones(length(V_qp)),'k');
ylim([0,0.0002]);
title('Slack value','interpreter','latex','FontSize',20)


subplot(2,2,3)
grid on
axis([0 15,-2.2 2.2])
plot(sim_t(1:length(sim_t)),d_qp(1,:),'b')
title('qp dstb 1','interpreter','latex','FontSize',20)

subplot(2,2,4)
grid on
axis([0 15,-2.2 2.2])
plot(sim_t(1:length(sim_t)),d_qp(2,:),'b')
title('qp ditb 2','interpreter','latex','FontSize',20)


%%
function dydt = sys(t,s,u,d)
dydt = [s(2)+d(1);(-s(2)+sin(s(1)))+u+d(2)];
end
