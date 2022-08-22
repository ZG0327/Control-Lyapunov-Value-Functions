close all; 
clear all; 
clc

% This is a toy example of CLF-QP
% \dot x1 = x2 
% \dot x2 = u , u \in [-0.5,0.5]. ROA:
% CLF : V(x) = x1^2 + x2^2
% HJ value function:  V = data2(x)

% Problem setup
dt = 0.1;
sim_t = [0:dt:20];
delta = 1e-3;

load('data1.mat')
load('g.mat')
load('params.mat')
Deriv = computeGradients(g, data1);
grad1 = Deriv{1};
grad2 = Deriv{2};

t = 0;
x0 = [-3;4];
x = nan(2,length(sim_t));
u = nan(1,length(sim_t));
u_delta = nan(2,length(sim_t));
x(:,1) = x0;


H_delta = [ 0 , 0 ; 0 , 1];
f_delta = [0,0];
lb_delta = [params.uMin;0];
ub_delta = [params.uMax;inf];

H = 1;
f = 0;
lb = params.uMin;
ub = params.uMax;
mu = params.mu;
gamma = 0.2;
options = optimoptions('quadprog', 'ConstraintTolerance', 1e-6, 'StepTolerance', 1e-12, 'Display','iter');


%% Setup without slack variable
for i = 1 : length(sim_t) 
    V(i) = eval_u(g,data1,x(:,i));
    deriv1 = eval_u(g,grad1,x(:,i));
    deriv2 = eval_u(g,grad2,x(:,i));
    LgV = deriv2;
    LfV = x(2,i)*deriv1 + (mu*(1-x(1,i)^2)*x(2,i)-x(1,i))*deriv2;

    A_delta = [ LgV , -1 ; 0 , -1 ];
    b_delta = [ -LfV-gamma*V(i) ; 0];
    [u_delta(:,i),~,flag] = quadprog(H_delta,f_delta,A_delta,b_delta,[],[],lb_delta,ub_delta);

    A = LgV;
    b = -LfV - gamma*V(i)+u_delta(2,i); % The 0.01 here accounts for the 
                                              % convergence threshold of
                                              % CLVF
    [u(i),~,flag] = quadprog(H,f,A,b,[],[],lb,ub);

    
    [ts_temp1, xs_temp1] = ode45(@(t,y) vdp1(t,y,u(i),params), [t t+dt], x(:,i));
    x(:,i+1) = xs_temp1(end,:);
    t = t+dt;
end


%% Figures and Videos
MakeVideo = 0;

% Traj on value function
figure
mesh(g.xs{1,1},g.xs{2,1},data1)
view(30,75)
set(gcf,'unit','normalized','position',[0.1,0.1,0.8,0.8]);
hold on
h_trajV = animatedline('Marker','o');


xlabel('x1')
ylabel('x2')
zlabel('value')
title('value function')
if MakeVideo==1
    v = VideoWriter('infea+QP','MPEG-4');
    v.FrameRate = 30;
    open(v);
end

for i = 1: length(V) - 1

    addpoints(h_trajV,x(1,i),x(2,i),V(i));

    drawnow
    if MakeVideo == 1
        frame = getframe(gcf);
        writeVideo(v,frame);
    end
end 
if MakeVideo == 1
    close(v);
end

%% Plot the value function and slack 
figure
subplot(3,1,1)
plot(sim_t(1:length(V)),V)
grid on

xlabel('t')
ylabel('Value')
legend('Initial Condition 1')
title('Change of Value Function')


subplot(3,1,2)
plot(sim_t(1:length(u)),u)
grid on
xlabel('t')
ylabel('control input')
legend('Initial Condition 1')
title('Change of control')

subplot(3,1,3)
plot(sim_t(1:length(u_delta)),u_delta(2,:))
grid on
xlabel('t')
ylabel('slack variable')
legend('Initial Condition 1')
title('slack variable')

%%
function dydt = vdp1(t,y,u,params)
mu = params.mu;
dydt = [y(2); (1-y(1)^2)*y(2)-y(1) + u];
end