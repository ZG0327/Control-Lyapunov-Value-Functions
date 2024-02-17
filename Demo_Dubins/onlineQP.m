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
sim_t = [0:dt:15];
delta = 1e-3;
v = 1;


data1 = importdata('V_gamma=03_c=05.mat');
%% Grid
grid_min = [-3; -3; -pi]; % Lower corner of computation domain
grid_max = [3; 3; pi];    % Upper corner of computation domain
N = [101; 101; 51];         % Number of grid points per dimension
pdDims = 3;               % 3rd dimension is periodic

g = createGrid(grid_min, grid_max, N, pdDims);
Deriv = computeGradients(g, data1);
grad1 = Deriv{1};
grad2 = Deriv{2};
grad3 = Deriv{3};

t = 0;
x0 = [ 2; -2 ; 0 ];
x = nan(3,length(sim_t));
u = nan(1,length(sim_t));
u_delta = nan(2,length(sim_t));
x(:,1) = x0;


H_delta = [ 0 , 0 ; 0 , 1];
f_delta = [0,0];
lb_delta = [-pi;0];
ub_delta = [pi;inf];

H = 1;
f = 0;
lb = -pi;
ub = pi;
gamma = 0.3;
options = optimoptions('quadprog', 'ConstraintTolerance', 1e-6, 'StepTolerance', 1e-12, 'Display','iter');


%% Setup without slack variable
for i = 1 : length(sim_t) 
    V(i) = eval_u(g,data1,x(:,i));
%     if V(i) >= 0.05
        deriv1 = eval_u(g,grad1,x(:,i));
        deriv2 = eval_u(g,grad2,x(:,i));
        deriv3 = eval_u(g,grad3,x(:,i));
        LgV = deriv3;
        LfV = deriv1*v*cos(x(3,i)) + deriv2*v*sin(x(3,i));

        A_delta = [ LgV , -1 ; 0 , -1 ];
        b_delta = [ -LfV-gamma*V(i) ; 0];
        [u_delta(:,i),~,flag] = quadprog(H_delta,f_delta,A_delta,b_delta,[],[],lb_delta,ub_delta);

        A = LgV;
%         b = -LfV - gamma*V(i)+u_delta(2,i); % The 0.01 here accounts for the
        %                                               convergence threshold of
        %                                               CLVF
        [u(i),~,flag] = quadprog(H,f,A,b,[],[],lb,ub);
%     else 
%         u(i) = 0;
%     end
    [ts_temp1, xs_temp1] = ode45(@(t,y) Dcar(t,y,u(i)), [t t+dt], x(:,i));

    x(:,i+1) = xs_temp1(end,:);
    t = t+dt;

end


%% Figures and Videos
l = length(V);
figure
plot(x(1,:),x(2,:));
grid on
xlabel('x','interpreter','latex');
ylabel('y','interpreter','latex');
% 
% figure
% subplot(3,1,1)
% plot(sim_t(1:l),u(1:l))
% grid on
% xlabel('time')
% ylabel('control')
% 
% subplot(3,1,2)
% plot(sim_t(1:l),u_delta(2,1:l))
% grid on
% xlabel('time')
% ylabel('violation')
% 
% subplot(3,1,3)
% plot(sim_t(1:l),V(1:l))
% grid on
% xlabel('time')
% ylabel('Value')
% 
% figure
% plot(sim_t(1:l),V(1:l))
% grid on
% xlabel('t','interpreter','latex')
% ylabel('V','interpreter','latex')
% 
%%

%% Figures and Videos
MakeVideo = 0;
[g1D, data1D] = proj(g,data1,[0 0 1],'min');
% Traj on value function
figure
mesh(g1D.xs{1,1},g1D.xs{2,1},data1D)
view(30,75)
set(gcf,'unit','normalized','position',[0.1,0.1,0.8,0.8]);
hold on
h_trajV = animatedline('Marker','o');


xlabel('x1')
ylabel('x2')
zlabel('value')
title('value function')
if MakeVideo==1
    v = VideoWriter('DubinsQP','MPEG-4');
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



%%
function dydt = Dcar(t,s,u)
    v = 1;
    dydt = [v*cos(s(3));v*sin(s(3));u];
end