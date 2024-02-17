% This file shows the robust control invaraince of the RSCIS
close all; 
clear; 
clc

% Problem setup
dt = 0.01;
sim_t = [0:dt:3];
delta = 1e-3;


% data1 = importdata('V0_2norm.mat');
% data1 = importdata('V0_infnorm.mat');
data1 = importdata('V0_qc.mat');

data_min = min(data1,[],'all');
data1 = data1 - data_min;

g = importdata('g.mat');
Deriv = computeGradients(g, data1);
grad1 = Deriv{1};
grad2 = Deriv{2};


uRange = [-1,1]; % Velocity of the Dubins car
dRange = [-0.5,0.5]; % Desired postion
dCar = Cart2D([0, 0], uRange, dRange);
gamma = 0;


t = 0;
j = 0;
% while j ~= 1
%     x00 = randi([-1,1],3,1);
%     V00 = eval_u(g,data1,x00);
%     if V00 <= 0.09
%         x0 = x00;
%         j = 1;
%     end
% end
    
x0 = [  0 , -0.05   ];
x = nan(2,length(sim_t));
u = nan(1,length(sim_t));
u_delta = nan(2,length(sim_t));
x(:,1) = x0;



%% HJ opt control
for i = 1 : length(sim_t)
    V(i) = eval_u(g,data1,x(:,i));
    deriv = eval_u(g,Deriv,x(:,i));
    if deriv(2)>0
        u(i) = -1;
    else
        u(i) = 1;
    end
    if deriv(1)>=0
        d(i) = 0.5;
    else
        d(i) = -0.5;
    end
%     u(i) = dCar.optCtrl(dCar,[],[] ,deriv, 'min');
    [ts_temp, xs_temp] = ode113(@(t, s) dCar.dynamics(t, s, u(i),d(i)), [t t+dt], x(:,i));
    x(:,i+1) = xs_temp(end,:);
    t = t+dt;
end


%% QP control: setup with slack variable
% H_delta = [ 0 , 0 ; 0 , 1];
% f_delta = [0,0];
% lb_delta = [-1;0];
% ub_delta = [1;inf];
% 
% H = 1;
% f = 0;
% lb = -1;
% ub = 1;
% gamma = 0;
% options = optimoptions('quadprog', 'ConstraintTolerance', 1e-6, 'StepTolerance', 1e-12, 'Display','iter');
% 
% for i = 1 : length(sim_t)
%     V(i) = eval_u(g,data1,x(:,i));
%     deriv1 = eval_u(g,grad1,x(:,i));
%     deriv2 = eval_u(g,grad2,x(:,i));
%     if deriv1>=0
%         d(i) = 0.5;
%     else
%         d(i) = -0.5;
%     end
%     LgV = deriv2;
%     LfV = deriv1*(x(2,i)+d(i));
% 
%     A_delta = [ LgV , -1 ; 0 , -1 ];
%     b_delta = [ -LfV-gamma*V(i) ; 0];
%     [u_delta(:,i),~,~] = quadprog(H_delta,f_delta,A_delta,b_delta,[],[],lb_delta,ub_delta);
% 
%     A = LgV;
%     b = -LfV - gamma*V(i)+u_delta(2,i); % The 0.01 here accounts for the
%     %                                               convergence threshold of
%     %                                               CLVF
%     [u(i),~,~] = quadprog(H,f,A,b,[],[],lb,ub);
%     %
%     [ts_temp1, xs_temp1] = ode45(@(t,y) dCar.dynamics(t,y,u(i),d(i)), [t t+dt], x(:,i));
% 
%     x(:,i+1) = xs_temp1(end,:);
%     
%     t = t+dt;
% 
% end
% 
% 
%% Figures 
l = length(V);
figure
plot(x(1,:),x(2,:));
hold on
visSetIm(g,data1,'c',0.01)

xlabel('x','interpreter','latex');
ylabel('y','interpreter','latex');
zlabel('$\theta$','interpreter','latex');

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
figure
set(gcf,'unit','normalized','position',[0.1,0.2,0.84,0.4]);

subplot(1,3,1)
plot(sim_t(1:l),V(1:l))
grid on
xlabel('t','interpreter','latex')
ylabel('V','interpreter','latex')

subplot(1,3,2)
plot(sim_t(1:l),u(1:l))
grid on
xlabel('t','interpreter','latex')
ylabel('u','interpreter','latex')

subplot(1,3,3)
plot(sim_t(1:l),d(1:l))
grid on
xlabel('t','interpreter','latex')
ylabel('d','interpreter','latex')
% 
%%

%% Videos
% MakeVideo = 0;
% [g1D, data1D] = proj(g,data1,[0 0 1],'min');
% % Traj on value function
% figure
% % visSetIm(g,data1,'c',0.01)
% visFuncIm(g1D,data1D,'c',0.6)
% view(40,25)
% set(gcf,'unit','normalized','position',[0.1,0.1,0.8,0.8]);
% hold on
% h_trajV = animatedline('Marker','o');
% 
% 
% xlabel('x1')
% ylabel('x2')
% zlabel('\theta')
% title('value function')
% if MakeVideo==1
%     v = VideoWriter('DubinsQP','MPEG-4');
%     v.FrameRate = 30;
%     open(v);
% end
% 
% for i = 1: length(V) - 1
%     addpoints(h_trajV,x(1,i),x(2,i),V(i));
%     drawnow
%     if MakeVideo == 1
%         frame = getframe(gcf);
%         writeVideo(v,frame);
%     end
% end 
% if MakeVideo == 1
%     close(v);
% end
% 
% 

