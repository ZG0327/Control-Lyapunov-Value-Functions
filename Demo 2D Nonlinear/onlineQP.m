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
num_traj = 3;




t = 0;
x0 = [-2.2;2.1];
x = nan(2,length(sim_t),num_traj);
u = nan(length(sim_t),num_traj);
V = nan(length(sim_t),num_traj);
Value = nan(201,201,num_traj);
u_delta = nan(2,length(sim_t),num_traj);
x(:,1,:) = repmat(x0,1,num_traj);
grad = nan(201,201,2,num_traj);
gamma = [0 , 0.5 , 1];

V_1 = importdata('V_05_0.mat');
V_2 = importdata('V_05_05.mat');
V_3 = importdata('V_05_1.mat');
g = importdata('g_05_0.mat');

Deriv_1 = computeGradients(g, V_1);
Deriv_2 = computeGradients(g, V_2);
Deriv_3 = computeGradients(g, V_3);

Value(:,:,1) = V_1;
Value(:,:,2) = V_2;
Value(:,:,3) = V_3;
grad(:,:,1,1) = Deriv_1{1};
grad(:,:,2,1) = Deriv_1{2};
grad(:,:,1,2) = Deriv_2{1};
grad(:,:,2,2) = Deriv_2{2};
grad(:,:,1,3) = Deriv_3{1};
grad(:,:,2,3) = Deriv_3{2};

H_delta = [ 0 , 0 ; 0 , 1];
f_delta = [0,0];
lb_delta = [-0.5;0];
ub_delta = [0.5;inf];

H = 1;
f = 0;
lb = -0.5;
ub = 0.5;

options = optimoptions('quadprog', 'ConstraintTolerance', 1e-6, 'StepTolerance', 1e-12, 'Display','iter');


%% Setup without slack variable
% for i = 1 : length(sim_t) 
%     V(i) = eval_u(g,data1,x(:,i));
%     deriv1 = eval_u(g,grad1,x(:,i));
%     deriv2 = eval_u(g,grad2,x(:,i));
%     LgV = deriv2;
%     LfV =  (-sin(x(1,i))-0.5*sin(x(1,i)-x(2,i)))*deriv1 + ...
%         (-0.5*sin(x(2,i))-0.5*sin(x(2,i)-x(1,i))) *deriv2;
% 
%     A_delta = [ LgV , -1 ; 0 , -1 ];
%     b_delta = [ -LfV-gamma*V(i) ; 0];
%     [u_delta(:,i),~,flag] = quadprog(H_delta,f_delta,A_delta,b_delta,[],[],lb_delta,ub_delta);
% 
%     A = LgV;
%     b = -LfV - gamma*V(i)+u_delta(2,i); % The 0.01 here accounts for the 
%                                               % convergence threshold of
%                                               % CLVF
%     [u(i),~,flag] = quadprog(H,f,A,b,[],[],lb,ub);
% 
%     
%     [ts_temp1, xs_temp1] = ode45(@(t,y) NL2D(t,y,u(i)), [t t+dt], x(:,i));
%     x(:,i+1) = xs_temp1(end,:);
%     t = t+dt;
% end

for j = 1:3
    for i = 1 : length(sim_t)
        V(i,j) = eval_u(g,Value(:,:,j),x(:,i,j));
        deriv1 = eval_u(g,grad(:,:,1,j),x(:,i,j));
        deriv2 = eval_u(g,grad(:,:,2,j),x(:,i,j));
        LgV = deriv2;
        LfV = (-sin(x(1,i,j))-0.5*sin(x(1,i,j)-x(2,i,j)))*deriv1 + ...
        (-0.5*sin(x(2,i,j))-0.5*sin(x(2,i,j)-x(1,i,j))) *deriv2;

        A_delta = [ LgV , -1 ; 0 , -1 ];
        b_delta = [ -LfV-gamma(j)*V(i,j) ; 0];
        [u_delta(:,i,j),~,flag] = quadprog(H_delta,f_delta,A_delta,b_delta,[],[],lb_delta,ub_delta);

        A = LgV;
        b = -LfV - gamma(j)*V(i,j)+u_delta(2,i,j); 
        [u(i,j),~,flag] = quadprog(H,f,A,b,[],[],lb,ub);


        [ts_temp, xs_temp] = ode45(@(t,y) NL2D(t,y,u(i,j)), [t t+dt], x(:,i,j));
        x(:,i+1,j) = xs_temp(end,:);
        t = t+dt;
    end
end

save('traj.mat','x');
save('traj_V.mat','V')
%% Figures and Videos
MakeVideo = 0;

% Traj on value function
% figure
% mesh(g.xs{1,1},g.xs{2,1},data1)
% view(30,75)
% set(gcf,'unit','normalized','position',[0.1,0.1,0.8,0.8]);
% hold on
% h_trajV = animatedline('Marker','o');
% 
% 
% xlabel('x1')
% ylabel('x2')
% zlabel('value')
% title('value function')
% if MakeVideo==1
%     v = VideoWriter('infea+QP','MPEG-4');
%     v.FrameRate = 30;
%     open(v);
% end
% 
% for i = 1: length(V) - 1
% 
%     addpoints(h_trajV,x(1,i),x(2,i),V(i));
% 
%     drawnow
%     if MakeVideo == 1
%         frame = getframe(gcf);
%         writeVideo(v,frame);
%     end
% end 
% if MakeVideo == 1
%     close(v);
% end

%% Plot the value function and slack 
% figure
% subplot(3,1,1)
% plot(sim_t(1:length(V)),V)
% grid on
% 
% xlabel('t')
% ylabel('Value')
% legend('Initial Condition 1')
% title('Change of Value Function')
% 
% 
% subplot(3,1,2)
% plot(sim_t(1:length(u)),u)
% grid on
% xlabel('t')
% ylabel('control input')
% legend('Initial Condition 1')
% title('Change of control')
% 
% subplot(3,1,3)
% plot(sim_t(1:length(u_delta)),u_delta(2,:))
% grid on
% xlabel('t')
% ylabel('slack variable')
% legend('Initial Condition 1')
% title('slack variable')

%%
function dydt = NL2D(t,y,u)
dydt = [-sin(y(1))-0.5*sin(y(1)-y(2)); ...
       -0.5*sin(y(2))-0.5*sin(y(2)-y(1)) + u];
end