close all; 
clear all; 
clc


% Problem setup
dt = 0.01;
sim_t = [0:dt:20];
delta = 1e-3;

load('data1.mat')
load('g.mat')
load('params.mat')
Deriv = computeGradients(g, data1);
grad1 = Deriv{1};
grad2 = Deriv{2};

t = 0;
x0 = [ 2 ; 2.5];
x = nan(2,length(sim_t));
u = nan(1,length(sim_t));
u_delta = nan(2,length(sim_t));
x(:,1) = x0;
x_CLF = x;
x_BS = x;
u_CLF = u;
u_BS = u;
V_CLF = nan(1,length(sim_t));

H_delta = [ 0 , 0 ; 0 , 1];
f_delta = [0,0];
lb_delta = [params.uMin;0];
ub_delta = [params.uMax;inf];

H = 1;
f = 0;
lb = params.uMin;
ub = params.uMax;
gamma = 0.1;
options = optimoptions('quadprog', 'ConstraintTolerance', 1e-6, 'StepTolerance', 1e-12, 'Display','iter');

%% Backsteping feedback control and CLF
X = g.xs{1};
Y = g.xs{2};
CLF = 1/2*X.^2+1/2*(3/2*X.^2+Y).^2;

Deriv2 = computeGradients(g, CLF);
grad21 = Deriv2{1};
grad22 = Deriv2{2};

for i = 1 : length(sim_t) 
    xclf = x_CLF(:,i);
    xBS = x_BS(:,i);
    u_BS(i) = -3*xBS(1)*((-3/2)*xBS(1)^2-1/2*xBS(1)^3-xBS(2))+xBS(1)-xBS(2)-3/2*xBS(1)^2;
    [ts_BS, xs_BS] = ode45(@(t,y) CCLF(t,y,u_BS(i)), [t t+dt], x_BS(:,i));
    x_BS(:,i+1) = xs_BS(end,:);


    V_CLF(i) = eval_u(g,CLF,xclf);  
    deriv21 = eval_u(g,grad21,xclf);
    deriv22 = eval_u(g,grad22,xclf);
    LgV2 = deriv22;
    LfV2 = deriv21*(-3/2*xclf(1)^2-1/2*xclf(1)^3-xclf(2)) ;
    A_delta2 = [ LgV2 , -1 ; 0 , -1 ];
    b_delta2 = [ -LfV2-gamma*V_CLF(i) ; 0];
    [u_delta2(:,i),~,flag] = quadprog(H_delta,f_delta,A_delta2,b_delta2,[],[],lb_delta,ub_delta);
    A2 = LgV2;
    b2 = -LfV2 - gamma*V_CLF(i)+u_delta2(2,i); % The 0.01 here accounts for the 
                                              % convergence threshold of
                                              % CLVF
    [u_CLF(i),~,flag] = quadprog(H,f,A2,b2,[],[],lb,ub);


    [ts_temp, xs_temp] = ode45(@(t,y) CCLF(t,y,u_CLF(i)), [t t+dt], x_CLF(:,i));
    x_CLF(:,i+1) = xs_temp(end,:);
    t = t+dt;
end

%%
% figure
% visFuncIm(g, CLF,'blue',.3);
% view(45,30)
% set(gcf,'unit','normalized','position',[0.1,0.1,0.8,0.8]);
% hold on
% h_trajV_CLF = animatedline('Marker','o');
% xlabel('x1')
% ylabel('x2')
% zlabel('value')
% title('value function')
% % if MakeVideo==1
% %     v = VideoWriter('infea+QP','MPEG-4');
% %     v.FrameRate = 30;
% %     open(v);
% % end
% 
% for i = 1: length(V_CLF) - 1
% 
%     addpoints(h_trajV_CLF,x_CLF(1,i),x_CLF(2,i),V_CLF(i));
% 
%     drawnow
% %     if MakeVideo == 1
% %         frame = getframe(gcf);
% %         writeVideo(v,frame);
% %     end
% end 


%% Setup with slack variable
for i = 1 : length(sim_t) 
    V(i) = eval_u(g,data1,x(:,i));
    deriv1 = eval_u(g,grad1,x(:,i));
    deriv2 = eval_u(g,grad2,x(:,i));
    LgV = deriv2;
    LfV = deriv1*(-3/2*x(1,i)^2-1/2*x(1,i)^3-x(2,i)) ;

    A_delta = [ LgV , -1 ; 0 , -1 ];
    b_delta = [ -LfV-gamma*V(i) ; 0];
    [u_delta(:,i),~,flag] = quadprog(H_delta,f_delta,A_delta,b_delta,[],[],lb_delta,ub_delta);

    A = LgV;
    b = -LfV - gamma*V(i)+u_delta(2,i); % The 0.01 here accounts for the 
                                              % convergence threshold of
                                              % CLVF
    [u(i),~,flag] = quadprog(H,f,A,b,[],[],lb,ub);

    
    [ts_temp1, xs_temp1] = ode45(@(t,y) CCLF(t,y,u(i)), [t t+dt], x(:,i));
    x(:,i+1) = xs_temp1(end,:);
    t = t+dt;
end


% %% Figures and Videos
% MakeVideo = 0;
% 
% % Traj on value function
% figure
% visFuncIm(g, data1,'magenta',.3);
% view(45,30)
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
% 
% %% Plot the value function and slack 
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
% hold on
% plot(sim_t(1:length(u_delta2)),u_delta2(2,:))
% grid on
% xlabel('t')
% ylabel('slack variable')
% legend('Initial Condition 1')
% title('slack variable')
% 
%%
close all

figure
set(gcf,'unit','normalized','position',[0.2,0.4,0.6,0.4]);
subplot(1,2,1)
visFuncIm(g, data1,'magenta',.3);
view(30,15)
set(gca,'zTick',[0:1.5:4.5]);
xlabel('$x_1$','interpreter','latex','FontSize',14);
ylabel('$x_2$','interpreter','latex','FontSize',14);
zlabel('$V_\gamma^\infty(\gamma = 0.1)$','interpreter','latex','FontSize',16);
title('Value Function','FontSize',16)

subplot(1,2,2)
visFuncIm(g, CLF,'blue',.3); view(30,15)
set(gca,'zTick',0:50:150);
xlabel('$x_1$','interpreter','latex','FontSize',14);
ylabel('$x_2$','interpreter','latex','FontSize',14);
zlabel('$V$','interpreter','latex','FontSize',16);
title('Hand-crafted CLF','FontSize',16)


figure
set(gcf,'unit','normalized','position',[0.2,0.4,0.6,0.4])
subplot(1,2,1)
plot(x(1,:),x(2,:),'b.')
hold on
plot(x_CLF(1,:),x_CLF(2,:),'r.')
set(gca,'xTick',-2.5 : 1 : 2.5);
set(gca,'yTick',-2.5 : 1 : 2.5);
xlabel('$x_1$','interpreter','latex','FontSize',14);
ylabel('$x_2$','interpreter','latex','FontSize',14);
xlim([-2.5,2.5])
ylim([-2.5,2.5])
grid on
legend('Trajectory using CLVF','Trajectory using CLF','FontSize',14)

subplot(1,2,2)
plot(sim_t(1:length(u)),u)
hold on
plot(sim_t(1:length(u_CLF)),u_CLF)
grid on

xlabel('$t$','interpreter','latex','FontSize',14);
ylabel('$u$','interpreter','latex','FontSize',14);
legend('Control Signal using CLVF','Control Signal using CLF','FontSize',14)

% subplot(3,2,3)
% plot(x_CLF(1,:),x_CLF(2,:),'b.')
% xlabel('$x_1$','interpreter','latex','FontSize',14);
% ylabel('$x_2$','interpreter','latex','FontSize',14);
% xlim([-2.5,2.5])
% ylim([-2.5,2.5])
% grid on
% subplot(3,2,4)
% plot(sim_t(1:length(u_CLF)),u_CLF)
% grid on
% xlabel('$t$','interpreter','latex','FontSize',14);
% ylabel('$u$','interpreter','latex','FontSize',14);

% subplot(3,2,5)
% plot(x_BS(1,:),x_BS(2,:),'b.')
% xlabel('$x_1$','interpreter','latex','FontSize',14);
% ylabel('$x_2$','interpreter','latex','FontSize',14);
% xlim([-2.5,2.5])
% ylim([-2.5,2.5])
% grid on
% subplot(3,2,6)
% plot(sim_t(1:length(u_BS)),u_BS)
% grid on
% xlabel('$t$','interpreter','latex','FontSize',14);
% ylabel('$u$','interpreter','latex','FontSize',14);

%%
function dydt = CCLF(t,y,u)
dydt = [ -3/2*y(1).^2-1/2*y(1).^3-y(2); u];
end