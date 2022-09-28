close all; 
clear all; 
clc

% This is a toy example of CLF-QP
% \dot x1 = x2 
% \dot x2 = (-b*x(2) + m*g*l*sin(x(1))) / (m*l^2) - u/(m*l^2) , u \in [-0.5,0.5]. ROA:
% CLF : V(x) = x1^2 + x2^2
% HJ value function:  V = data2(x)

% Problem setup
dt = 0.025;
sim_t = [0:dt:15];
delta = 1e-3;


x0 = [2;2];
x = nan(2,length(sim_t));
u = nan(1,length(sim_t));
x(:,1) = x0;
x1 = x;
u1 = u;
H = 1;

gamma = 0.5;

options = optimoptions('quadprog', 'ConstraintTolerance', 1e-6, 'StepTolerance', 1e-12, 'Display','iter');
t = 0;
load('value.mat')
load('grid.mat')
load('params.mat')
b = params.b;
G = params.g;
m = params.m;
l = params.l;
lb = params.u_min;
ub = params.u_max;

% data2 = data2 - min(data2,[],'all');
Deriv = computeGradients(g, data1);
grad1 = Deriv{1};
grad2 = Deriv{2};



%%
% using value function
for i = 1 : length(sim_t) 
    V1(i) = eval_u(g,data1,x1(:,i));
    deriv1 = eval_u(g,grad1,x1(:,i));
    deriv2 = eval_u(g,grad2,x1(:,i));
    LgV = deriv2/m/(l^2);
    LfV = x1(2,i)*deriv1 + deriv2 * (-b*x1(2,i)+m*G*l*sin(x1(1,i)))/m/(l^2);
    A1 = LgV;
    b1 = -LfV - gamma*V1(i)+0.02; % The 0.01 here accounts for the 
                                              % convergence threshold of
                                              % CLVF

    [u1(i),~,flag] = quadprog(H,0,A1,b1,[],[],lb,ub);

    [ts_temp1, xs_temp1] = ode45(@(t, s) sys(t, s, u1(i),params), [t t+dt], x1(:,i));
    x1(:,i+1) = xs_temp1(end,:);
    t = t+dt;
    norm_x(i) = norm(x1(:,i)-[0;0]);
%     if norm_x(i) <= 0.01
%         break
%     end
end



%% Figures and Videos
MakeVideo = 0;

% figure
% plot(sim_t,u1)

% Traj on value function
myfig = figure()
hold on
mesh(g.xs{1,1},g.xs{2,1},data1)
view(45,70)
set(gcf,'unit','normalized','position',[0.1,0.1,0.8,0.8]);
hold on
h_trajV = animatedline('Marker','o');
hold on
plot3(pi,0,eval_u(g,data1,[pi,0]),'r*')
hold on
plot3(-pi,0,eval_u(g,data1,[-pi,0]),'r*')
xlabel('x1')
ylabel('x2')
zlabel('value')
title('value function')
if MakeVideo==1
    v = VideoWriter('Final_debugged','MPEG-4');
    v.FrameRate = 30;
    open(v);

end

for i = 1:length(V1) - 1
           

    addpoints(h_trajV,x1(1,i),x1(2,i),V1(i));
    drawnow
    frame = getframe(gcf);
    if MakeVideo == 1
        writeVideo(v,frame);
    end
end 

if MakeVideo == 1
    close(v);
end


% Plot the value function and control
figure
subplot(2,1,1)
grid on
axis([0 15,-2.2 2.2])
xlabel('t')
ylabel('u')
plot(sim_t(1:length(sim_t)),u1)
hold on
title('control')

subplot(2,1,2)
axis([0 15 ,-0.1 3])
grid on
plot(sim_t(1:length(V1)),V1(1)*exp(-gamma*sim_t(1:length(V1))))
hold on
h_V = animatedline;
xlabel('t')
ylabel('Value')
legend('Exp reference','Decay of value function')
title('Decay of value function')


if MakeVideo==1
    v1 = VideoWriter('EspStb','MPEG-4');
    v1.FrameRate = 30;
    open(v1);
end
for i = 1:length(V1) - 1
    addpoints(h_V,sim_t(i),V1(i));
    drawnow
    if MakeVideo == 1
        frame = getframe(gcf);
        writeVideo(v1,frame);
    end
end



if MakeVideo == 1
    close(v1);
end


%%
function dydt = sys(t,s,u,params)
b = params.b;
G = params.g;
m = params.m;
l = params.l;
dydt = [s(2);(-b*s(2) + m*G*l*sin(s(1))) / (m*l^2) + u/(m*l^2)];
end