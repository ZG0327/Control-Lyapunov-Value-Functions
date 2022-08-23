close all; clear all; clc

% This is a toy example of CLF-QP
% \dot x1 = v*cos(u),
% \dot x2 = v*sin(u),
% \dot x3 = u,
% u \in [-pi pi];

% CLF : V(x) = (cos(x3)*(x2-x20) - sin(x3)*(x1-x10))^2
%


dt = 0.02;
sim_t = [0:dt:5];
delta = 1e-3;

px = 10;
py = 2;
v = 2;


x0 = [4 , 5 , 0];
x = zeros(3,length(sim_t));
x(1:3,1) = x0;
x1 = x(1,:);
x2 = x(2,:);
x3 = x(3,:);
u = zeros(1,length(sim_t));
H = 1;
lb = -pi;
ub = pi;
gamma = 0.7;




% options = optimoptions('quadprog', 'ConstraintTolerance', 1e-6, 'StepTolerance', 1e-12, 'Display','iter');


t = 0;
%% CLF QP 
for i = 1 : length(sim_t) - 1
    t;
    x1(i) = x(1,i);
    x2(i) = x(2,i);
    x3(i) = x(3,i);

    %(cos(theta).*(p_y-params.yd)-sin(theta).*(p_x-params.xd)).^2
    V(i) =  (cos(u(i))*(x2(i)-py) - sin(u(i))*(x1(i)-px))^2;
    A = 2*(cos(u(i))*(x2(i)-py) -sin(u(i))*(x1(i)-px)) ...
        * (-sin(u(i))*(x2(i)-py) -cos(u(i))*(x1(i)-px));
    b = - gamma* V(i);    
    u(i) = quadprog(H,[],A,b,[],[],lb,ub,[]);
%     u(:,i) = quadprog(H,[],A,b);
    [ts_temp, xs_temp] = ode45(@(t, s) Dcar(t, s, u(i)), [t t+dt], x(:,i));
    x(:,i+1) = xs_temp(end, :);
    t = t+dt;
  
    

    if norm(x(1:2,i+1)-[px,py] ) <= delta
        break
    end
end

%%
figure(1)
subplot(3,1,1)
plot(sim_t(1:length(x1)),x1)
hold on 
plot(sim_t(1:length(x2)),x2)
grid on
xlabel('t')
ylabel('x')
title('solution')

subplot(3,1,2)
plot(sim_t(1:length(x1)),u)
grid on
xlabel('t')
ylabel('u')
legend
title('control signal')

subplot(3,1,3)
plot(sim_t(1:length(V)),V)
hold on
plot(sim_t,V(1)*exp(-gamma*sim_t))
grid on
xlabel('t')
ylabel('V(x)')
title('Value of CLF')

figure(2)
plot(x1(1:length(V)),x2(1:length(V)))
hold on 
plot(px,py,'o')
grid on
xlabel('x1')
ylabel('x2')
title('state tracjectory')


function dydt = Dcar(t,s,u)
    v = 2;
    dydt = [v*cos(u);v*sin(u);u];
end