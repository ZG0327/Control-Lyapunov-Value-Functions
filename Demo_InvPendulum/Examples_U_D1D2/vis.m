close all; clear; clc

% This is a toy example of CLVF-QP 
% \dot x1 = x2 + d1
% \dot x2 = -a*x2 + b*sin(x1) + u + d2, 
% u \in [-2,2], d1 = 0, d2 \in [-0.5,0.5]. 
% Note this is equavalent to u \in [-1.5,1.5] and no d.
% where a=b=1. 

% V = importdata('value_gamma=0.mat');
% V_min = min(V,[],'all');
% V_g = importdata('value_0.1_min.mat');
V_g = importdata('value_g=01_601.mat');
g = importdata('grid_large.mat');
% V = V-V_min;
% V = V-0.15;
params = importdata('params.mat');

% Deriv = computeGradients(g, V);
% grad1 = Deriv{1};
% grad2 = Deriv{2};

% Deriv_g = computeGradients(g, V_g);
[Deriv_g1 , Deriv_g2, Deriv_g] = computeGradients(g, V_g);

grad1_g = Deriv_g{1};
grad2_g = Deriv_g{2};

D_g1 = computeGradients(g, grad1_g);
D_g2 = computeGradients(g, grad2_g);

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

x1 = g.xs{1};
x2 = g.xs{2};

x0 = [0,0];
dyn = InvertedPendulum(x0, params);

% [X,Y] = meshgrid(-1:0.1:1,-1:0.1:1);
dOpt = optDstb(dyn, [], [], Deriv_g );
uOpt = optCtrl(dyn, [], [], Deriv_g );
lth = 20;
x1s = x1(1:lth:end,1:lth:end);
x2s = x2(1:lth:end,1:lth:end);
d1 = dOpt{1};
d1s = d1(1:lth:end,1:lth:end);
d2 = dOpt{2};
d2s = d2(1:lth:end,1:lth:end);
us = uOpt(1:lth:end,1:lth:end);

f1 = x2s+d1s;
f2 = -x2s +sin(x1s)+us+d2s;

l = size(x1s,1);
uqp = nan(l,l);
gamma = 0.1; 

H = 1;
H_delta = [ 0 , 0 ; 0 , 1];
f_delta = [0,0];
lb_delta = [params.u_min;0];
ub_delta = [params.u_max;inf];
lb = params.u_min;
ub = params.u_max;
for i = 1 : l
    for j = 1 : l
        V = eval_u(g,V_g,[x1s(i,j),x2s(i,j)]);
        deriv1_qp = eval_u(g,grad1_g,[x1s(i,j),x2s(i,j)]);
        deriv2_qp = eval_u(g,grad2_g,[x1s(i,j),x2s(i,j)]);
        LgV_qp = deriv2_qp;
        LfV_qp = x2s(i,j)*deriv1_qp + ...
        deriv2_qp * (-x2s(i,j)+sin(x1s(i,j)));
        max_D = d1s(i,j)*deriv1_qp+d2s(i,j)*deriv2_qp;
        A_delta = [ LgV_qp , -1 ; 0 , -1 ];
        b_delta = [ -LfV_qp-gamma*V-max_D ; 0];
        [u_delta(i,j,:),~,] = quadprog(H_delta,f_delta,A_delta,b_delta,[],[],lb_delta,ub_delta);
        A1 = LgV_qp;
        b1 = -LfV_qp - gamma*V-max_D+u_delta(i,j,2);
        uqp(i,j) = quadprog(H,0,A1,b1,[],[],lb,ub);
    end
end

f2qp = f2-us+uqp;

%%
% figure
% plot(x1(I_1),x2(I_1),'.')
% hold on
% plot(x1(I_2),x2(I_2),'.')
% legend('dVdx1 nonD','dVdx2 nonD')
%% Value functions and gradients 
% 
% figure
% set(gcf,'unit','normalized','position',[0.1,0.1,0.8,0.8]);
% visSetIm(g,V_g,'r',0)
% hold on
% visFuncIm(g,V_g,'b',0.3)
% c = camlight;
% c.Position = [-30 -30 -30];
% grid off
% view(45,25)
% xlabel('$x_1$','interpreter','latex','FontSize',25);
% ylabel('$x_2$','interpreter','latex','FontSize',25);
% zlabel('$V$','interpreter','latex','FontSize',25);
% title('R-CLVF','interpreter','latex','FontSize',20)
% 
% figure
% set(gcf,'unit','normalized','position',[0.1,0.1,0.8,0.8]);
% visSetIm(g,grad1_g,'k',0)
% hold on
% visFuncIm(g,grad1_g,'r',0.5)
% hold on
% plot3(x1(I_1),x2(I_1),grad1_g(I_1),'.')
% c = camlight;
% c.Position = [-30 -30 -30];
% grid off
% view(45,25)
% xlabel('$x_1$','interpreter','latex','FontSize',25);
% ylabel('$x_2$','interpreter','latex','FontSize',25);
% zlabel('$\frac{dV}{dx_1}$','interpreter','latex','FontSize',25);
% title('Gradient 1','interpreter','latex','FontSize',20)
% 
% figure
% set(gcf,'unit','normalized','position',[0.1,0.1,0.8,0.8]);
% visSetIm(g,grad2_g,'k',0)
% hold on
% visFuncIm(g,grad2_g, 'g',0.5)
% hold on
% plot3(x1(I_2),x2(I_2),grad2_g(I_2),'.')
% c = camlight;
% c.Position = [-30 -30 -30];
% grid off
% view(45,25)
% xlabel('$x_1$','interpreter','latex','FontSize',25);
% ylabel('$x_2$','interpreter','latex','FontSize',25);
% zlabel('$\frac{dV}{dx_2}$','interpreter','latex','FontSize',25);
% title('Gradient 2','interpreter','latex','FontSize',20)

%% Flow field 
figure
set(gcf,'unit','normalized','position',[0.1,0.1,0.8,0.8]);
visSetIm(g,grad1_g,'r',0)
hold on
visSetIm(g,grad2_g,'g',0)
hold on
plot(x1(I_1),x2(I_1),'r.')
hold on
plot(x1(I_2),x2(I_2),'g.')
hold on
quiver(x1s,x2s,f1,f2);

xlabel('$x_1$','interpreter','latex','FontSize',25);
ylabel('$x_2$','interpreter','latex','FontSize',25);
title('Zero Gradients','interpreter','latex','FontSize',20)
legend('$\frac{dV}{dx_1}=0$','$\frac{dV}{dx_2}=0$',...
    'non diff wrt $x_1$','non diff wrt $x_2$','interpreter','latex','FontSize',25)

% figure
% set(gcf,'unit','normalized','position',[0.1,0.1,0.8,0.8]);
% visSetIm(g,V_g,'b',0)
% hold on
% visSetIm(g,V,'r',0.01)
% xlim([-1,1]);
% ylim([-1,1]);
% xlabel('$x_1$','interpreter','latex','FontSize',25);
% ylabel('$x_2$','interpreter','latex','FontSize',25);
% legend('$\gamma = 0.1$','$\gamma =0$','interpreter','latex','FontSize',20)
% title('Zero-level set','interpreter','latex','FontSize',20)
% 
