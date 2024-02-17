clear; close all; clc

V = importdata('value.mat');
g = importdata('grid.mat');
params = importdata('params.mat');

Deriv = computeGradients(g, V);
grad1 = Deriv{1};
grad2 = Deriv{2};

%opt controller
u_opt = (grad2>=0).*params.u_min +(grad2<0).*params.u_max;

% QP controller
H = 1;
lb = params.u_min;
ub = params.u_max;
f = 0;
LfV = grad1.*g.xs{2}+grad2.*(-g.xs{2}+ sin(g.xs{1}));
LgV = grad2;

A = LgV;
b = -LfV-0.3*V;

for i = 1:151
    for j = 1: 151
        [u_qp(i,j),~,flag] = quadprog(H,f,A(i,j),b(i,j),[],[],lb,ub);
    end
end


%%
figure
visFuncIm(g,u_opt,'blue',0.3)