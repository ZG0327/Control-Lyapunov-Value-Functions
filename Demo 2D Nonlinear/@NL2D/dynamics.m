function dx = dynamics(obj, t, x, u, ~)
% Dynamics:
%    \dot{x}_1 = -sin(x_1)-0.5sin(x_1-x_2)
%    \dot{x}_2 = -0.5*sin(x_2)-0.5*sin(x_2-x_1) + u
%         u \in [uMin, uMax]
% if numel(u) ~= obj.nu
%   error('Incorrect number of control dimensions!')
% end


if iscell(x)
  dx = cell(obj.nx, 1);
  
  % Kinematic plane (speed can be changed instantly)
  dx{1} =  -sin(x{1})-0.5*sin(x{1}-x{2});
  dx{2} =  -0.5*sin(x{2})-0.5*sin(x{2}-x{1}) + u;
else
  dx = zeros(obj.nx, 1);
  
  % Kinematic plane (speed can be changed instantly)
  dx(1) =  -sin(x(1))-0.5*sin(x(1)-x(2));
  dx(2) = -0.5*sin(x(2))-0.5*sin(x(2)-x(1)) + u;
end


end