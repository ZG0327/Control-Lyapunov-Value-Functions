function dx = dynamics(obj, t, x, u, ~)
% Dynamics of the Plane
%    \dot{x}_1 = x2
%    \dot{x}_2 = mu*(1 - x_1^2)*x_2 -x_1 + u
%   Control: u 
%
% if numel(u) ~= obj.nu
%   error('Incorrect number of control dimensions!')
% end

mu = obj.mu;

if iscell(x)
  dx = cell(obj.nx, 1);
  
  % Kinematic plane (speed can be changed instantly)
  dx{1} = x{2};
  dx{2} = mu*(1 - x{1}.^2).*x{2} -x{1} + u;
else
  dx = zeros(obj.nx, 1);
  
  % Kinematic plane (speed can be changed instantly)
  dx(1) = x(2);
  dx(2) = mu*(1 - x(1).^2).*x(2) -x(1) + u;
end


end