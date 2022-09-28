function dx = dynamics(obj, t, x, u, ~)
% Dynamics:
%    \dot{x}_1 =  -3/2*x{1}.^2-1/2*x{1}.^3-x{2}
%    \dot{x}_2 = u
%         u \in [uMin, uMax]
% if numel(u) ~= obj.nu
%   error('Incorrect number of control dimensions!')
% end


if iscell(x)
  dx = cell(obj.nx, 1);
  
  % Kinematic plane (speed can be changed instantly)
  dx{1} =  -3/2*x{1}.^2-1/2*x{1}.^3-x{2};
  dx{2} = u;
else
  dx = zeros(obj.nx, 1);
  
  % Kinematic plane (speed can be changed instantly)
  dx(1) = -3/2*x(1).^2-1/2*x(1).^3-x(2);
  dx(2) = u;
end


end