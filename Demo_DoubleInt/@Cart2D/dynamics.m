function dx = dynamics(obj, t, x, u, d )
% Dynamics of the Plane
%    \dot{x}_1 = x2+d
%    \dot{x}_2 = u
%   Control: u 
%   Disturbance: d
%
% if numel(u) ~= obj.nu
%   error('Incorrect number of control dimensions!')
% end

if iscell(x)
  dx = cell(obj.nx, 1);
  
  % Kinematic plane (speed can be changed instantly)
  dx{1} = x{2}+d;
  dx{2} = u;
else
  dx = zeros(obj.nx, 1);
  
  % Kinematic plane (speed can be changed instantly)
  dx(1) = x(2)+d;
  dx(2) = u;
end


end