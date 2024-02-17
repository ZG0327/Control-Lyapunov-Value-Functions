function data = QuadCost(grid, Q)
% QuadCost: Quadratic cost function l(x) = x'Qx.
%
%   data = QuadCost(grid, ignoreDims, center, radius)
%
% parameters:
% Input Parameters:
%
%   grid: Grid structure (see processGrid.m for details).
%
%   Q: symatric matrix
%
% Output Parameters:
%
%   data: Output data array (of size grid.size) containing the implicit
%   surface function.

%---------------------------------------------------------------------------
% Default parameter values.
if(nargin < 2)
  error('no cost matrix specified');
end

if( size(Q,1) ~= grid.dim )
    error('wrong cost matrix dimension');
end
%---------------------------------------------------------------------------
% Quadratic cost function calculation.
data = zeros(grid.shape);
for i = 1 : grid.dim
    for j = 1:grid.dim
        data =  data + Q(i,j)*(grid.xs{i}.*grid.xs{j});
    end
end
% data = sqrt(data) - radius;

