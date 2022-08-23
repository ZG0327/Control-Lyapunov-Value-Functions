classdef VanDerPol < DynSys
  properties
    % Bound for speed in x
    uMin
    uMax
    % Damping parameter
    mu
  end
  
  methods
    function obj = VanDerPol(x, params)
      % obj = Example(x, params)
      %
      % Constructor. Creates a VDP object with a unique ID,
      % state x, and reachable set information reachInfo
      %
      % Dynamics:
      %    \dot{x}_1 = x_2
      %    \dot{x}_2 = mu*(1 - x_1^2)*x_2 -x_1 + u
      %         u \in [uMin, uMax]
      
      %
      % Inputs:
      %   x     - state: [position; velocity]
      %   mu    - damping parameter
      %   uMin  - minimum control
      %   uMax  - maximum control
      
      % Output:
      %   obj       - a Van Der Pol object
      %
      
      if numel(x) ~= 2
        error('Initial state does not have right dimension!');
      end
      
      if ~iscolumn(x)
        x = x';
      end
      
      
      obj.x = x;
      obj.xhist = obj.x;
      
      obj.mu = params.mu;
      obj.uMin = params.uMin;
      obj.uMax = params.uMax;
     
      
%       obj.pdim = 1:2;
      
      obj.nx = 2;
      obj.nu = 1;
    end
    
  end % end methods
end % end classdef
