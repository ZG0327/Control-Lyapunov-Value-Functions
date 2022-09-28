classdef NL2D < DynSys
  properties
    % Bound for speed in x
    uMin
    uMax
    % Damping parameter
    mu
  end
  
  methods
    function obj = NL2D(x, params)
      % obj = Example(x, params)
      %
      % Constructor. Creates a VDP object with a unique ID,
      % state x, and reachable set information reachInfo
      %
      % Dynamics:
      %    \dot{x}_1 = -sin(x_1)-0.5sin(x_1-x_2)
      %    \dot{x}_2 = -0.5*sin(x_2)-0.5*sin(x_2-x_1) + u
      %         u \in [uMin, uMax]
      
      %
      % Inputs:
      %   x     - state: [position; velocity]
      %   mu    - damping parameter
      %   uMin  - minimum control
      %   uMax  - maximum control
      
      % Output:
      %   obj       - a 2D example
      %
      
      if numel(x) ~= 2
        error('Initial state does not have right dimension!');
      end
      
      if ~iscolumn(x)
        x = x';
      end
      
      
      obj.x = x;
      obj.xhist = obj.x;
      
      obj.uMin = params.uMin;
      obj.uMax = params.uMax;
      
      obj.nx = 2;
      obj.nu = 1;
    end
    
  end % end methods
end % end classdef
