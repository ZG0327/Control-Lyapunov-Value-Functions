classdef Cart2D < DynSys
  properties
    % Bound for speed in x
    uMin
    uMax
    dMin
    dMax
  end
  
  methods
      function obj = Cart2D(x, uRange, dRange)
      % obj = Example1D(x, uMin, uMax, dMin, dMax)
      %
      % Constructor. Creates a 1D object with a unique ID,
      % state x, and reachable set information reachInfo
      %
      % Dynamics:
      %    \dot{x}_1 = x_2 +d
      %    \dot{x}_2 = u
      %         u \in [uMin, uMax]
      %         d \in [dMin, dMax]
      %
      % Inputs:
      %   x     - state: [position; velocity]
      %   uMin  - minimum control
      %   uMax  - maximum control
      %   dMin  - minimum dstb
      %   dMax  - maximum dstb      

      % Output:
      %   obj       - a Cart2D object
      %
      
      if numel(x) ~= 2
        error('Initial state does not have right dimension!');
      end
      
      if ~iscolumn(x)
        x = x';
      end
      
      
      obj.x = x;
      obj.xhist = obj.x;
      
      obj.uMin = uRange(1);
      obj.uMax = uRange(2);
      obj.dMin = dRange(1);
      obj.dMax = dRange(2);     
      
      obj.pdim = 1;
      
      obj.nx = 2;
      obj.nu = 1;
      obj.nd = 1;
    end
    
  end % end methods
end % end classdef
