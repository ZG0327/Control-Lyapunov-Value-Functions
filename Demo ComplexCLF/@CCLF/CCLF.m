classdef CCLF < DynSys
  properties
    % Bound for speed in x
    uMin
    uMax
    % Damping parameter
    mu
  end
  
  methods
    function obj = CCLF(x, params)
    
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
