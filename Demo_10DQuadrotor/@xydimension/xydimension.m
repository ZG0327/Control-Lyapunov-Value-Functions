classdef xydimension < DynSys
  properties
    uMin
    uMax
    dMin
    dMax
    d0
    d1
    n0
    g
  end
  
  methods
      function obj = xydimension(x, params)
      
      if numel(x) ~= 4
        error('Initial state does not have right dimension!');
      end
      
      if ~iscolumn(x)
        x = x';
      end
      
      obj.x = x;
      obj.xhist = obj.x;
      
      obj.uMin = params.u_min;
      obj.uMax = params.u_max;
      obj.dMin = params.d_min;
      obj.dMax = params.d_max;

%       obj.pdim = 1;
      obj.nx = 4;
      obj.nu = 1;
      obj.nd = 1;
      
      obj.d0 = params.d0;    
      obj.d1 = params.d1;   
      obj.g = params.g; 
      obj.n0 = params.n0; 
    end
    
  end % end methods
end % end classdef
