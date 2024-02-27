function dOpt = optDstb(obj, t, xs, deriv, dMode, ~)
% uOpt = optCtrl(obj, t, deriv, uMode, dMode, MIEdims)

%% Input processing
if nargin < 5
  dMode = 'max';
end

%% Optimal control
if iscell(deriv)
  dOpt = cell(obj.nd, 1);
  if strcmp(dMode, 'max')
    dOpt = (deriv{1}>=0)*obj.dMax + (deriv{1}<0)* obj.dMin;
    
  elseif strcmp(dMode, 'min')
    dOpt = (deriv{1}>=0)* obj.dMin + (deriv{1}<0)*obj.dMax;
  else
    error('Unknown uMode!')
  end  
  
else
  dOpt = zeros(obj.nd, 1);
  if strcmp(dMode, 'max')
    dOpt = (deriv(1)>=0)*obj.dMax + (deriv(1)<0)* obj.dMin;
    
  elseif strcmp(dMode, 'min')
    dOpt = (deriv(1)>=0)* obj.dMin + (deriv(1)<0)*obj.dMax;
    
  else
    error('Unknown uMode!')
  end
end



end