function dOpt = optDstb(obj, t, xs, deriv, dMode, ~)
% uOpt = optCtrl(obj, t, deriv, uMode, dMode, MIEdims)

    l = obj.l;    % [m]        length of pendulum
    m = obj.m;    % [kg]       mass of pendulum

    g1 = 0;
    g2 = 1 / (m*l^2);
    
%% Input processing
if nargin < 5
  dMode = 'max';
end

%% Optimal control
if iscell(deriv)
%  multiplier = deriv{1}.*g1 + deriv{2}.*g2;
  dOpt = cell(obj.nd, 1);
  if strcmp(dMode, 'max')
    dOpt{1} = (deriv{1}>=0)*obj.dMax(1) + (deriv{1}<0)* obj.dMin(1);
    dOpt{2} = (deriv{2}>=0)*obj.dMax(2) + (deriv{2}<0)* obj.dMin(2);

  elseif strcmp(dMode, 'min')
    dOpt{1} = (deriv{1}<=0)*obj.dMax(1) + (deriv{1}>0)* obj.dMin(1);
    dOpt{2} = (deriv{2}<=0)*obj.dMax(2) + (deriv{2}>0)* obj.dMin(2);

    error('Unknown dMode!')
  end  
%   dOpt = cell2mat(dOpt);

else
  uOpt = zeros(obj.nd, 1);
%   multiplier = deriv(1).*g1 + deriv(2).*g2;
  if strcmp(dMode, 'max')
    dOpt(1) = (deriv(1)>=0)*obj.dMax(1) + (deriv(1)<0)* obj.dMin(1);
    dOpt(2) = (deriv(2)>=0)*obj.dMax(2) + (deriv(2)<0)* obj.dMin(2);
    
  elseif strcmp(dMode, 'min')
    dOpt(1) = (deriv(1)<=0)*obj.dMax(1) + (deriv(1)>0)* obj.dMin(1);
    dOpt(2) = (deriv(2)<=0)*obj.dMax(2) + (deriv(2)>0)* obj.dMin(2);    
  else
    error('Unknown dMode!')
  end
end



end