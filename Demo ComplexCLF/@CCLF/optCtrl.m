function uOpt = optCtrl(obj, t, xs, deriv, uMode, ~)
% uOpt = optCtrl(obj, t, deriv, uMode, dMode, MIEdims)

%% Input processing
if nargin < 5
  uMode = 'min';
end

if ~iscell(deriv)
  deriv = num2cell(deriv);
end

if strcmp(uMode, 'max')
  uOpt = (deriv{2}>=0)*obj.uMax + (deriv{2}<0)* obj.uMin;
elseif strcmp(uMode, 'min')
  uOpt = (deriv{2}>=0)* obj.uMin + (deriv{2}<0)*obj.uMax;
else
  error('Unknown uMode!')
end


end