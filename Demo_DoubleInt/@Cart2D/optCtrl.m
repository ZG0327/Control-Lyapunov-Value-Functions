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

%% Optimal control
% if iscell(deriv)
%   uOpt = cell(obj.nu, 1);
%   uOpt{1} = 0
%   if strcmp(uMode, 'max')
%     uOpt{2} = (deriv{2}>=0)*obj.uMax + (deriv{2}<0)* obj.uMin;
%     
%   elseif strcmp(uMode, 'min')
%     uOpt = (deriv{2}>=0)* obj.uMin + (deriv{2}<0)*obj.uMax;
%   else
%     error('Unknown uMode!')
%   end  
%   
% else
%   uOpt = zeros(obj.nu, 1);
%   uOpt(1) = 0
%   if strcmp(uMode, 'max')
%     uOpt(2) = (deriv(2)>=0)*obj.uMax + (deriv(2)<0)* obj.uMin;
%     
%   elseif strcmp(uMode, 'min')
%     uOpt(2) = (deriv(2)>=0)* obj.uMin + (deriv(2)<0)*obj.uMax;
%     
%   else
%     error('Unknown uMode!')
%   end
% end




end