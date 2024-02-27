function dx = dynamics(obj, t, x, u, d)

%       d0 = obj.d0;    
%       d1 = obj.d1;   
%       g = obj.g; 
%       n0 = obj.n0; 

if iscell(x)
   dx = cell(obj.nx, 1);

   dx{1} = x{2} + d;
   dx{2} = obj.g * tan(x{3});  
   dx{3} = -obj.d1 * x{3} + x{4};
   dx{4} = -obj.d0 * x{3} + obj.n0 * u;

else
   dx = zeros(obj.nx, 1);
   dx(1) = x(2) + d;
   dx(2) = obj.g * tan(x(3));  
   dx(3) = -obj.d1 * x(3) + x(4);
   dx(4) = -obj.d0 * x(3) + obj.n0 * u;
end


end
