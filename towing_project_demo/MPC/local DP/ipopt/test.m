
clc; clear all;
  x0         = [ 2.5 0.5 2 -1 0.5 ];  % The starting point.
  options.cl = [ 4 0 0 ];             % Lower bounds on constraints.
  options.cu = [ 4 0 0 ];             % Upper bounds on constraints.

  % Set the IPOPT options.
  options.ipopt.jac_c_constant        = 'yes';
  options.ipopt.hessian_approximation = 'limited-memory';
  options.ipopt.mu_strategy           = 'adaptive';
  options.ipopt.tol                   = 1e-7;
  

  % The callback functions.
  funcs.objective         = @objective;
  funcs.constraints       = @constraints;
  funcs.gradient          = @gradient;
  funcs.jacobian          = @jacobian;
  funcs.jacobianstructure = @jacobian;
  
  % Run IPOPT.
  [x info] = ipopt(x0,funcs,options);
  
% ----------------------------------------------------------------------
function f = objective (x)
  f = (x(1) - x(2))^2 + ...
      (x(2) + x(3) - 2)^2 + ...
      (x(4) - 1)^2 + (x(5) - 1)^2;
end
% ----------------------------------------------------------------------
function g = gradient (x)
  g = 2*[ x(1) - x(2);
	  x(2) + x(3) - 2 - x(1) + x(2);
	  x(2) + x(3) - 2;
	  x(4) - 1;
	  x(5) - 1 ];
end
% ----------------------------------------------------------------------
function c = constraints (x)
  c = [ x(1) + 3*x(2);
        x(3) + x(4) - 2*x(5);
        x(2) - x(5) ];
end
% ----------------------------------------------------------------------
function J = jacobian (x)  
  J = sparse([ 1  3  0  0  0;
	       0  0  1  1 -2;
	       0  1  0  0 -1 ]);
end  