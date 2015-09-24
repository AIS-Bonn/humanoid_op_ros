% undistort.m - Philipp Allgeuer - 03/07/13
% Perform camera undistortion on a camera pixel coordinate
%
% pixvec: Pixel coordinates to undistort (N-by-2 matrix for N coordinates)
% p:      Camera parameters (see cameraparams.m)
% tol:    The Euclidean distance tolerance to which to calculate the world vector (in the plane z = 1)
% camvec: Camera frame vectors corresponding to the given pixel coordinates
% numits: The number of iterations that was required to solve the distortion equations

% Main function
function [camvec, numits] = undistort(pixvec, p, tol)

	% Get missing parameters
	if nargin < 1
		error('No input pixel coordinate supplied to the undistort function!');
	end
	if nargin < 2
		p = cameraparams;
	end
	if nargin < 3
		tol = 0.2; % Expressed in units of pixels
	end
	if size(pixvec,2) ~= 2
		error('Every row of pixvec needs to be a 2D pixel coordinate!');
	end
	
	% Allocate memory for output
	N = size(pixvec,1);
	camvec = zeros(N,3);
	numits = zeros(N,1);
	
	% Undistort each pixel coordinate in turn
	for k = 1:N
		if (pixvec(k,1) < 0) || (pixvec(k,1) >= p.rx)
			warning(['Row ' num2str(k) ' has an x pixel coordinate outside the image - ignoring!']);
			continue;
		end
		if (pixvec(k,2) < 0) || (pixvec(k,2) >= p.ry)
			warning(['Row ' num2str(k) ' has a y pixel coordinate outside the image - ignoring!']);
			continue;
		end
		xin = (pixvec(k,1)-p.cx)/p.fx;
		yin = (pixvec(k,2)-p.cy)/p.fy;
		[tmp1,tmp2] = numsolve(xin,yin,p,tol);
		camvec(k,:) = tmp1;
		numits(k,1) = tmp2;
	end

end

% Numerical solution to the undistortion equations
function [camvec, k] = numsolve(xin, yin, p, tol) % Stopping condition is ||fx*F(a,b)|| < tol

	% Starting guess
	k = 0;
	aOld = 0;
	bOld = 0;
	
	% Evaluate Fx, Fy and f of initial guess
	[FxOld,FyOld,fOld] = F(aOld,bOld,xin,yin,p);
	
	% Check stopping tolerance
	ftol = 0.5*tol*tol/(p.fx*p.fx);
	if fOld < ftol
		camvec = [aOld bOld 1];
		return;
	end
	
	% Apply Newton-Raphson method
	for k = 1:20 % Maximum of 20 steps!
		
		% Evaluate Jacobian
		Jmat = J(aOld,bOld,p);
		
		% Calculate required adjustment
		Jdet = Jmat(1,1)*Jmat(2,2) - Jmat(1,2)*Jmat(2,1);
		if abs(Jdet) < 1e-12
			warning(['Encountered singular Jacobian at [' num2str([aOld bOld 1]) ']!']);
			camvec = [aOld bOld 1];
			return;
		end
		da = (-1/Jdet)*(Jmat(2,2)*FxOld - Jmat(1,2)*FyOld);
		db = (-1/Jdet)*(Jmat(1,1)*FyOld - Jmat(2,1)*FxOld);
		
		% Always take a full step for now
		lambda = 1.0;
		
		% Calculate new guess
		aNew = aOld + lambda*da;
		bNew = bOld + lambda*db;
		
		% Evaluate how good the new guess is
		[FxNew,FyNew,fNew] = F(aNew,bNew,xin,yin,p);
		
		% Check stopping tolerance
		if fNew < ftol
			camvec = [aNew bNew 1];
			return;
		end
		
		% Update variables for next iteration
		aOld = aNew;
		bOld = bNew;
		FxOld = FxNew;
		FyOld = FyNew;
		
	end
	
	% Failed (maximum iterations reached)
	warning(['Maximum number of iterations reached although f(x) is still ' num2str(fOld) '!']);
	camvec = [aOld bOld 1];

end

% Evaluate distortion equation
function [Fx,Fy,f] = F(a, b, xin, yin, p)

	% Evaluate parameters
	r2 = a*a + b*b;
	pn = 1.0 + r2*(p.k1 + r2*(p.k2 + r2*p.k3));
	pd = 1.0 + r2*(p.k4 + r2*(p.k5 + r2*p.k6));
	kr = pn/pd;
	kab = 2.0*a*b;
	
	% Evaluate distorted point deviations
	Fx = a*kr + p.p1*kab + p.p2*(r2 + 2.0*a*a) - xin;
	Fy = b*kr + p.p2*kab + p.p1*(r2 + 2.0*b*b) - yin;
	
	% Evaluate potential value function f
	f = 0.5*(Fx*Fx + Fy*Fy);

end

% Evaluate Jacobian matrix
function Jmat = J(a, b, p)

	% Evaluate parameters
	r2 = a*a + b*b;
	c1 = 2.0*(p.k1 + 2.0*p.k2*r2 + 3.0*p.k3*r2*r2);
	c2 = 2.0*(p.k4 + 2.0*p.k5*r2 + 3.0*p.k6*r2*r2);
	pn = 1.0 + r2*(p.k1 + r2*(p.k2 + r2*p.k3));
	pd = 1.0 + r2*(p.k4 + r2*(p.k5 + r2*p.k6));
	kr = pn/pd;
	c3 = (c1*pd - c2*pn)/(pd*pd);
	
	% Evaluate matrix entries
	dFxda = kr + a*a*c3 + 2.0*p.p1*b + 6.0*p.p2*a;
	dFydb = kr + b*b*c3 + 2.0*p.p2*a + 6.0*p.p1*b;
	dFxdb = a*b*c3 + 2.0*(p.p1*a + p.p2*b); % Note: dFyda == dFxdb
	
	% Construct Jacobian matrix
	Jmat = [dFxda dFxdb;dFxdb dFydb];

end
% EOF