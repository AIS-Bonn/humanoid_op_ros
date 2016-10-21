% distort.m - Philipp Allgeuer - 02/07/13
% Perform camera distortion on a camera coordinate vector
%
% camvec: Vector in camera frame to distort (every row should be a 3-vector, missing z => assume to be 1)
% pixvec: Pixels corresponding to the given vectors, accounting for distortion
% p:      Camera parameters (see cameraparams.m)

% Main function
function [pixvec] = distort(camvec, p)

	% Get missing parameters
	if nargin < 1
		error('No input camera vector supplied to the distort function!');
	end
	if nargin < 2
		p = cameraparams;
	end
	if size(camvec,2) == 2
		camvec = [camvec ones(size(camvec,1),1)];
	elseif size(camvec,2) ~= 3
		error('Every row of camvec needs to be either a 2-vector or 3-vector!');
	end

	% Error checking
	if any(camvec(:,3) <= 0)
		error('This routine can''t deal with zero or negative z-values yet!');
	end

	% Normalise the z-component of the input vector [resulting vector is (a,b,1)]
	a = camvec(:,1)./camvec(:,3);
	b = camvec(:,2)./camvec(:,3);
	
	% Saturate a and b if using linear extension
	if p.uselinext
		olda = a;
		oldb = b;
		a = max(p.ano,min(p.apo,a));
		b = max(p.bno,min(p.bpo,b));
	end
	
	% Calculate additional parameters required for distortion equations
	r2 = a.*a + b.*b;
	kr = (1.0 + r2.*(p.k1 + r2.*(p.k2 + r2*p.k3)))./(1.0 + r2.*(p.k4 + r2.*(p.k5 + r2*p.k6)));
	kab = 2.0*a.*b;
	
	% Adjust for tangential and radial distortions
	x = kr.*a + kab*p.p1 + (r2+2.0*a.*a)*p.p2;
	y = kr.*b + kab*p.p2 + (r2+2.0*b.*b)*p.p1;
	
	% Adjust for focal lengths and offsets
	pixvec = [p.fx*x+p.cx p.fy*y+p.cy];
	
	% Extend the distortion function linearly if desired
	if p.uselinext
	
		% Calculate lambda values (0 => Use raw value / 1 => Use linear extension)
		alambda = (olda >= 0.0).*max(0.0, min(1.0, (olda-p.api)/(p.apo-p.api))) + (olda < 0.0).*max(0.0, min(1.0, (olda-p.ani)/(p.ano-p.ani)));
		blambda = (oldb >= 0.0).*max(0.0, min(1.0, (oldb-p.bpi)/(p.bpo-p.bpi))) + (oldb < 0.0).*max(0.0, min(1.0, (oldb-p.bni)/(p.bno-p.bni)));
		
		% Calculate linear extension in x direction
		pixvec(:,1) = (1.0-alambda).*pixvec(:,1) + alambda.*(p.mx*olda+p.bx);
		pixvec(:,2) = (1.0-blambda).*pixvec(:,2) + blambda.*(p.my*oldb+p.by);

	end

end
% EOF