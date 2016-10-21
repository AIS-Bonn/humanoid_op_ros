% plotundistort.m - Philipp Allgeuer - 04/07/13
% Visualise the undistortion of the camera

% Main function
function plotundistort(p)

	% Retrieve camera parameters
	if nargin < 1
		p = cameraparams;
	end

	% Parameters
	pxmin = 0;
	pxmax = p.rx - 1;
	pymin = 0;
	pymax = p.ry - 1;
	Nx = 81;
	Ny = 61;

	% Construct the meshgrid to distort
	Xvals = linspace(pxmin,pxmax,Nx);
	Yvals = linspace(pymin,pymax,Ny);
	[X, Y] = meshgrid(Xvals,Yvals);
	Xc = X(:);
	Yc = Y(:);
	pixvec = [Xc Yc];
	
	% Undistort the required data points
	camvec = undistort(pixvec,p);
	Xp = reshape(camvec(:,1),size(X));
	Yp = reshape(camvec(:,2),size(Y));
	
	% Get plotting coordinates (and adjust for fact that y is positive downward)
	camX = Xp';
	camY = -Yp';
	
	% Plot the undistorted meshgrid
	figure(3);
	subplot(1,1,1);
	h = plot(camX,camY,'o-');
	set(h,'markersize',2);
	maxcamX = max(max(abs(camX)));
	maxcamY = max(max(abs(camY)));
	xlim(maxcamX*[-1.1 1.1]);
	ylim(maxcamY*[-1.1 1.1]);
	title('Undistorted pixel meshgrid');
	xlabel('x \rightarrow');
	ylabel('\leftarrow y');
	grid on;

end
% EOF