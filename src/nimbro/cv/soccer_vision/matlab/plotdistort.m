% plotdistort.m - Philipp Allgeuer - 02/07/13
% Visualise the distortion of the camera

% Main function
function plotdistort(Ax, Ay, p)

	% Retrieve camera parameters
	if nargin < 1
		Ax = 7.0;
	end
	if nargin < 2
		Ay = 5.0;
	end
	if nargin < 3
		p = cameraparams;
	end

	% Parameters
	Az = 1;
	Nx = 81;
	Ny = 61;

	% Construct the meshgrid to distort
	Xvals = linspace(-Ax,Ax,Nx);
	Yvals = linspace(-Ay,Ay,Ny);
	[X, Y] = meshgrid(Xvals,Yvals);
	Xc = X(:);
	Yc = Y(:);
	Zc = ones(size(Xc,1),1);
	camvec = [Xc Yc Zc];
	
	% Distort the required data points
	pixvec = distort(camvec,p);
	Xp = reshape(pixvec(:,1),size(X));
	Yp = reshape(pixvec(:,2),size(Y));
	
	% Get plotting coordinates (and adjust for fact that y is positive downward)
	pixX = Xp';
	pixY = p.ry - Yp';
	
	% Plot the distorted meshgrid
	figure(4);
	subplot(1,1,1);
	h = plot(pixX,pixY,'o-');
	set(h,'markersize',2);
	axis([0 p.rx 0 p.ry]);
	title('Distorted camera frame meshgrid');
	xlabel('x \rightarrow');
	ylabel('\leftarrow y');
	grid on;

end
% EOF