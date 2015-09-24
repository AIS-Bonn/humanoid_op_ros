% calcparams.m - Philipp Allgeuer - 05/07/13
% Calculate additional camera parameters based on the OpenCV ones
%
% p:    A struct containing the camera parameters from the OpenCV calibration
% lims: The a and b values to start and stop fading to the linear extension model

% Main function
function pext = calcparams(p, lims)

	% Set parameters if they weren't provided
	if nargin < 1
		p = cameraparams;
	end
	if nargin < 2
		lims = [-12.0 -8.0 8.0 14.0;-14.0 -8.0 8.0 12.0];
	end
	
	% Constants
	N = 51; % Number of sample data points to take around the edges
	
	% Give the limits systematic names ([a]/[b] + [p]ositive/[n]egative + [i]nner/[o]uter)
	api = lims(1,3);
	apo = lims(1,4);
	ani = lims(1,2);
	ano = lims(1,1);
	bpi = lims(2,3);
	bpo = lims(2,4);
	bni = lims(2,2);
	bno = lims(2,1);
	
	% Create points around the outer border
	Uedgec = [linspace(ano,apo,N)' bno*ones(N,1) ones(N,1)];
	Dedgec = [linspace(ano,apo,N)' bpo*ones(N,1) ones(N,1)];
	Ledgec = [ano*ones(N,1) linspace(bno,bpo,N)' ones(N,1)];
	Redgec = [apo*ones(N,1) linspace(bno,bpo,N)' ones(N,1)];
	
	% Distort the edges
	p.uselinext = false; % Force ignoring of current linear extension parameters
	Uedgep = distort(Uedgec,p);
	Dedgep = distort(Dedgec,p);
	Ledgep = distort(Ledgec,p);
	Redgep = distort(Redgec,p);
	
	% Calculate average coordinates of the distorted edges
	Uedgeavg = mean(Uedgep(:,2));
	Dedgeavg = mean(Dedgep(:,2));
	Ledgeavg = mean(Ledgep(:,1));
	Redgeavg = mean(Redgep(:,1));
	
	% Calculate linear extension parameters
	mx = (Redgeavg - Ledgeavg)/(apo - ano);
	bx = Redgeavg - mx*apo;
	my = (Uedgeavg - Dedgeavg)/(bno - bpo);
	by = Uedgeavg - my*bno;
	
	% Construct the required pext
	pext = p;
	pext.uselinext = true;
	pext.api = api;
	pext.apo = apo;
	pext.ani = ani;
	pext.ano = ano;
	pext.bpi = bpi;
	pext.bpo = bpo;
	pext.bni = bni;
	pext.bno = bno;
	pext.mx = mx;
	pext.bx = bx;
	pext.my = my;
	pext.by = by;

end
% EOF