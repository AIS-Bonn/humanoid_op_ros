% cameraparams.m - Philipp Allgeuer - 02/07/13
% Set the required camera parameters (returns a struct)

% Main function
function [p] = cameraparams()

	% Camera resolution
	p.rx = 800;
	p.ry = 600;

	% Camera parameters
	p.fx = 2.7724805111667723e+02;
	p.fy = 2.7701284455878204e+02;
	p.cx = 4.0952798866945773e+02;
	p.cy = 2.9034600289643578e+02;

	% Radial distortion parameters
	p.k1 = -2.5628781903712217e-01;
	p.k2 =  3.8457107179943981e-02;
	p.k3 =  5.6597466197309959e-03;
	p.k4 =  4.5320486285066977e-02;
	p.k5 = -6.7883260548786864e-02;
	p.k6 =  2.5466647250764318e-02;
	
	% Tangential distortion parameters
	p.p1 =  7.1863283209847332e-04;
	p.p2 = -4.6246652219500986e-04;
	
	% Linear distortion extension parameters
	p.uselinext = false; % The next line should set this to true internally
	p = calcparams(p,[-12.0 -8.0 8.0 14.0;-14.0 -8.0 8.0 12.0]);

%  	%
%  	% Old Parameters
%  	%
%  	
%  	% Camera resolution
%  	p.rx = 800;
%  	p.ry = 600;
%  
%  	% Camera parameters
%  	p.fx = 2.9923719209941140e+02;
%  	p.fy = 2.9869074219007530e+02;
%  	p.cx = 3.9217782062880826e+02;
%  	p.cy = 3.0355174547068725e+02;
%  
%  	% Radial distortion parameters
%  	p.k1 = -2.3508879327145624e-01;
%  	p.k2 =  4.0710216140580013e-02;
%  	p.k3 = -2.5554680737527942e-03;
%  	p.k4 = 0;
%  	p.k5 = 0;
%  	p.k6 = 0;
%  	
%  	% Tangential distortion parameters
%  	p.p1 = -1.7019357304546496e-03;
%  	p.p2 =  1.8013653365863906e-03;
%  	
%  	% Disable linear distortion extension
%  	p.uselinext = false;

end
% EOF