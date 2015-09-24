%
% Do a test of undistort -> distort
%

% Parameters
N = 20000;
p = cameraparams;

% Print header
disp(' ');
disp('UNDISTORT => DISTORT TEST');
disp('=========================');
disp(' ');

% Generate random data points
xr = p.rx*rand(N,1);
yr = p.ry*rand(N,1);

% Plot the random data points
figure(5);
subplot(1,1,1);
h = plot(xr,p.ry-yr,'bo');
set(h,'markersize',3);
xlim([-50 p.rx+50]);
ylim([-50 p.ry+50]);
grid on;
xlabel('x \rightarrow');
ylabel('\leftarrow y');
title('Pixel points for Undistort \rightarrow Distort test');

% Flush output to screen
fflush(stdout);

% Perform the undistortion and re-distortion
origpixvec = [xr yr];
[camvec, numits] = undistort(origpixvec);
pixvec = distort(camvec);

% Calculate deviations
differ = pixvec - origpixvec;
dist = sqrt(differ(:,1).*differ(:,1) + differ(:,2).*differ(:,2));

% Find maximums and minimums
[maxdist, maxrow] = max(dist);
[mindist, minrow] = min(dist);

% Print information to screen
disp('Iteration counts:');
maxits = max(numits);
for it = 0:(maxits+1)
	num = sum(numits == it);
	disp([num2str(it,'%2d') ': ' num2str(num)]);
end
disp(' ');
disp(['Maximum pixel norm-error: ' num2str(maxdist)]);
disp(['     Corresponding pixel: ' num2str(origpixvec(maxrow,:))]);
disp(['Minimum pixel norm-error: ' num2str(mindist)]);
disp(['     Corresponding pixel: ' num2str(origpixvec(minrow,:))]);
disp(' ');

% Plot undistortion results
figure(6);
subplot(2,1,1);
plot(1:N,numits,'Color',[0.15 0.70 0.15]);
title('Number of iterations (Undistort \rightarrow Distort)');
ylim([0 maxits+1]);
grid on;
subplot(2,1,2);
plot(1:N,dist,'m');
title('Pixel norm-error (Undistort \rightarrow Distort)');
ylim([0 1.2*max(dist)]);
grid on;

% Plot undistorted points
figure(7);
subplot(1,1,1);
h = plot(camvec(:,1),-camvec(:,2),'ro');
set(h,'markersize',3);
maxcamvec = max(abs(camvec));
xlim(maxcamvec(1)*[-1.1 1.1]);
ylim(maxcamvec(2)*[-1.1 1.1]);
title('Camera frame points for Undistort \rightarrow Distort test (z = 1 plane)');
grid on;

% Flush output to screen
fflush(stdout);

%
% Do a test of distort -> undistort
%

% Print header
disp(' ');
disp('DISTORT => UNDISTORT TEST');
disp('=========================');
disp(' ');

% Generate random data points
xr = 4.55*(2*rand(N,1)-1);
yr = 1.70*(2*rand(N,1)-1);

% Plot the random data points
figure(8);
subplot(1,1,1);
h = plot(xr,-yr,'ro');
set(h,'markersize',3);
xlim([-5.0 5.0]);
ylim([-2.5 2.5]);
grid on;
xlabel('x \rightarrow');
ylabel('\leftarrow y');
title('Camera frame points for Distort \rightarrow Undistort test');

% Perform the distortion and undistortion
origcamvec = [xr yr ones(N,1)];
pixvec = distort(origcamvec);
[camvec, numits] = undistort(pixvec);

% Calculate deviations
differ = camvec - origcamvec;
dist = sqrt(differ(:,1).*differ(:,1) + differ(:,2).*differ(:,2));

% Find maximums and minimums
[maxdist, maxrow] = max(dist);
[mindist, minrow] = min(dist);

% Print information to screen
disp('Iteration counts:');
maxits = max(numits);
for it = 0:(maxits+1)
	num = sum(numits == it);
	disp([num2str(it,'%2d') ': ' num2str(num)]);
end
disp(' ');
disp(['Maximum camera frame norm-error: ' num2str(maxdist)]);
disp(['       Corresponding coordinate: ' num2str(origcamvec(maxrow,:))]);
disp(['Minimum camera frame norm-error: ' num2str(mindist)]);
disp(['       Corresponding coordinate: ' num2str(origcamvec(minrow,:))]);
disp(' ');

% Plot undistortion results
figure(9);
subplot(2,1,1);
plot(1:N,numits,'Color',[0.15 0.70 0.15]);
title('Number of iterations (Distort \rightarrow Undistort)');
ylim([0 maxits+1]);
grid on;
subplot(2,1,2);
plot(1:N,dist,'m');
title('Camera frame norm-error (Distort \rightarrow Undistort)');
ylim([0 1.2*max(dist)]);
grid on;

% Plot distorted points
figure(10);
subplot(1,1,1);
h = plot(pixvec(:,1),pixvec(:,2),'bo');
set(h,'markersize',3);
xlim([0 p.rx]);
ylim([0 p.ry]);
title('Pixel points for Distort \rightarrow Undistort test');
grid on;
% EOF