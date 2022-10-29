d2r = pi/180;

% Boresight angles
dr = 0;
dp = 45 * d2r;
dh = 0;

% Lever arms 45(deg) pitch
axINS = -0.597;
ayINS = 0.081;
azINS = 0.356;
a_INS = [axINS;ayINS;azINS];

% Incertitudes of LiDAR points
xLidarSd = 0.0031;
yLidarSd = 0.0031;
zLidarSd = 0.0031;

% To be calculated with error propagation law
axSD = 0.02;
aySD = 0.02;
azSD = 0.02;

% Standard deviation of the lidar data
% These values come from the specs of the LiDAR system!!!
% I have to check with Z+F scans
% According to "datasheet_p9012.zf.en.pdf"
sd_rho = 0.0031;        % worse scenario: 1 Sigma Range Noise, 50 m. Z+F Profiler 9012. Black (14 %)
sd_gamma =  0.02 * d2r;
sd_alpha =  0.02 * d2r;

% Latency between LiDAR and GNSS receiver
% dTlp = 0.000;
dT_LiDAR_GNSS = 0.000;

% Latency between IMU and LiDAR
% dTli = 0.000;
dT_LiDAR_INS = 0.000;
