function T = R_LiDAR2INS(dr, dp, dh)
    % % R_LiDAR_INS
    % % Input : roll boresight, pitch boresight, heading boresight => Unknowns calibration params. 
    % % Output: 3x3 rotation matrix from LiDAR to INS.
    % 
    % % dr => roll boresight
    % % dp => pitch boresight
    % % dh => heading boresight
    % syms dr dp dh;
    % 
    % % Rotation matrix around the Z-Axis
    % Mz = [cos(dh)  sin(dh) 0;-sin(dh) cos(dh) 0;0 0 1];
    % 
    % % Rotation matrix around the Y-Axis
    % My = [cos(dp) 0 sin(dp);0 1 0;-sin(dp) 0 cos(dp)];
    % 
    % % Rotation matrix around the X-Axis
    % Mx = [1 0  0;0 cos(dr) -sin(dr);0 sin(dr) cos(dr)];
    % 
    % T = eval(Mz * My * Mx);
    % 
    % R_LiDAR_INS_eval = Mz * My * Mx;
    % 
    % disp('R_LiDAR_INS:'); disp(R_LiDAR_INS_eval);
    % disp('T:'); disp(T);

    T = [[  cos(dh)*cos(dp), cos(dr)*sin(dh) + cos(dh)*sin(dp)*sin(dr),   cos(dh)*cos(dr)*sin(dp) - sin(dh)*sin(dr)]
    [ -cos(dp)*sin(dh), cos(dh)*cos(dr) - sin(dh)*sin(dp)*sin(dr), - cos(dh)*sin(dr) - cos(dr)*sin(dh)*sin(dp)]
    [         -sin(dp),                           cos(dp)*sin(dr),                             cos(dp)*cos(dr)]];

end
