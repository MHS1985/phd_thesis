function T = DCM(roll, pitch, heading)

% function [C] = C_bi_n(PHI,THETA,PSI)
% This function returns a 3x3 matrice of rotation between two frames
% The parameters (PHI,THETA,PSI) are the attitude btw the two frames
% The convention in this function is based on the convention of the
% iXblue ATLANS-C IMU system (NWU with the clockwise positive
% z axis rotation )
    
%     % r for roll
%     % p for pitch
%     % h for heading
%     syms r p h;
%     
%     % Rotation matrix around the Z-Axis
%     Mz = [cos(h)  sin(h) 0;-sin(h) cos(h) 0;0 0 1];
%      
%     % Rotation matrix around the Y-Axis
%     My = [cos(p) 0 sin(p);0 1 0;-sin(p) 0 cos(p)];
%     
%     % Rotation matrix around the X-Axis
%     Mx = [1 0  0;0 cos(r) -sin(r);0 sin(r) cos(r)];
%          
%     C_b_n = Mz * My * Mx;
%     
%     r = roll;
%     p = pitch;
%     h = heading;
%     
%     T = eval(C_b_n);
T = [[  cos(heading)*cos(pitch), cos(roll)*sin(heading) + cos(heading)*sin(pitch)*sin(roll),   cos(heading)*cos(roll)*sin(pitch) - sin(heading)*sin(roll)]
     [ -cos(pitch)*sin(heading), cos(heading)*cos(roll) - sin(heading)*sin(pitch)*sin(roll), - cos(heading)*sin(roll) - cos(roll)*sin(heading)*sin(pitch)]
     [        -sin(pitch),                        cos(pitch)*sin(roll),                          cos(pitch)*cos(roll)]];
    
end