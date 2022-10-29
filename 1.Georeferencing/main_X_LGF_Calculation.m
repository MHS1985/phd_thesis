clear; clc; 

d2r = pi/180;
format long g;

% Lever arms
a1 = -0.602;
a2 = 0.065;
a3 = 0.356;
a_INS = [a1;a2;a3];

% Initial boresight angles
rollBoresight = 0;
pitchBoresight = 45 * d2r;
headingBoresight = 0;

filename = '20181019_2_10';

data_ZF = load(strcat(filename, '.txt_Plane.txt'));
data_INS_Interp = load(strcat(filename, '.txt_INS_data_Interp.txt'));

% P_LGF
P_LGF_NWU = load(strcat(filename, '.txt_P_LGF_NWU.txt'));

% Attitude
heading = data_INS_Interp(:, 9) * d2r;
roll = data_INS_Interp(:, 10) * d2r;
pitch = data_INS_Interp(:, 11) * d2r;

% r_LiDAR
x_LiDAR = data_ZF(:, 1);
y_LiDAR = data_ZF(:, 2);
z_LiDAR = data_ZF(:, 3);
r_LiDAR = [data_ZF(:, 1), data_ZF(:, 2), data_ZF(:, 3)];

NumOfPoints = size(data_ZF,1);
X_LGF = zeros(NumOfPoints,3);

R_LiDAR_INS = DCM(rollBoresight, pitchBoresight, headingBoresight);

for i = 1:NumOfPoints
    R_INS_LGF = DCM(roll(i), pitch(i), heading(i));
    X_LGF(i, 1:3) = (Georef(P_LGF_NWU(i,:)', R_INS_LGF, R_LiDAR_INS,r_LiDAR(i,:)', a_INS))';
end 

key = zeros(NumOfPoints, 1);
for i = 1:NumOfPoints   
   key(i, 1) = i; 
end

DataWriter(strcat(filename, '.txt_Plane_X_LGF_NWU_Key.txt'), X_LGF, key);
