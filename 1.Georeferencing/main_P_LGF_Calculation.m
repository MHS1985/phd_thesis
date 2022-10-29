% Pn Calculation
clc; clear;
d2r = pi/180;
format long g;

% Reference point calculation
data_INS_NAV = load('ATLANS-20181019-100528_OutC_POSTPROCESSING-replay.xpf.txt');

lat_All = data_INS_NAV(:, 5);
lat_Ref = mean(lat_All) * d2r;

lon_All = data_INS_NAV(:, 6);
lon_Ref = mean(lon_All) * d2r;

alt_All = data_INS_NAV(:, 7);
alt_Ref = mean(alt_All);

% Calculation the ECEF coordinates of the reference point
p_ECEF_Ref = LatLonAlt2ECEF(lat_Ref, lon_Ref, alt_Ref);

% R_ECEF_NWU for Reference Point
R_ECEF_NWU_Ref = R_ECEF2NWU(lat_Ref, lon_Ref);

ZF_filename = '20181019_2_10';
data_INS_Interp = load(strcat(ZF_filename, '.txt_INS_data_Interp.txt'));

% Calculation of ECEF coordinates with (lat, lon, alt) parameters
lat = data_INS_Interp(:, 2) * d2r;
lon = data_INS_Interp(:, 3) * d2r;
alt = data_INS_Interp(:, 4);

NumOfPoints = size(lat,1);
p_ECEF = zeros(NumOfPoints,3);

for i = 1 : NumOfPoints
   p_ECEF(i, 1:3) = LatLonAlt2ECEF(lat(i, 1), lon(i, 1), alt(i,1));
end

% P_LGF final calculation
P_LGF_NWU = zeros(NumOfPoints, 3);

for i = 1 : NumOfPoints
   P_LGF_NWU(i,1:3) = (P_LGF(p_ECEF(i,:), R_ECEF_NWU_Ref, p_ECEF_Ref))';
 end

DataWriter(strcat(ZF_filename, '.txt_P_LGF_NWU.txt'), P_LGF_NWU);

