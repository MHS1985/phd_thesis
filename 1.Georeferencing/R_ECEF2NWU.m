function R_ECEF_NWU = R_ECEF2NWU(lat, lon)
% Calculation of the Rotation matrix between ECEF frame to LGF frame

% syms lambda phi
% 
% % LGF = NWU
% R_ECEF_NWU(lambda, phi) = [-cos(lambda)*sin(phi), -sin(lambda)*sin(phi), cos(phi);
%                     sin(lambda)    ,      -cos(lambda)    ,   0     ;
%                cos(lambda)*cos(phi), cos(phi)*sin(lambda) , sin(phi)];
% 
% disp(R_ECEF_NWU);

R_ECEF_NWU = [[ -cos(lon)*sin(lat), -sin(lon)*sin(lat), cos(lat)]
             [           sin(lon),          -cos(lon),        0]
             [  cos(lon)*cos(lat),  cos(lat)*sin(lon), sin(lat)]];
              
end 