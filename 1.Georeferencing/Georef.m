function [X_LGF] = Georef(P_LGF, R_IMU_LGF, R_LiDAR_IMU, r_LiDAR, a_IMU)
% function [X_LGF] = Georef(P_LGF, R_IMU_LGF, R_LiDAR_IMU, r_LiDAR, a_IMU)
% This function returns the georeferenced coordinates of MLS 
% Georeference the points
% Input: P_LGF, R_IMU_LGF, R_LiDAR_IMU, r_LiDAR, a_IMU

    X_LGF = P_LGF + R_IMU_LGF * R_LiDAR_IMU * r_LiDAR + R_IMU_LGF * a_IMU;
    
end