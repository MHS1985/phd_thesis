function T = Jac_F_dh(r, p, h, dr, dp, dh, xLiDAR, yLiDAR, zLiDAR, a, b, c)
    T = c*(xLiDAR*(cos(dp)*sin(dh)*sin(p) - cos(dh)*cos(dp)*cos(p)*sin(r)) - yLiDAR*(sin(p)*(cos(dh)*cos(dr) - sin(dh)*sin(dp)*sin(dr)) + cos(p)*sin(r)*(cos(dr)*sin(dh) + cos(dh)*sin(dp)*sin(dr))) + zLiDAR*(sin(p)*(cos(dh)*sin(dr) + cos(dr)*sin(dh)*sin(dp)) + cos(p)*sin(r)*(sin(dh)*sin(dr) - cos(dh)*cos(dr)*sin(dp)))) - b*(xLiDAR*(cos(dh)*cos(dp)*(cos(h)*cos(r) - sin(h)*sin(p)*sin(r)) - cos(dp)*cos(p)*sin(dh)*sin(h)) + yLiDAR*((cos(dr)*sin(dh) + cos(dh)*sin(dp)*sin(dr))*(cos(h)*cos(r) - sin(h)*sin(p)*sin(r)) + cos(p)*sin(h)*(cos(dh)*cos(dr) - sin(dh)*sin(dp)*sin(dr))) - zLiDAR*((sin(dh)*sin(dr) - cos(dh)*cos(dr)*sin(dp))*(cos(h)*cos(r) - sin(h)*sin(p)*sin(r)) + cos(p)*sin(h)*(cos(dh)*sin(dr) + cos(dr)*sin(dh)*sin(dp)))) - a*(xLiDAR*(cos(dh)*cos(dp)*(cos(r)*sin(h) + cos(h)*sin(p)*sin(r)) + cos(dp)*cos(h)*cos(p)*sin(dh)) + yLiDAR*((cos(dr)*sin(dh) + cos(dh)*sin(dp)*sin(dr))*(cos(r)*sin(h) + cos(h)*sin(p)*sin(r)) - cos(h)*cos(p)*(cos(dh)*cos(dr) - sin(dh)*sin(dp)*sin(dr))) - zLiDAR*((sin(dh)*sin(dr) - cos(dh)*cos(dr)*sin(dp))*(cos(r)*sin(h) + cos(h)*sin(p)*sin(r)) - cos(h)*cos(p)*(cos(dh)*sin(dr) + cos(dr)*sin(dh)*sin(dp))));
end