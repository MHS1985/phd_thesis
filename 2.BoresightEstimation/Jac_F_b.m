function T = Jac_F_b(r, p, h, dr, dp, dh, xLiDAR, yLiDAR, zLiDAR, PyLGF, axINS, ayINS, azINS)
     T = PyLGF + ayINS*(cos(h)*cos(r) - sin(h)*sin(p)*sin(r)) - azINS*(cos(h)*sin(r) + cos(r)*sin(h)*sin(p)) - zLiDAR*((cos(dh)*sin(dr) + cos(dr)*sin(dh)*sin(dp))*(cos(h)*cos(r) - sin(h)*sin(p)*sin(r)) - cos(p)*sin(h)*(sin(dh)*sin(dr) - cos(dh)*cos(dr)*sin(dp)) + cos(dp)*cos(dr)*(cos(h)*sin(r) + cos(r)*sin(h)*sin(p))) - xLiDAR*(cos(dp)*sin(dh)*(cos(h)*cos(r) - sin(h)*sin(p)*sin(r)) - sin(dp)*(cos(h)*sin(r) + cos(r)*sin(h)*sin(p)) + cos(dh)*cos(dp)*cos(p)*sin(h)) - yLiDAR*(cos(p)*sin(h)*(cos(dr)*sin(dh) + cos(dh)*sin(dp)*sin(dr)) - (cos(dh)*cos(dr) - sin(dh)*sin(dp)*sin(dr))*(cos(h)*cos(r) - sin(h)*sin(p)*sin(r)) + cos(dp)*sin(dr)*(cos(h)*sin(r) + cos(r)*sin(h)*sin(p))) - axINS*cos(p)*sin(h);
end