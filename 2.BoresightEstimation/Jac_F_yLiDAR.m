function T = Jac_F_yLiDAR(r, p, h, dr, dp, dh, a, b, c)
    T = c*(cos(p)*sin(r)*(cos(dh)*cos(dr) - sin(dh)*sin(dp)*sin(dr)) - sin(p)*(cos(dr)*sin(dh) + cos(dh)*sin(dp)*sin(dr)) + cos(dp)*cos(p)*cos(r)*sin(dr)) + a*((cos(dh)*cos(dr) - sin(dh)*sin(dp)*sin(dr))*(cos(r)*sin(h) + cos(h)*sin(p)*sin(r)) + cos(h)*cos(p)*(cos(dr)*sin(dh) + cos(dh)*sin(dp)*sin(dr)) - cos(dp)*sin(dr)*(sin(h)*sin(r) - cos(h)*cos(r)*sin(p))) - b*(cos(p)*sin(h)*(cos(dr)*sin(dh) + cos(dh)*sin(dp)*sin(dr)) - (cos(dh)*cos(dr) - sin(dh)*sin(dp)*sin(dr))*(cos(h)*cos(r) - sin(h)*sin(p)*sin(r)) + cos(dp)*sin(dr)*(cos(h)*sin(r) + cos(r)*sin(h)*sin(p)));
end