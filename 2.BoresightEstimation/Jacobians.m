syms r p h;
syms dr dp dh;
syms PxLGF PyLGF PzLGF;
syms axINS ayINS azINS;
syms xLiDAR yLiDAR zLiDAR;
syms a b c d;

R_INS_LGF = [[  cos(h)*cos(p), cos(r)*sin(h) + cos(h)*sin(p)*sin(r),   cos(h)*cos(r)*sin(p) - sin(h)*sin(r)]
[ -cos(p)*sin(h), cos(h)*cos(r) - sin(h)*sin(p)*sin(r), - cos(h)*sin(r) - cos(r)*sin(h)*sin(p)]
[        -sin(p),                        cos(p)*sin(r),                          cos(p)*cos(r)]];

R_LiDAR_INS = [[  cos(dh)*cos(dp), cos(dr)*sin(dh) + cos(dh)*sin(dp)*sin(dr),   cos(dh)*cos(dr)*sin(dp) - sin(dh)*sin(dr)]
[ -cos(dp)*sin(dh), cos(dh)*cos(dr) - sin(dh)*sin(dp)*sin(dr), - cos(dh)*sin(dr) - cos(dr)*sin(dh)*sin(dp)]
[         -sin(dp),                           cos(dp)*sin(dr),                             cos(dp)*cos(dr)]];

R_LiDAR_LGF = R_INS_LGF * R_LiDAR_INS;

pLGF = [PxLGF; PyLGF; PzLGF] + R_LiDAR_LGF * [xLiDAR; yLiDAR; zLiDAR] + R_INS_LGF * [axINS; ayINS; azINS];

xLGF = pLGF(1, 1);
yLGF = pLGF(2, 1);
zLGF = pLGF(3, 1);

F = a * xLGF + b * yLGF + c * zLGF + d;

G = a * a + b * b + c * c - 1;

% Elements of the design matrix A
Jac_F_dr = diff(F, dr);
Jac_F_dp = diff(F, dp);
Jac_F_dh = diff(F, dh);
Jac_F_a = diff(F, a);
Jac_F_b = diff(F, b);
Jac_F_c = diff(F, c);
Jac_F_d = diff(F, d);

% Elements of the B matrix
Jac_F_PxLGF = diff(F, PxLGF);
Jac_F_PyLGF = diff(F, PyLGF);
Jac_F_PzLGF = diff(F, PzLGF);

Jac_F_roll = diff(F, r);
Jac_F_pitch = diff(F, p);
Jac_F_heading = diff(F, h);

Jac_F_xLiDAR = diff(F, xLiDAR);
Jac_F_yLiDAR = diff(F, yLiDAR);
Jac_F_zLiDAR = diff(F, zLiDAR);

% Elements of the Ac matrix
Jac_G_a = diff(G, a);
Jac_G_b = diff(G, b);
Jac_G_c = diff(G, c);
Jac_G_d = diff(G, d);
