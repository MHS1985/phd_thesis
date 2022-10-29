function T = R_INS2LGF(r, p, h)
% % R_INS_LGF
% % Input : roll, pitch, heading => Observations from INS. 
% % Output: 3x3 rotation matrix from INS to LGF.
    % 
    % % r => roll
    % % p => pitch
    % % h => heading
    % syms r p h;
    % 
    % % Rotation matrix around the Z-Axis
    % Mz = [cos(h)  sin(h) 0;-sin(h) cos(h) 0;0 0 1];
    % 
    % % Rotation matrix around the Y-Axis
    % My = [cos(p) 0 sin(p);0 1 0;-sin(p) 0 cos(p)];
    % 
    % % Rotation matrix around the X-Axis
    % Mx = [1 0  0;0 cos(r) -sin(r);0 sin(r) cos(r)];
    % 
    % R_INS_LGF_eval = Mz * My * Mx;
    % 
    % T = eval(Mz * My * Mx);

    % disp('R_INS_LGF:'); disp(R_INS_LGF_eval);
    % disp('T:'); disp(T);


    T = [[  cos(h)*cos(p), cos(r)*sin(h) + cos(h)*sin(p)*sin(r),   cos(h)*cos(r)*sin(p) - sin(h)*sin(r)]
    [ -cos(p)*sin(h), cos(h)*cos(r) - sin(h)*sin(p)*sin(r), - cos(h)*sin(r) - cos(r)*sin(h)*sin(p)]
    [        -sin(p),                        cos(p)*sin(r),                          cos(p)*cos(r)]];
 
end
