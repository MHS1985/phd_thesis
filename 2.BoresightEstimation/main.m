clear; clc;
d2r = pi/180;
format long g;

Metadata45;

selectedPlaneFiles = ...  
          ['Input\UQAR_Calib_45_Montage2_Hor_SubSample.txt_Normal.txt';
           'Input\UQAR_Calib_45_Montage2_Inc_SubSample.txt_Normal.txt';
           'Input\UQAR_Calib_45_Montage2_Ver_SubSample.txt_Normal.txt';
           'Input\UQAR_Calib_45_Montage1_Hor_SubSample.txt_Normal.txt';
           'Input\UQAR_Calib_45_Montage1_Inc_SubSample.txt_Normal.txt';
           'Input\UQAR_Calib_45_Montage1_Ver_SubSample.txt_Normal.txt'];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initial value of factor variance a posteriori
So2 = 1;
epsilon = 0.001;
MaxNumIteration = 10;
chi2Test = 0;

[Key, xLiDAR, yLiDAR, zLiDAR, ...
 r, p, h, rollSd, pitchSd, headingSd, ...
 PxLGF, PyLGF, PzLGF,...
 northingSd, eastingSd, verticalSd,...
 a,b,c,d, planeIndex] = DataReader(selectedPlaneFiles);

nPts = size(Key, 1); % Number of points
n0 = nPts; % Number of equations

numberOfPlanes = size(selectedPlaneFiles ,1);
u0 = 3 + 4 * numberOfPlanes; % Number of unknowns

numObsPerPts = 9;
numOfObs = nPts * numObsPerPts;   % Number of observations
                % For each point we have 9 observations

PLGF = [PxLGF, PyLGF, PzLGF];
rLiDAR = [xLiDAR, yLiDAR, zLiDAR];

delta_Correction_norm = 1;
XLGF = zeros(nPts, 3);

counterIteration = 0;

while( (delta_Correction_norm > epsilon))
    
    % Partial Derivatives of F(droll, dpitch, dheading, a, b, c, d)
    %                                   = a * xLGF + b * yLGF + c * zLGF + d 

    A = zeros(n0, u0);        
    B = zeros(n0, numOfObs);
    Qll = zeros(numOfObs, numOfObs);

    % Georeference to find XLGF   
    R_LiDAR2INS_t = R_LiDAR2INS(dr, dp, dh);

    for i = 1:nPts
        R_INS2LGF_t = R_INS2LGF(r(i), p(i), h(i));
        XLGF(i, 1:3) = PLGF(i,:)' + R_INS2LGF_t * R_LiDAR2INS_t * rLiDAR(i,:)' + R_INS2LGF_t * a_INS;
    end 

    % Add the new normal of plane with the new value (for check only)!!!!    
    for i = 1:nPts

        % Design matrix A
        % Add comments 
        % Jac_F_dr comes from the function!
        A(i, 1) = Jac_F_dr(r(i), p(i), h(i), dr, dp, dh, yLiDAR(i), zLiDAR(i), a(i), b(i), c(i));
        A(i, 2) = Jac_F_dp(r(i), p(i), h(i), dr, dp, dh, xLiDAR(i), yLiDAR(i), zLiDAR(i), a(i), b(i), c(i));
        A(i, 3) = Jac_F_dh(r(i), p(i), h(i), dr, dp, dh, xLiDAR(i), yLiDAR(i), zLiDAR(i), a(i), b(i), c(i));

        A(i, 4 * planeIndex(i)) = Jac_F_a(r(i), p(i), h(i), dr, dp, dh, xLiDAR(i), yLiDAR(i), zLiDAR(i), PxLGF(i), axINS, ayINS, azINS);
        A(i, 4 * planeIndex(i) + 1) = Jac_F_b(r(i), p(i), h(i), dr, dp, dh, xLiDAR(i), yLiDAR(i), zLiDAR(i), PyLGF(i), axINS, ayINS, azINS);
        A(i, 4 * planeIndex(i) + 2) = Jac_F_c(r(i), p(i), dr, dp, dh, xLiDAR(i), yLiDAR(i), zLiDAR(i), PzLGF(i), axINS, ayINS, azINS);
        A(i, 4 * planeIndex(i) + 3) = 1; 

    end
   
    % Calculate B (derivative wrt observations)
    for i = 1:nPts
        B(i, 9 * i - 8) = a(i);     % Jac_F_PxLGF
        B(i, 9 * i - 7) = b(i);     % Jac_F_PyLGF
        B(i, 9 * i - 6) = c(i);     % Jac_F_PzLGF
        
        B(i, 9 * i - 5) = Jac_F_roll(r(i), p(i), h(i), dr, dp, dh, xLiDAR(i), yLiDAR(i), zLiDAR(i), a(i), b(i), c(i), ayINS, azINS);
        B(i, 9 * i - 4) = Jac_F_pitch(r(i), p(i), h(i), dr, dp, dh, xLiDAR(i), yLiDAR(i), zLiDAR(i), a(i), b(i), c(i), axINS, ayINS, azINS);
        B(i, 9 * i - 3) = Jac_F_heading(r(i), p(i), h(i), dr, dp, dh, xLiDAR(i), yLiDAR(i), zLiDAR(i), a(i), b(i), axINS, ayINS, azINS);
        
        B(i, 9 * i - 2) = Jac_F_xLiDAR(r(i), p(i), h(i), dp, dh, a(i), b(i), c(i));
        B(i, 9 * i - 1) = Jac_F_yLiDAR(r(i), p(i), h(i), dr, dp, dh, a(i), b(i), c(i));
        B(i, 9 * i) = Jac_F_zLiDAR(r(i), p(i), h(i), dr, dp, dh, a(i), b(i), c(i));
        
        
        % B1(i, 1) = (a(i) * XLGF(i, 1) + b(i) * XLGF(i, 2) + c(i) * XLGF(i, 3) + d(i));
    end
 
    for i = 1:nPts
        Qll(9 * i - 8, 9 * i - 8) = northingSd(i) ^ 2;
        Qll(9 * i - 7, 9 * i - 7) = eastingSd(i) ^ 2;
        Qll(9 * i - 6, 9 * i - 6) = verticalSd(i) ^ 2;
   
        Qll(9 * i - 5, 9 * i - 5) = rollSd(i) ^ 2;
        Qll(9 * i - 4, 9 * i - 4) = pitchSd(i) ^ 2;
        Qll(9 * i - 3, 9 * i - 3) = headingSd(i) ^ 2;
        
        Qll(9 * i - 2, 9 * i - 2) = xLidarSd ^ 2;
        Qll(9 * i - 1, 9 * i - 1) = yLidarSd ^ 2;
        Qll(9 * i, 9 * i) = zLidarSd ^ 2;
    end
    
    % Closure Matrix 
    
    w = zeros(nPts, 1);
    for i = 1:nPts
        w(i,1) = a(i) * XLGF(i, 1) + b(i) * XLGF(i, 2) + c(i) * XLGF(i, 3) + d(i);
    end
    
    M = B * Qll * B';
    
    N = A' * (M \ A);
    n = - A' * (M \ w);
    
    % Constraint
    
    Ac = zeros(numberOfPlanes, u0);
    
    a_unique = unique(a, 'stable');
    b_unique = unique(b, 'stable');
    c_unique = unique(c, 'stable');
    d_unique = unique(d, 'stable');
    
    for i = 1:numberOfPlanes
       Ac(i, 4 * i) = 2 * a_unique(i);
       Ac(i, 4 * i + 1) = 2 * b_unique(i);
       Ac(i, 4 * i + 2) = 2 * c_unique(i);
       Ac(i, 4 * i + 3) = 0; 
    end
    
    % Pc: constraint weight matrix
    Pc = zeros(numberOfPlanes, numberOfPlanes);
    SD_PlanParam = 1.0e-06; 
    for i = 1:numberOfPlanes
        Pc(i, i) = 1/(SD_PlanParam ^ 2);
    end
    
    % Wc: Closure matrix
    Wc = zeros(numberOfPlanes, 1);
    for i = 1:numberOfPlanes
       Wc(i, 1) = a_unique(i, 1)^2 + b_unique(i, 1)^2 + c_unique(i, 1)^2 -1; 
    end
    
    % Nc calculation
    Nc = Ac' * Pc * Ac;
    
    % nc calculation
    nc = -1 * Ac' * Pc * Wc;
    
    % Correction vector calculation
    N_tot = N + Nc;
    n_tot = n + nc;
    
    delta_Unknowns = N_tot \ n_tot;
    
    % Residual calculation
    v = Qll * B' * inv(M) * (A * inv(N_tot) * A' * inv(M) - eye(n0, n0)) * w;
    
    v2 = Qll * B' * inv(M) * (A * delta_Unknowns + w);

    % Statistics 
     df = n0 - u0 + numberOfPlanes;
     Poid = inv(Qll);

     So2 = (v' * Poid * v)/df;
     disp('So2:'); disp(So2);
     Qxx = inv(N_tot);
    
    % Chi2Test
    % Significance level
    alpha = 0.05; % 95%

    g1 = chi2inv(alpha/2, df);
    g2 = chi2inv(1 - alpha/2, df);
    disp('df * So2 / g2:'); disp(df * So2 / g2);
    disp('df * So2 / g1:'); disp(df * So2 / g1);
    
    if (df * So2 / g2 <= 1 && 1 <= df * So2 / g1)
        chi2Test = 1;   % Chi2Test passed
    else
        chi2Test = 0;   % Chi2Test failed
    end 
       
    dr = dr + delta_Unknowns(1);
    dp = dp + delta_Unknowns(2);
    dh = dh + delta_Unknowns(3);
    
    for i = 1:nPts
        switch planeIndex(i)
            case 1
                a(i) = a(i) + delta_Unknowns(4);
                b(i) = b(i) + delta_Unknowns(5);
                c(i) = c(i) + delta_Unknowns(6);
                d(i) = d(i) + delta_Unknowns(7);
            case 2
                a(i) = a(i) + delta_Unknowns(8);
                b(i) = b(i) + delta_Unknowns(9);
                c(i) = c(i) + delta_Unknowns(10);
                d(i) = d(i) + delta_Unknowns(11);
            case 3
                a(i) = a(i) + delta_Unknowns(12);
                b(i) = b(i) + delta_Unknowns(13);
                c(i) = c(i) + delta_Unknowns(14);
                d(i) = d(i) + delta_Unknowns(15);
            case 4
                a(i) = a(i) + delta_Unknowns(16);
                b(i) = b(i) + delta_Unknowns(17);
                c(i) = c(i) + delta_Unknowns(18);
                d(i) = d(i) + delta_Unknowns(19);
            case 5
                a(i) = a(i) + delta_Unknowns(20);
                b(i) = b(i) + delta_Unknowns(21);
                c(i) = c(i) + delta_Unknowns(22);
                d(i) = d(i) + delta_Unknowns(23);
            case 6
                a(i) = a(i) + delta_Unknowns(24);
                b(i) = b(i) + delta_Unknowns(25);
                c(i) = c(i) + delta_Unknowns(26);
                d(i) = d(i) + delta_Unknowns(27);
        end
    end
    
    delta_Correction_norm = norm(delta_Unknowns);
    
    counterIteration = counterIteration + 1;
    disp('counterIteration:'); disp(counterIteration);
    disp('Corrections:'); disp(delta_Unknowns(1:3));
    
    disp('delta_Correction_norm:'); disp(delta_Correction_norm); 
    
end

disp('dr:'); disp(dr * 180/pi);
disp('dp:'); disp(dp * 180/pi);
disp('dh:'); disp(dh * 180/pi);
disp('Standard deviation dr:'); disp(sqrt(Qxx(1,1))* 180/pi);
disp('Standard deviation dp:'); disp(sqrt(Qxx(2,2))* 180/pi);
disp('Standard deviation dh:'); disp(sqrt(Qxx(3,3))* 180/pi);

% Add plot

