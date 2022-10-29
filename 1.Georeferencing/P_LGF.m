function P = P_LGF(P_ECEF, R_ECEF_Ref, P_ECEF_Ref)
% function P_LGF_NWU = P_LGF2NWU(P_ECEF, R_ECEF_Ref, P_ECEF_Ref)
% This function calculates the coordinates of the point in LGF frame with
% regard to ECEF frame

    P = R_ECEF_Ref * [(P_ECEF(1) - P_ECEF_Ref(1)); (P_ECEF(2) - P_ECEF_Ref(2)); (P_ECEF(3) - P_ECEF_Ref(3))];
end