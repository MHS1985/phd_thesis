function p_ECEF = LatLonAlt2ECEF(lat, lon, alt)
    % function  [x_ECEF, y_ECEF, z_ECEF] = LatLonATL2ECEF(lat, lon, alt)
    % This function calculates the coordinates of a point in ECEF frame
    % with respect to latitude, longitude and altitude data
    
    a = 6378137;
    e = 0.081819190842622;
    
    Rn = a / sqrt(1 - e * e * sin(lat) * sin(lat));
    x_ECEF = (Rn + alt) * cos(lat) * cos(lon);
    y_ECEF = (Rn + alt) * cos(lat) * sin(lon);
    z_ECEF = (Rn * (1 - e * e) + alt) * sin(lat);
    
    p_ECEF = [x_ECEF, y_ECEF, z_ECEF];
end