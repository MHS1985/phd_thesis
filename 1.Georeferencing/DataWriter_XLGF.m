function DataWriter(filename, X_LGF, key)

% This funtion write the X_LGF calculated in the LGF frame in a text file
% by the name filename
    p = [key'; X_LGF(:,1)';X_LGF(:,2)';X_LGF(:,3)'];
    fid = fopen(filename,'w');
    fprintf(fid, '%%[1]key [2]X1 \t [3]X2 \t [4]X3\n');

    fprintf(fid, '%d %.12f %.12f %.12f\n', p);
    fclose(fid);
end