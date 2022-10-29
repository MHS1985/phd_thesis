function DataWriter(filename, P_LGF)
% function DataWriter(filename, P_LGF)
% This funtion write the P_LGF calculated in the LGF frame in a text file
% by the name filename
    p = [P_LGF(:,1)';P_LGF(:,2)';P_LGF(:,3)'];
    fid = fopen(filename,'w');
    fprintf(fid, '%%[1]P1 \t [2]P2 \t [3]P3\n');

    fprintf(fid, '%.12f %.12f %.12f\n', p);
    fclose(fid);
end