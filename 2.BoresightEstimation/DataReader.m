function [Key, x_LiDAR, y_LiDAR, z_LiDAR, roll, pitch, heading, rollSd, pitchSd, headingSd, P_LGF1, P_LGF2, P_LGF3, northingSd, eastingSd, verticalSd, a_normal, b_normal, c_normal, d_normal, planeIndex] = DataReader(selectedPlaneFiles)
% PlaneDataReader
% This function reads a plane file and save its data in various variables
% Input: plane filename
% Output: various data from the plane file

% [1]Key	[2]X_LGF	[3]Y_LGF	[4]Z_LGF	[5]Timestamp_LiDAR_Plane	
% [6]x_LiDAR	[7]y_LiDAR	[8]z_LiDAR	
% [9]roll(rad)	[10]pitch(rad)	[11]heading(rad)	
% [12]roll_Sd(rad)	[13]pitchSd(rad)	[14]headingSd(rad)	
% [15]P_LGF1	[16]P_LGF2	[17]P_LGF3	
% [18]northingSd(m)	[19]eastingSd(m)	[20]verticalSd(m)
% [21]a_normal [22]b_normal [23]c_normal [24]d_normal [25] PlaneIndex

numSelectedPlane = size(selectedPlaneFiles, 1);
indexPoint = 0;

for i = 1:numSelectedPlane
    
    data = load(selectedPlaneFiles(i,1:size(selectedPlaneFiles, 2)));
    numSelectedPlanePoints = size(data, 1);
    
    for j = 1:numSelectedPlanePoints        
         
            Key(j + indexPoint, 1) = data(j, 1);

            x_LiDAR(j + indexPoint, 1) = data(j, 6);
            y_LiDAR(j + indexPoint, 1) = data(j, 7);
            z_LiDAR(j + indexPoint, 1) = data(j, 8);

            roll(j + indexPoint, 1) = data(j, 9);
            pitch(j + indexPoint, 1) = data(j, 10);
            heading(j + indexPoint, 1) = data(j, 11);

            rollSd(j + indexPoint, 1) = data(j, 12);
            pitchSd(j + indexPoint, 1) = data(j, 13);
            headingSd(j + indexPoint, 1) = data(j, 14);

            P_LGF1(j + indexPoint, 1) = data(j, 15);
            P_LGF2(j + indexPoint, 1) = data(j, 16);
            P_LGF3(j + indexPoint, 1) = data(j, 17);    

            northingSd(j + indexPoint, 1) = data(j, 18);
            eastingSd(j + indexPoint, 1) = data(j, 19);
            verticalSd(j + indexPoint, 1) = data(j, 20);
        
            a_normal(j + indexPoint, 1) = data(j, 21);
            b_normal(j + indexPoint, 1) = data(j, 22);
            c_normal(j + indexPoint, 1) = data(j, 23);
            d_normal(j + indexPoint, 1) = data(j, 24);
            
            planeIndex(j + indexPoint, 1) = i;
    end
    
    indexPoint = indexPoint + numSelectedPlanePoints;
end