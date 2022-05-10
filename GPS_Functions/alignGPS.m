function [gpsVec1] = alignGPS(gpsData1,gpsTime1,gpsTime2)
%ALIGNGPS Summary of this function goes here
%   Detailed explanation goes h

gpsTime1 = gpsTime1 - gpsTime2(1);
gpsTime2 = gpsTime2 - gpsTime2(1);
j = 1;
thresh = .05;
gps1Index = [];
diff = [];
kk = 1;
for k = 1:length(gpsTime1)
    
    if(abs(gpsTime1(k) - gpsTime2(j)) < thresh)
        diff(kk) = abs(gpsTime1(k) - gpsTime2(j));
        kk = kk+1;
        gps1Index(j) = k;
        j = j+1;
        if( j > length(gpsTime2))
            break
        end
    end
end


gpsVec1 = gpsData1(gps1Index);
end

