function [eccAnom] = calcAnomaly(meanAnomaly,e)
%CALCANOMALY Summary of this function goes here
%   Detailed explanation goes here
eccAnom = 10;
fVal = meanAnomaly;
tmpVal = 0;
while(abs(fVal - eccAnom) > 1e-12)
eccAnom = fVal;    
fVal = meanAnomaly + e*sin(eccAnom);
tmpVal = fVal;
end
eccAnom = fVal;
    
end

