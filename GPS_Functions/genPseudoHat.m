function [pseudoRange,pseudoRangeHat,unitX,unitY,unitZ] = genPseudoHat(svPos,svVel,userPos,clockBias,biasRate)
%UNITVECTORS Summary of this function goes here
%   Detailed explanation goes here

%C = 299792458; %m/s
range = sqrt((svPos(1)-userPos(1))^2 + (svPos(2)-userPos(2))^2 + (svPos(3)-userPos(3))^2);
pseudoRange = range + clockBias;

unitX = (svPos(1)-userPos(1)) / range;
unitY = (svPos(2)-userPos(2)) / range;
unitZ = (svPos(3)-userPos(3)) / range;

pseudoRangeHat = unitX*svVel(1) + unitY*svVel(2) + unitZ*svVel(3);

end
