function [G,Y] = genGeometryMatrix(ranges,svVel,psr,doppler,estimatedPos,estVel,estClockBias,estBiasRate)
%GENGEOMETRYMATRIX Summary of this function goes here
%   Detailed explanation goes here

xRanges = ranges(:,1);
yRanges = ranges(:,2);
zRanges = ranges(:,3);

xVel = svVel(:,1);
yVel = svVel(:,2);
zVel = svVel(:,3);

pseudoranges = psr;

userPos = estimatedPos;
userVel = estVel;
clockBias = estClockBias;
biasRate = estBiasRate;

G_unit = zeros(length(xRanges),4); %unit vector matrix
zeroMat = zeros(length(xRanges),4);
G_total = zeros(2*length(xRanges),8); %geometry matrix
Y_psr = zeros(length(xRanges),1); %measurement matrix
Y_dop = zeros(length(xRanges),1); %measurement matrix

for i = 1:length(xRanges)
    %build the geometry matrix and 
    svPos = [xRanges(i),yRanges(i),zRanges(i)];
    svVel = [xVel(i),yVel(i),zVel(i)];
    [pseudoRange,pseudoRangeHat,unitX,unitY,unitZ] = genPseudoHat(svPos,svVel,userPos,clockBias,biasRate);
    G_unit(i,:) = [-unitX , -unitY, -unitZ, 1];
    Y_psr(i,1) = pseudoranges(i) - pseudoRange;
    Y_dop(i,1) = doppler(i) - pseudoRangeHat;
    
end
G_total = [G_unit , zeroMat ; zeroMat , G_unit];

G = G_total;

Y = [Y_psr ; Y_dop];
end

