function [userPos, userVel, clockBias, biasRate, covar] = getUserPos(satRange,satVel,pseudoranges,dopp,initPos,initVel,initCB,initCBR)
%GETUSERPOS getUserPos(xRanges,yRanges,zRanges,pseudoranges)
%   Detailed explanation goes here

% if((length(xRanges) ~= length(yRanges)) && (length(yRanges) ~= length(zRanges)))
%     error('Range Vecctors must be same length!')
% end
c = 299792458; %m/s

userPos = [initPos(1),initPos(2),initPos(3)];
userVel = [initVel(1), initVel(2), initVel(3)];
clockBias = initCB*c;
BiasRate = initCBR*c;
converged = 0;
i = 1;
while converged == 0
    
    [G,Y] = genGeometryMatrix(satRange,satVel,pseudoranges,dopp,userPos,userVel,clockBias,BiasRate);

    [correction,covariance] = least_squares(G,Y);

    if(isnan(correction(1)))
        error('Position Update is NaN')
    end
    
    userPos(1) = userPos(1) + correction(1);
    userPos(2) = userPos(2) + correction(2);
    userPos(3) = userPos(3) + correction(3);
    clockBias = clockBias + correction(4);
    
    userVel(1) = userVel(1) + correction(5);
    userVel(2) = userVel(1) + correction(6);
    userVel(3) = userVel(1) + correction(7);
    BiasRate = userVel(1) + correction(8);

    if(sqrt(correction(1)^2 + correction(2)^2 + correction(3)^2 + correction(4)^2) < .001)
        converged = 1;
    end
    if(i > 50)
        converged = 1;
        warning('Maximum Iterations Reached - Poor Geometry Detected')
    end
    i;
    i = i+1;
end

userPos = [userPos(1), userPos(2), userPos(3)];
userVel = [userVel(1), userVel(2), userVel(3)];
clockBias = clockBias / c;
biasRate = BiasRate / c;
covar = diag(covariance);
end

