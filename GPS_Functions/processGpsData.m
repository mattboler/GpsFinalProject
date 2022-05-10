function [time, lla, cb,cbr,dopDiag] = processGpsData(psrTimeIn,psrIn,dopIn,carrierIn,cnoIn,prnIn,ephem)
%PROCESSGPSDATA Summary of this function goes here
%   Detailed explanation goes here
xVec = {};
yVec = {};
zVec = {};
xVelVec = {};
yVelVec = {};
zVelVec = {};
dopVec = {};
psrVec = {};
cnoVec = {};
carVec = {};

[rows,cols] = size(psrIn);
c = physconst('LightSpeed');
for i = 1:rows
    [psr,cno,doppler,carrier,satID] = removeZeros(psrIn(i,:),cnoIn(i,:),dopIn(i,:),carrierIn(i,:),prnIn(i,:));
    timeVec = psrTimeIn(i);
    clear x_ecef y_ecef z_ecef xv_ecef yv_ecef zv_ecef
    for j = 1:length(satID)
        transTime = psr(j)/c;
        [satPosition, satVel, clockCorrection] = getEphem(satID(j),ephem,timeVec,transTime);
        psr(j) = psr(j) + clockCorrection*c;
        x_ecef(j) = satPosition(1);
        y_ecef(j) = satPosition(2);
        z_ecef(j) = satPosition(3);
        xv_ecef(j) = satVel(1);
        yv_ecef(j) = satVel(2);
        zv_ecef(j) = satVel(3);
        if(~isnan(doppler(j)))
            dop(j) = -dop2speed(doppler(j),.1905);
        end
    end  
    %[9xN] matrix of all satellite measurements
    xVec{i} = x_ecef;
    yVec{i} = y_ecef;
    zVec{i} = z_ecef;
    xVelVec{i} = xv_ecef;
    yVelVec{i} = yv_ecef;
    zVelVec{i} = zv_ecef;
    psrVec{i} = psr;
    carVec{i} = carrier;
    dopVec{i} = dop;
    cnoVec{i} = cno;
end

%% get the user position
[rows,cols] = size(xVec);
initPos = [0,0,0];
initVel = [0,0,0];
initCB = 0;
initCBR = 0;
k = 1;
sigma = 3;
sigma_sq = sigma^2;


for i = 1:(cols)
    satPos = [xVec{i}(:),yVec{i}(:),zVec{i}(:)];
    psrEpoch = psrVec{i}(:);
    cnoEpoch = cnoVec{i}(:);
    dopEpoch = dopVec{i}(:);
    cnoEpoch = cnoVec{i}(:);
    satVel = [xVelVec{i}(:),yVelVec{i}(:),zVelVec{i}(:)];   
    [userPos, userVel, clockBias,biasRate, covar] = getUserPos(satPos,satVel,psrEpoch,dopEpoch,initPos,initVel,initCB,initCBR);
    dopDiag(:,k) = sigma_sq*covar;
    initPos = [userPos(1),userPos(2),userPos(3)];
    initCB = clockBias;
    [wlat, wlon, walt] = wgsxyz2lla([userPos(1),userPos(2),userPos(3)]);
    userLat(k) = wlat;
    userLong(k) = wlon;
    userAlt(k) = walt;
    numSats(k) = length(psrEpoch);
    minCNO(k) = mean(cnoEpoch);
    cb(k) = clockBias;
    cbr(k) = biasRate;
    time(k) = psrTimeIn(i);
    speed(k) = sqrt(userVel(1)^2 + userVel(2)^2 + userVel(3)^2);
    course(k) = atan2(userVel(2),userVel(1));
    k = k+1;
end
    lla = [userLat',userLong', userAlt'];
end

