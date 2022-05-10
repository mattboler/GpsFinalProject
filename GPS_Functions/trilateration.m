function [DOP,bias,userPos] = trilateration(userPos,refPos,satPos,rangesUsr,rangesRef,clockBias,flag)
%TRILATERATION Summary of this function goes here
%   Detailed explanation goes here

userPos = [0,0];
speedLight = 299792458;
clockBias = clockBias*speedLight;

%run 4.a for flag 1
if flag == 1
    converged = 0;
    iters = 1;
    while converged == 0
        [G,pseudoHat] = genGeometryMat_noB(userPos,satPos);
        y_psr = pseudoHat - rangesUsr;
        [correction,DOP] = least_squares(G,y_psr);

        userPos(1) = userPos(1) + correction(1);
        userPos(2) = userPos(2) + correction(2);

        if(sqrt(correction(1)^2 + correction(2)^2) < 1.0)
            converged = 1;
            bias = clockBias / speedLight;
        end
        iters = iters + 1;

        if(iters > 50)
            warning('Bad Geometry Detected in Trilateration')
            converged = 1;
        end
    end
end

%run 4.b for flag 2
if flag == 2
    converged = 0;
    iters = 1;
    while converged == 0
        [G,pseudoHat] = genGeometryMat(userPos,satPos,clockBias);
        y_psr = pseudoHat - rangesUsr;
        [correction,DOP] = least_squares(G,y_psr);

        userPos(1) = userPos(1) + correction(1);
        userPos(2) = userPos(2) + correction(2);
        clockBias = clockBias + correction(3);

        if(sqrt(correction(1)^2 + correction(2)^2) < 1.0)
            converged = 1;
            bias = clockBias / speedLight;
        end
        iters = iters + 1;

        if(iters > 50)
            warning('Bad Geometry Detected in Trilateration')
            converged = 1;
        end
    end
end

%run 4.c for flag c
if flag == 3
        G = zeros(4,3);
        for j = 1:4
            G(j,1) = (satPos(1,j) - refPos(1)) / rangesRef(j);
            G(j,2) = (satPos(2,j) - refPos(1)) / rangesRef(j);
            G(j,3) = 1;
        end
        y_psr = rangesRef - rangesUsr;
        [diffPos,DOP] = least_squares(G,y_psr);
        userPos = diffPos(1:2);
        bias = diffPos(3);
end


    function [G,pseudoHat] = genGeometryMat_noB(userPos,satPos)
        pseudoHat = zeros(1,length(satPos));
        G = ones(length(satPos),2);
        for i = 1:length(satPos)
            range = sqrt((satPos(1,i) - userPos(1))^2 + (satPos(2,i) - userPos(2))^2);
            pseudoHat(i) = range;
            G(i,1) = (satPos(1,i) - userPos(1)) / range;
            G(i,2) = (satPos(2,i) - userPos(2)) / range;
        end
    end

    function [G,pseudoHat] = genGeometryMat(userPos,satPos,clockBias)
        pseudoHat = zeros(1,length(satPos));
        G = ones(length(satPos),3);
        for i = 1:length(satPos)
            range = sqrt((satPos(1,i) - userPos(1))^2 + (satPos(2,i) - userPos(2))^2);
            pseudoHat(i) = range + clockBias;
            G(i,1) = (satPos(1,i) - userPos(1)) / range;
            G(i,2) = (satPos(2,i) - userPos(2)) / range;
            G(i,3) = 1;
        end
    end
end

