function [newSatPos,newSatVel,newPsrVec,newDopVec,newCarVec] = remove_nans(satPos,satVel,psrVec,dopVec,carVec)
%REMOVE_NANS Summary of this function goes here
%   Detailed explanation goes here
newSatPos = [];
newSatVel = [];
newPsrVec = [];
newDopVec = [];
newCarVec = [];

for i = 1:length(dopVec)
    if(~isnan(satPos(i,1)) && ~isnan(dopVec(i)) && ~isnan(psrVec(i)))
        newSatPos = [newSatPos;satPos(i,:)];
        newSatVel = [newSatVel;satVel(i,:)];
        newPsrVec = [newPsrVec;psrVec(i)];
        newDopVec = [newDopVec;dopVec(i)];
        newCarVec = [newCarVec;carVec(i)];
    end
    
end

end

