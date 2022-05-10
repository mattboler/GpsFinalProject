function [radarTracks] = getRadarTracks(data,relativeWheelSpeed)
%GETRADARTRACKS Summary of this function goes here
%   Detailed explanation goes here
radarTracks = [];

for i = 1:length(data)
    if(abs(data(i) - relativeWheelSpeed) < .01)
        radarTracks = [radarTracks,i];
    end
end

end


