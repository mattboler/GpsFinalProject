function [signal] = upSample(inSig,sampleRatio)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
signal = zeros(1,length(inSig*sampleRatio));
j = 1;
i = 1;
k = 1;
for i = 1:length(signal)
    for j = 1:sampleRatio
        signal(k) = inSig(i);
        k = k+1;
    end
end


end