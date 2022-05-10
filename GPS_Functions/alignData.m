function [newData] = alignData(inputData,inputTime,inputGpsTime,refGpsTime)
%ALIGNDATA Summary of this function goes here
%   Detailed explanation goes here

[rows,cols] = size(inputData);
newDownSampleData = [];
for i = 1:cols
    ds = interp1(inputTime,inputData(:,i),inputGpsTime,'next');
    newDownSampleData = [newDownSampleData,ds'];
end

%align the downsampled data to GPS time
newData = [];
for i = 1:cols
    [output] = alignGPS(newDownSampleData(:,i),inputGpsTime,refGpsTime);
    newData = [newData,output];
end
end

