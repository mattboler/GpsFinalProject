function [newData] = reSampleData(inputTime,inputData,refTime)
%DOWNSAMPLEDATA Summary of this function goes here
%   Detailed explanation goes here

[rows,cols] = size(inputData);
newData = [];
for i = 1:cols
    ds = interp1(inputTime,inputData(:,i),refTime,'next');
    newData = [newData,ds'];
end
end

