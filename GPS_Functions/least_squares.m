function [estimate,covariance] = least_squares(Hmat,measurementMat)
%LEAST_SQUARES [estimate,covariance] = least_squares(Hmat,measurementMat)
[rows_h,cols_h] = size(Hmat);
[rows_y,cols_y] = size(measurementMat);
if(rows_h ~= rows_y && rows_h ~= cols_y)
    error('matrix dimensions must agree')
end

if(rows_h ~= rows_y)
    estimate = pinv(Hmat'*Hmat)*Hmat'*measurementMat';
else
    estimate = pinv(Hmat'*Hmat)*Hmat'*measurementMat;
end

covariance = pinv(Hmat'*Hmat);

end

