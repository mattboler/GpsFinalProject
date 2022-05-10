function [estimate,covariance] = WLS(Hmat,measurementMat,cno)
%Weighted LEAST_SQUARES [estimate,covariance] = least_squares(Hmat,measurementMat,weights)
W = diag(1./(2*cno).^2);
W = [W zeros(size(W));zeros(size(W)) W];
[rows_h,cols_h] = size(Hmat);
[rows_y,cols_y] = size(measurementMat);
if(rows_h ~= rows_y && rows_h ~= cols_y)
    error('matrix dimensions must agree')
end

if(rows_h ~= rows_y)
    estimate = pinv(Hmat'*W*Hmat)*Hmat'*W*measurementMat';
else
    estimate = pinv(Hmat'*W*Hmat)*Hmat'*W*measurementMat;
end

covariance = pinv(Hmat'*W*Hmat);

end

