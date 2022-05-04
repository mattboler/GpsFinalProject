function [new_X, new_P] = update_GPS_follow(X, Y, P, R)
%UPDATE_GPS_FOLLOW Summary of this function goes here
%   Update the filter with a measurement of the position of the following
%   truck

H = kron([0, 0, 1, 0], eye(3));
S = H * P * H' + R;
K = P * H' * inv(S);
z = Y - H*X;

% Residual test
d = sqrt( z' * inv(S) * z  );
if d < chi2inv(0.9, length(Y))
    % Kalman Filter correction equation
    new_X = X + K*z;
    new_P = (eye(12) - K*H) * P * (eye(12) - K*H)' + K*R*K';
else
    % Reject measurement
    new_X = X;
    new_P = P;
end

end

