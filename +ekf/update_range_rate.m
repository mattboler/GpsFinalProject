function [new_X, new_P] = update_range_rate(X, Y, P, R)
%UPDATE_RANGE Summary of this function goes here
%   Update the filter with a measurement of the range between the GPS
%   receivers

% Radar measures distance from radar (on rear truck) to rear of lead truck
% This offset accounts for the distance from rear of lead truck to GPS
% receiver on lead truck as well as distance from radar to GPS receiver on
% rear truck.
RECEIVER_RANGE_OFFSET = 22.324;
Y = Y + RECEIVER_RANGE_OFFSET;

% Compute jacobians
H = H_x(X);

% Other matrices
S = H * P * H' + R;
K = P * H' * inv(S);

% Compute residual
Y_hat = h(X);
z = Y - Y_hat;

% Residual test
d = sqrt( z' * inv(S) * z  );
if d < chi2inv(0.5, length(Y))
    % Kalman Filter correction equation
    new_X = X + K*z;
    new_P = (eye(12) - K*H) * P * (eye(12) - K*H)' + K*R*K';
else
    % Reject measurement
    new_X = X;
    new_P = P;
end

end

function Y = h(X)
V_lead = X(4:6);
V_follow = X(10:12);
dV = V_lead - V_follow;
Y = norm(dV);
end

function H = H_x(X)
% From Jacob Pryor's thesis (Appendix A2, eqs A.48 - A.52)
P_lead = X(1:3);
V_lead = X(4:6);
P_follow = X(7:9);
V_follow = X(10:12);

dP = P_lead - P_follow;
dV = V_lead - V_follow;

U = dP / norm(dP);

N = dV / norm(dP);
D = dot(dP, dP)^(3/2);
B = dot(dP, dV);

L = N - dP .* (B / D);

H = [L', U', -L', -U'];
end
