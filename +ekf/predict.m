function [new_X, new_P] = predict(X, P, sigma_q, dt)
%PREDICT Summary of this function goes here
%   Detailed explanation goes here

% See Zarchan p135 for derivations

% Discrete state transition matrix
Phi = [1, dt; ...
       0, 1];
A_truck = kron(Phi, eye(3));
A_convoy = blkdiag(A_truck, A_truck);

% Discrete noise matrix
M = [(1/3)*dt^3, (1/2)*dt^2; ...
     (1/2)*dt^2, dt];
Q = M * sigma_q^2;
Q_truck = kron(Q, eye(3));
Q_convoy = blkdiag(Q_truck, Q_truck);

% Kalman Filter equations
new_X = A_convoy * X;
new_P = A_convoy * P * A_convoy' + Q_convoy;
end