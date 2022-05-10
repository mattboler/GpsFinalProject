%{
Example script
%}

clc; clear all; close all;

addpath(genpath('data/'))
addpath(genpath('utils/'))
addpath(genpath('GPS_FUNCTIONS/'))
import ekf.*;

%% Tuning Parameters

% Standard deviation of process noise
SIGMA_Q = 1;

% Standard deviation of GPS position measurement
SIGMA_POS = 3;
SIGMA_VEL = 0.5;

% Standard deviation of radar range measurement
SIGMA_RANGE = 0.5;
% Standard deviation of radar range rate measurement
SIGMA_RANGE_RATE = 0.01;

%% Data Loading
a1 = load('controller_eval_3_2020-06-22-15-13-03.mat');
a2 = load('controller evaluation 3_2020-06-22-15-14-01.mat');

% get reference gps time
a1GpsTime = a1.data.novatel_local.gpsTimeTagged.gpsSeconds;
a2GpsTime = a2.data.novatel_local.gpsTimeTagged.gpsSeconds;

% get A1/A2 solutions
a1GpsPosition = a1.data.novatel_local.odom.positionEcef;
a1GpsVelocity = a1.data.novatel_local.odom.velocityEcef;

a2GpsPosition = a2.data.novatel_local.odom.positionEcef;
a2GpsVelocity = a2.data.novatel_local.odom.velocityEcef;

% get A1/A2 pseudoRanges
a1PseudoRanges = a1.data.novatel_local.rawMeasurementsTagged.pseudorange;
a1cno = a1.data.novatel_local.rawMeasurementsTagged.cno;
a1Carrier = a1.data.novatel_local.rawMeasurementsTagged.carrierPhase;
a1Prn = a1.data.novatel_local.rawMeasurementsTagged.prn;

a2PseudoRanges = a2.data.novatel_local.rawMeasurementsTagged.pseudorange;
a2cno = a2.data.novatel_local.rawMeasurementsTagged.cno;
a2Carrier = a2.data.novatel_local.rawMeasurementsTagged.carrierPhase;
a2Prn = a2.data.novatel_local.rawMeasurementsTagged.prn;

a1PsrTime = linspace(a1GpsTime(1),a1GpsTime(length(a1GpsTime)),length(a1PseudoRanges));
a2PsrTime = linspace(a2GpsTime(1),a2GpsTime(length(a2GpsTime)),length(a2PseudoRanges));

[a1PseudoRanges] = alignData(a1PseudoRanges,a1PsrTime,a1GpsTime,a2GpsTime);
[a1cno] = alignData(a1cno,a1PsrTime,a1GpsTime,a2GpsTime);
[a1Carrier] = alignData(a1Carrier,a1PsrTime,a1GpsTime,a2GpsTime);
[a1Prn] = alignData(double(a1Prn),a1PsrTime,a1GpsTime,a2GpsTime);
[a1Prn] = recoverPRN(a1Prn);

a2PseudoRanges = reSampleData(a2PsrTime,a2PseudoRanges,a2GpsTime);
a2cno = reSampleData(a2PsrTime,a2cno,a2GpsTime);
a2Carrier = reSampleData(a2PsrTime,a2Carrier,a2GpsTime);
a2Prn = reSampleData(a2PsrTime,double(a2Prn),a2GpsTime);
a2Prn = recoverPRN(a2Prn);

% get Radar Data
% get the range estimation radar tracks
radarTracks = a2.data.range_estimator.rangeEstimationOutput.radarTrackId;

% get radar data
delphiRanges = a2.data.delphi.data.range;
delphiRRs = a2.data.delphi.data.rangeRate;
radarTime = a2.data.delphi.data.time;

offsetIdx = length(delphiRanges) - length(radarTracks);

j = 1;
for i = offsetIdx:length(radarTracks)
    if radarTracks(j) > 0
        delphiRange(j) = delphiRanges(radarTracks(j),i);
        delphiRR(j) = delphiRRs(radarTracks(j),i);
        delphiMeasTime(j) = radarTime(i);
        j = j+1;
    else
        j = j+1;
    end
end
delphiMeasTime = delphiMeasTime + a2GpsTime(1);

% get DRTK Data
drtkRPV = a2.data.drtk.rpvNorm;
drtkTime = a2.data.drtk.time;
drtkIntegers = a2.data.drtk.basePrn;

% Get range estimation data
rangeEstimate = a2.data.range_estimator.rangeEstimationOutput.range;
rrEstimate = a2.data.range_estimator.rangeEstimationOutput.rangeRate;
estimateTime = a2.data.range_estimator.rangeEstimationOutput.time;

% Bookkeeping fun!
A1_KEY = 0;
a1GpsIndeces = 1 : length(a1GpsTime);
a1GpsIndicators = A1_KEY * ones(size(a1GpsTime));

A2_KEY = 1;
a2GpsIndeces = 1 : length(a2GpsTime);
a2GpsIndicators = A2_KEY * ones(size(a2GpsIndeces));

DELPHI_KEY = 2;
delphiIndeces = 1 : length(delphiMeasTime);
delphiIndicators = DELPHI_KEY * ones(size(delphiRange));

MEASUREMENT_TIMES = [a1GpsTime, a2GpsTime, delphiMeasTime];
MEASUREMENT_INDECES = [a1GpsIndeces, a2GpsIndeces, delphiIndeces];
MEASUREMENT_INDICATORS = [a1GpsIndicators, a2GpsIndicators, delphiIndicators];

[MEASUREMENT_TIMES, IDX] = sort(MEASUREMENT_TIMES);
MEASUREMENT_INDECES = MEASUREMENT_INDECES(IDX);
MEASUREMENT_INDICATORS = MEASUREMENT_INDICATORS(IDX);

%% Run

init_lead = false;
init_follow = false;
is_initialized = false;
t = 0;

times = [];
leader_positions = [];
leader_position_stds = [];

leader_velocities = [];
leader_velocity_stds = [];

follower_positions = [];
follower_position_stds = [];

follower_velocities = [];
follower_velocity_stds = [];

X = zeros(12,1);
P = diag(kron([SIGMA_POS, SIGMA_VEL, SIGMA_POS, SIGMA_VEL].^2, [1,1,1])) * 1;

A1_COUNT = 0;
A2_COUNT = 0;
DELPHI_COUNT = 0;

for i = 1 : length(MEASUREMENT_TIMES)
    % Grab measurement
    dt = MEASUREMENT_TIMES(i) - t;
    flag = MEASUREMENT_INDICATORS(i);
    idx = MEASUREMENT_INDECES(i);

    t = t + dt;

    if ~is_initialized
        % Don't run filter until we have actual states for both trucks
        if flag == A1_KEY
            A1_COUNT = A1_COUNT + 1;
            position = a1GpsPosition(:, idx);
            velocity = a1GpsVelocity(:, idx);
            X(1:6) = [position; velocity];
            if init_lead == false
                disp(['Initialized leader at ', num2str(t)])
            end
            init_lead = true;
        elseif flag == A2_KEY
            A2_COUNT = A2_COUNT + 1;
            position = a2GpsPosition(:, idx);
            velocity = a1GpsVelocity(:, idx);
            X(7:12) = [position; velocity];
            if init_follow == false
                disp(['Initialized follower at ', num2str(t)])
            end
            init_follow = true;
        elseif flag == DELPHI_KEY
            DELPHI_COUNT = DELPHI_COUNT + 1;
        end

        if init_lead == true && init_follow == true
            is_initialized = true;
            disp(['Initialized at t = ', num2str(t)])
        end
        continue
    else
        % Filter!
        [X, P] = ekf.predict(X, P, SIGMA_Q, dt);

        if flag == A1_KEY
            A1_COUNT = A1_COUNT + 1;
            Y = a1GpsPosition(:, idx);
            [X, P] = ekf.update_GPS_lead(X, Y, P, eye(3) * SIGMA_POS^2);
        elseif flag == A2_KEY
            A2_COUNT = A2_COUNT + 1;
            Y = a2GpsPosition(:, idx);
            [X, P] = ekf.update_GPS_follow(X, Y, P, eye(3) * SIGMA_POS^2);
        elseif flag == DELPHI_KEY
            % TODO: merge range and range-rate updates
            DELPHI_COUNT = DELPHI_COUNT + 1;
            Y = delphiRange(idx);
            [X, P] = ekf.update_range(X, Y, P, eye(1) * SIGMA_RANGE^2);
            %Y = delphiRR(idx);
            %[X, P] = ekf.update_range_rate(X, Y, P, eye(1) * SIGMA_RANGE_RATE^2);
        end
    end
    % Log results
    times(end+1) = t;
    leader_positions(:, end+1) = X(1:3);
    leader_velocities(:, end+1) = X(4:6);
    follower_positions(:, end+1) = X(7:9);
    follower_velocities(:, end+1) = X(10:12);

    % Check if we're done
    if A1_COUNT == length(a1GpsTime)
        break
    elseif A2_COUNT == length(a2GpsTime)
        break
    elseif DELPHI_COUNT == length(delphiMeasTime)
        break
    end
end

delta_positions_ecef = leader_positions - follower_positions;
ranges = vecnorm(delta_positions_ecef, 2, 1);

a1GpsPosition_llh = ecef2lla(a1GpsPosition');
a2GpsPosition_llh = ecef2lla(a2GpsPosition');

%% Data Wrangling

RANGE_OFFSET = 21.85;

estimateTime = estimateTime + a2GpsTime(1);
drtkTime = drtkTime + a2GpsTime(1);

ESTIMATE_IDX = find(estimateTime > times(1));
DRTK_IDX = find(drtkTime > times(1));
DELPHI_IDX = find(delphiMeasTime > times(1));

drtkTime = drtkTime(DRTK_IDX);
drtkRPV = drtkRPV(DRTK_IDX);

estimateTime = estimateTime(ESTIMATE_IDX);
rangeEstimate = rangeEstimate(ESTIMATE_IDX) + RANGE_OFFSET;

delphiMeasTime = delphiMeasTime(DELPHI_IDX);
delphiRange = delphiRange(DELPHI_IDX) + RANGE_OFFSET;




%% Plotting

figure()
plot(times, ranges)
hold on
plot(estimateTime, rangeEstimate)
plot(drtkTime, drtkRPV, '.')
ylim([20, 100])
legend("Estimated Ranges (Ours)", "Estimated Ranges (Trucks)", "DRTK Ranges")
title("Range Estimates")
xlabel("Time (s)")
ylabel("Range (m)")

leader_positions_llh = ecef2lla(leader_positions');
follower_positions_llh = ecef2lla(follower_positions');

figure()
subplot(2,1,1)
plot(times, leader_positions_llh(:,1))
hold on
plot(times, follower_positions_llh(:,1))
title("Latitude")
legend("Leader", "Follower")

subplot(2,1,2)
plot(times, leader_positions_llh(:,2))
hold on
plot(times, follower_positions_llh(:,2))
title("Longitude")
legend("Leader", "Follower")

figure()
geoscatter(leader_positions_llh(:, 1), leader_positions_llh(:, 2), 'Marker', '.');
hold on
geoscatter(follower_positions_llh(:, 1), follower_positions_llh(:, 2), 'Marker', '.');
legend("Leader (EKF)", "Follower (EKF)")
title("Estimated Positions")
geobasemap satellite;

figure()
geoscatter(leader_positions_llh(:, 1), leader_positions_llh(:, 2), 'Marker', '.');
hold on
geoscatter(a1GpsPosition_llh(:, 1), a1GpsPosition_llh(:, 2), 'Marker', '.')
legend("EKF", "Novatel")
title("Estimated Leader Positions")
geobasemap satellite

figure()
geoscatter(follower_positions_llh(:, 1), follower_positions_llh(:, 2), 'Marker', '.');
hold on
geoscatter(a2GpsPosition_llh(:, 1), a2GpsPosition_llh(:, 2), 'Marker', '.')
legend("EKF", "Novatel")
title("Estimated Follower Positions")
geobasemap satellite

