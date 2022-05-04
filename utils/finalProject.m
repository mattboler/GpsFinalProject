clc; clear all; close all

addpath '../data/A1' '../data/A2'

a1 = load('controller_eval_3_2020-06-22-15-13-03');

a2 = load('controller evaluation 3_2020-06-22-15-14-01');

%% get reference gps time
a1GpsTime = a1.data.novatel_local.gpsTimeTagged.gpsSeconds;
a2GpsTime = a2.data.novatel_local.gpsTimeTagged.gpsSeconds;

%% get A1/A2 pseudoRanges
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



%% get Radar Data
%get the range estimation radar tracks
radarTracks = a2.data.range_estimator.rangeEstimationOutput.radarTrackId;

%get radar data
delphiRanges = a2.data.delphi.data.range;

delphiRRs = a2.data.delphi.data.rangeRate;

radarTime = a2.data.delphi.data.time;%linspace(a2GpsTime(1),a2GpsTime(length(a2GpsTime)),length(delphiRanges));

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

%% get DRTK Data

drtkRPV = a2.data.drtk.rpvNorm;
drtkTime = a2.data.drtk.time;
drtkIntegers = a2.data.drtk.basePrn;

%% Get range estimation data

rangeEstimate = a2.data.range_estimator.rangeEstimationOutput.range;
rrEstimate = a2.data.range_estimator.rangeEstimationOutput.rangeRate;
estimateTime = a2.data.range_estimator.rangeEstimationOutput.time;

%% plot
drtkOffset = 22.3240;
figure(1)
plot(drtkTime,drtkRPV - drtkOffset,'LineWidth',2.25)
hold on
plot(estimateTime - estimateTime(1),rangeEstimate,'LineWidth',2.25)
hold on
scatter(delphiMeasTime,delphiRange,'.')
legend('drtk','range estimate','DelphiRaw')

figure(2)
plot(estimateTime - estimateTime(1), rrEstimate, 'LineWidth', 2.25)
hold on
scatter(delphiMeasTime, delphiRR, '.')
legend('Estimate', 'Delphi')
title("Range Rates")


%% get the wheel speed data

a1WS = a1.data.j1939.vehicle_speed.wheelBasedSpeed*0.277778;

a2WS = a2.data.j1939.vehicle_speed.wheelBasedSpeed*0.277778;

a1WsTime = linspace(a1GpsTime(1),a1GpsTime(length(a1GpsTime)),length(a1WS));
a2WsTime = linspace(a2GpsTime(1),a2GpsTime(length(a2GpsTime)),length(a2WS));

a1WS = alignData(a1WS',a1WsTime,a1GpsTime,a2GpsTime);

a2WS = reSampleData(a2WsTime',a2WS',a2GpsTime);

rangeRate = a1WS - a2WS;

