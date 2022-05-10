clc; clear all; close all

addpath './data/2tRear' './data/BRDM00DLR_S_20201890000_01D_MN.rnx' './GPS_Functions'

a1 = load('T13_2T_100_1_2020-07-07-16-54-12');

a2 = load('A2_2T_100_1_2020-07-07-17-10-29');

ephem = readMultiGNSSRinex('BRDM00DLR_S_20201890000_01D_MN.rnx',2.646982e5);

ephem.ephem = ephem.gps;

%% get reference gps time
a1GpsTime = a1.data.novatel_local.gpsTimeTagged.gpsSeconds;
a2GpsTime = a2.data.novatel_local.gpsTimeTagged.gpsSeconds;

%% get A1/A2 pseudoRanges
a1PseudoRanges = a1.data.novatel_local.rawMeasurementsTagged.pseudorange;
a1cno = a1.data.novatel_local.rawMeasurementsTagged.cno;
a1Carrier = a1.data.novatel_local.rawMeasurementsTagged.carrierPhase;
a1Dop = a1.data.novatel_local.rawMeasurementsTagged.doppler;
a1Prn = a1.data.novatel_local.rawMeasurementsTagged.prn;

a2PseudoRanges = a2.data.novatel_local.rawMeasurementsTagged.pseudorange;
a2cno = a2.data.novatel_local.rawMeasurementsTagged.cno;
a2Carrier = a2.data.novatel_local.rawMeasurementsTagged.carrierPhase;
a2Dop = a2.data.novatel_local.rawMeasurementsTagged.doppler;
a2Prn = a2.data.novatel_local.rawMeasurementsTagged.prn;

a1PsrTime = linspace(a1GpsTime(1),a1GpsTime(length(a1GpsTime)),length(a1PseudoRanges));
a2PsrTime = linspace(a2GpsTime(1),a2GpsTime(length(a2GpsTime)),length(a2PseudoRanges));

[a1PseudoRanges] = alignData(a1PseudoRanges,a1PsrTime,a1GpsTime,a2GpsTime);
[a1cno] = alignData(a1cno,a1PsrTime,a1GpsTime,a2GpsTime);
[a1Carrier] = alignData(a1Carrier,a1PsrTime,a1GpsTime,a2GpsTime);
[a1Dop] = alignData(a1Dop,a1PsrTime,a1GpsTime,a2GpsTime);
[a1Prn] = alignData(double(a1Prn),a1PsrTime,a1GpsTime,a2GpsTime);
[a1Prn] = recoverPRN(a1Prn);

a2PseudoRanges = reSampleData(a2PsrTime,a2PseudoRanges,a2GpsTime);
a2cno = reSampleData(a2PsrTime,a2cno,a2GpsTime);
a2Carrier = reSampleData(a2PsrTime,a2Carrier,a2GpsTime);
a2Dop = reSampleData(a2PsrTime,a2Dop,a2GpsTime);
a2Prn = reSampleData(a2PsrTime,double(a2Prn),a2GpsTime);
a2Prn = recoverPRN(a2Prn);



%% get Radar Data
%get the range estimation radar tracks
radarTracks = a2.data.range_estimator.rangeEstimationOutput.radarTrackId;

%get radar data
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

%% Get user position and clock bias
[timeA1, llaA1,cbA1,cbrA1,dopDiagA1] = processGpsData(a2GpsTime,a1PseudoRanges,a1Dop,a1Carrier,a1cno,a1Prn,ephem);
[timeA2, llaA2,cbA2,cbrA2,dopDiagA2] = processGpsData(a2GpsTime,a2PseudoRanges,a2Dop,a2Carrier,a2cno,a2Prn,ephem);

%% Plot
figure(2)
geoscatter(llaA1(:,1), llaA1(:,2),'Marker', '.')
hold on
geoscatter(llaA2(:,1), llaA2(:,2),'Marker', '.')
geobasemap('satellite')

deltaCB = cbA1 - cbA2;

a1XYZ = [];
a2XYZ = [];
RPV = [];
rpvNorm = [];

for i = 1:length(a2GpsTime)
a1Ecef = lla2ecef([llaA1(i,1) llaA1(i,2) 0]);
a1XYZ = [a1XYZ;a1Ecef];

a2Ecef = lla2ecef([llaA2(i,1) llaA2(i,2) 0]);
a2XYZ = [a2XYZ;a2Ecef];

RPV = [RPV;(a1Ecef-a2Ecef)];
rpvNorm(i) = norm(a1Ecef-a2Ecef);
end

figure(3)
plot(a2GpsTime,deltaCB)
xlabel('Time [s]')
ylabel('Delta CB')

figure(4)
plot(a2GpsTime - a2GpsTime(1),rpvNorm,'LineWidth',2.25)
hold on
plot(drtkTime,drtkRPV,'LineWidth',2.25)
legend('Downsampled RPV','DRTK')
ylim([0, 150])

% figure(2)
% plot(time - time(1),cb)






