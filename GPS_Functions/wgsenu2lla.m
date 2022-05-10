% Function enu = wgslla2enu(lat,lon,alt,reflat,reflon,refalt)
% returns a 3 x 1 vector enu representing the East, North, and Up
% coordinates (in meters) of a point with coordinates represented
% by latitude lat (degrees), longitude lon (degrees), and altitude
% alt (meters above the ellipsoid) in an ENU coordinate system
% located at latitude reflat (degrees), longitude reflon (degrees)
% and altitude above the WGS84 ellipsoid refalt (meters)
%
% Note: requires the functions wgslla2xyz.m and wgsxyz2enu.m
% to be in the same directory

function lla = wgsenu2lla(enu, reflat, reflon, refalt)

% ned to xyz
refxyz = wgslla2xyz(reflat, reflon, refalt);

% Now rotate the (often short) diffxyz vector to enu frame

R1=rot(90+reflon, 3);
R2=rot(90-reflat, 1);
R=R2*R1;

diffxyz = R'*enu;



% Difference xyz from reference point
xyz = diffxyz + refxyz;

[lat,long,alt]=wgsxyz2lla(xyz);

lla = [lat;long;alt];

end


% 
% xyz = wgslla2xyz(lat, lon, alt);
% enu = wgsxyz2enu(xyz, reflat, reflon, refalt);
% 
% ned = [enu(2),enu(1),-enu(3)]';
% 
