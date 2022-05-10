function [satPos, satVel, clockCorrection] = getEphem(prn,ephmeris,cTime,transTime);
%GETEPHEM Summary of this function goes here
%   Detailed explanation goes here

    satEphem.omegaE = 7.2921151467e-5;  % Earth rotation rate, [rad/s]
    satEphem.GM = 3.986005e14;      % Earth's universal [m^3/s^2]
    satEphem.Pi = 3.1415926535898;
    satEphem.F = -4.442807633e-10; % Constant, [sec/(meter)^(1/2)]
    %uncorrected orbit elements
    satEphem.a = (ephmeris.ephem((prn)).A);
    satEphem.ecc = ephmeris.ephem((prn)).e;
    satEphem.i_e = ephmeris.ephem((prn)).i_0;
    satEphem.argPerig = ephmeris.ephem((prn)).omega;
    satEphem.meanAnom = ephmeris.ephem((prn)).M_0;
    satEphem.nCorr = ephmeris.ephem((prn)).deltan;
    satEphem.Omega_0 = ephmeris.ephem((prn)).omega_0;
    satEphem.OmegaDot = ephmeris.ephem((prn)).omegaDot;
    satEphem.iDot = ephmeris.ephem((prn)).iDot;
    satEphem.t_gd = ephmeris.ephem((prn)).T_GD;
        
    %mean motion
    n = sqrt(satEphem.GM / (satEphem.a^3));
    
    %corrected mean motion
    satEphem.n = n + satEphem.nCorr;
    
    %correct the argument of latitude
    satEphem.C_us = ephmeris.ephem((prn)).C_us;
    satEphem.C_uc = ephmeris.ephem((prn)).C_uc;
    
    %radius correction
    satEphem.c_rc = ephmeris.ephem((prn)).C_rc;
    satEphem.c_rs = ephmeris.ephem((prn)).C_rs;
    
    %inclination correction
    satEphem.c_ic = ephmeris.ephem((prn)).C_ic;
    satEphem.c_is = ephmeris.ephem((prn)).C_is;
    
    %get clock correction elements
    satEphem.af0 = ephmeris.ephem((prn)).a_f0;
    satEphem.af1 = ephmeris.ephem((prn)).a_f1;
    satEphem.af2 = ephmeris.ephem((prn)).a_f2;
    satEphem.t_oc = ephmeris.ephem((prn)).t_oc;
    satEphem.t_oe = ephmeris.ephem((prn)).t_oe;
    
    %correct the current time
    dt = (cTime - satEphem.t_oc);
    delt_SV = (satEphem.af2*dt + satEphem.af1) * dt + satEphem.af0 - satEphem.t_gd;
    t = cTime - delt_SV;
    t_k = t - satEphem.t_oe;

    %calculate mean anomaly
    satEphem.meanAnom = satEphem.meanAnom + t_k*satEphem.n;
    satEphem.eccAnom = calcAnomaly(satEphem.meanAnom,satEphem.ecc);

    satEphem.eccAnom = rem(satEphem.eccAnom + 2*satEphem.Pi, 2*satEphem.Pi);

    %calculate true anomaly
    satEphem.trueAnom = atan2(sqrt(1 - satEphem.ecc^2) * sin(satEphem.eccAnom), cos(satEphem.eccAnom)-satEphem.ecc);

    %calculate argument of latitude
    satEphem.argLat = satEphem.trueAnom + satEphem.argPerig;

    satEphem.argLat = rem(satEphem.argLat, 2*satEphem.Pi);

    %calculate inclination angle
    iCor = satEphem.c_is*sin(2*satEphem.argLat) + satEphem.c_ic*cos(2*satEphem.argLat);    
    satEphem.i_e = satEphem.i_e + iCor + satEphem.iDot*t_k;

    %correct orbit radius
    rCor = satEphem.c_rs*sin(2*satEphem.argLat) + satEphem.c_rc*cos(2*satEphem.argLat);
    satEphem.rOrbit = satEphem.a*(1 - satEphem.ecc*cos(satEphem.eccAnom)) + rCor;

    %correct the argument of latitude
    satEphem.argLat = satEphem.argLat + satEphem.C_us*sin(2*satEphem.argLat) + satEphem.C_uc*cos(2*satEphem.argLat); 

    %correct longitude of ascending node
    satEphem.Omega_0 = satEphem.Omega_0 + (satEphem.OmegaDot - satEphem.omegaE)*t_k - satEphem.omegaE*satEphem.t_oe - satEphem.omegaE*transTime;
    satEphem.Omega_0 = rem(satEphem.Omega_0 + 2*satEphem.Pi, 2*satEphem.Pi);
    
    %Compute relativistic correction term
    dtr = satEphem.F * satEphem.ecc * sqrt(satEphem.a) * sin(satEphem.eccAnom);
    
    delt_SV = delt_SV + dtr;

    %calculate x/y prime
    x_p = satEphem.rOrbit*cos(satEphem.argLat);
    y_p = satEphem.rOrbit*sin(satEphem.argLat);

    %ecefXYZ
    x_ecef = x_p*cos(satEphem.Omega_0) - (y_p*sin(satEphem.Omega_0)*cos(satEphem.i_e));       
    y_ecef = x_p*sin(satEphem.Omega_0) + (y_p*cos(satEphem.Omega_0)*cos(satEphem.i_e));
    z_ecef = y_p*sin(satEphem.i_e);
    satPos = [x_ecef, y_ecef, z_ecef];
    clockCorrection = delt_SV;
    
%% SV velocity calculations
      
    Edot = (n + satEphem.nCorr)/(1-satEphem.ecc*cos(satEphem.eccAnom));

    phidot = (sqrt(1-satEphem.ecc^2)/(1-satEphem.ecc*cos(satEphem.eccAnom)))*Edot;

    udot = (1+2*satEphem.C_us*cos(2*satEphem.argLat)-2*satEphem.C_uc*sin(2*satEphem.argLat))*phidot;

    rdot = 2*(satEphem.c_rs*cos(2*satEphem.argLat)-satEphem.c_rc*sin(2*satEphem.argLat))*phidot + satEphem.a*satEphem.ecc*sin(satEphem.eccAnom)*Edot;

    idot = 2*(satEphem.c_is*cos(2*satEphem.argLat)-satEphem.c_ic*sin(2*satEphem.argLat))*phidot + satEphem.iDot;

    Xdot = rdot*cos(satEphem.argLat) - satEphem.rOrbit*sin(satEphem.argLat)*udot;

    Ydot = rdot*sin(satEphem.argLat) + satEphem.rOrbit*cos(satEphem.argLat)*udot;

    Omegadot = satEphem.OmegaDot - satEphem.omegaE;

    xdot = Xdot*cos(satEphem.Omega_0) - Ydot*cos(satEphem.i_e)*sin(satEphem.Omega_0) + y_p*sin(satEphem.i_e)*sin(satEphem.Omega_0)*idot - y_ecef*Omegadot;

    ydot = Xdot*sin(satEphem.Omega_0) + Ydot*cos(satEphem.i_e)*cos(satEphem.Omega_0) - y_p*sin(satEphem.i_e)*cos(satEphem.Omega_0)*idot + x_ecef*Omegadot;

    zdot = Ydot*sin(satEphem.i_e) + y_p*cos(satEphem.i_e)*idot;
    
    satVel = [xdot, ydot, zdot];
end

