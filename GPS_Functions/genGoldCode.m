function goldCode = genGoldCode(prn)
%GENGOLDCODE Summary of this function goes here
%   Detailed explanation goes here

satPrnVec = [1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19;
             2 3 4 5 1 2 1 2 3 2 3 5 6 7 8 9 1 2 3;
             6 7 8 9 9 10 8 9 10 3 4 6 7 8 9 10 4 5 6];

phaseSelector_1 = satPrnVec(2,prn);
phaseSelector_2 = satPrnVec(3,prn);

%define consants
total_bits = 1023;
n = 1;
goldCode(n) = 1;

% Generate the initial matrices
G1 = ones(1,10);
G2 = ones(1,10);

% Calculate the new bit value for G1 vector
while length(goldCode) < total_bits
    
% G1 xor calculator
if G1(3) + G1(10) == 1
    g1_new = 1;
else
    g1_new = 0;
end

% Calculate the new bit value for G2 Vector

% G2 xor Calculator for 10-9
if G2(9) + G2(10) == 1
    g2_109 = 1;
else
    g2_109 = 0;
end

% G2 xor Calculator for 98
if g2_109 + G2(8) == 1
    g2_98 = 1;
else
    g2_98 = 0;
end

% G2 xor Calculator for 86
if g2_98 + G2(6) == 1
    g2_86 = 1;
else
    g2_86 = 0;
end

% G2 xor Calculator for 63
if g2_86 + G2(3) == 1
    g2_63 = 1;
else
    g2_63 = 0;
end

% G2 xor Calculator for 32
if g2_63 + G2(2) == 1
    g2_new = 1;
else
    g2_new = 0;
end

% Phase selector XOR gate

if G2(phaseSelector_1) + G2(phaseSelector_2) == 1
    Phase_selector = 1;
else
    Phase_selector = 0;
end


% Gold Code Xor Gate

if G1(10) + Phase_selector == 1
    gc = 1;
else
    gc = 0;
end

goldCode(n) = gc;

% Create new G1 & G2 vectors

G1 = [g1_new G1(1:9)];

G2 = [g2_new G2(1:9)];

n = n+1;
end
end

