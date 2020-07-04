
% Speed of light
c = 3 * 1e8;

R_max = 300;

% fb: [0 MHz, 1.1 MHz, 13 MHz, 24 MHz]
% The radar maximum range = 300m
% The range resolution = 1m
% The speed of light c = 3*10^8
% T_chirp = 5.5 * 2 * R_max / c

% d_res = c / (2 * Bsweep)

d_res = 1;

% TODO : Find the Bsweep of chirp for 1 m resolution
% Ans: 150 MHz
Bsweep = c / (2 * d_res);

% TODO : Calculate the chirp time based on the Radar's Max Range
T_chirp = 5.5 * 2 * R_max / c;

% TODO : define the frequency shifts 
fb = [0 1.1 13 24] * 1e6;

calculated_range = c * T_chirp * fb / (2 * Bsweep);

% Display the calculated range
disp(calculated_range);