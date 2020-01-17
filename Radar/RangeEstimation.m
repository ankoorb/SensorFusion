% calculate the range in meters of four targets with respective measured 
% beat frequencies [0 MHz, 1.1 MHz, 13 MHz, 24 MHz]
% The radar maximum range = 300m, The range resolution = 1m, 
% The speed of light c = 3*10^8


% TODO : Find the Bsweep of chirp for 1 m resolution
Dres = 1.0;  % Range resolution
c = 3 * 10^8; % Speed of light
Bsweep = c / (2 * Dres);
disp(Bsweep);


% TODO : Calculate the chirp time based on the Radar's Max Range
Rmax = 300;
Tchirp = 5.5 * 2 * Rmax / c;
disp(Tchirp);

% TODO : define the frequency shifts
Fbeat = [0, 1.1e6, 13e6, 24e6];
calculated_range = c * Tchirp * Fbeat / (2 * Bsweep);

% Display the calculated range
disp(calculated_range);