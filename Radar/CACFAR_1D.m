% Implement 1D CFAR using lagging cells on the given noise and target scenario.

% Close and delete all currently open figures
clc;
close all;

% Data_points
Ns = 1000;  % # of samples for which we want to run CFAR

% Generate random noise
s = abs(randn(Ns, 1));  % generate random noise for the same amount

%Targets location. Assigning bin 100, 200, 300 and 700 as Targets with the amplitudes of 8, 9, 4, 11.
s([100, 200, 350, 700]) = [8, 15, 7, 13];

%plot the output
plot(s);

% TODO: Apply CFAR to detect the targets by filtering the noise.

% 1. Define the following:
% 1a. Training Cells
T = 12;

% 1b. Guard Cells 
G = 4;

% Offset : Adding room above noise threshold for desired SNR 
offset = 3;

% Vector to hold threshold values 
threshold_cfar = [];

% Vector to hold final signal after thresholding
signal_cfar = [];

% 2. Slide window across the signal length
for i = 1:(Ns-(G+T))     

    % 2. - 5. Determine the noise threshold by measuring it within the training cells
    
    % For each step add the noise within all the training cells
    noise_level = sum(s(i:i+T-1));
    
    % To determine threshold take the average of summed noise and multiply
    % it with the offset
    threshold = (noise_level/T) * offset;
    threshold_cfar = [threshold_cfar, {threshold}];

    % 6. Measuring the signal within the CUT
    % Pick CUT which is T+G away from the first training cell and measure
    % signal level
    signal = s(i+T+G);

    % 8. Filter the signal above the threshold
    % If the signal level at CUT is below threshold
    if (signal < threshold)
        signal = 0;
    end

    signal_cfar = [signal_cfar, {signal}];
    
end


% plot the filtered signal
plot (cell2mat(signal_cfar), 'g--');

% plot original sig, threshold and filtered signal within the same figure.
figure, plot(s);
hold on, plot(cell2mat(circshift(threshold_cfar, G)), 'r--', 'LineWidth', 2)
hold on, plot (cell2mat(circshift(signal_cfar, (T+G))), 'g--', 'LineWidth', 3);
legend('Signal','CFAR Threshold','detection')

