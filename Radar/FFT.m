Fs = 1000;            % Sampling frequency, Hz              
T = 1/Fs;             % Sampling period, sec       
L = 1500;             % Length of signal
t = (0:L-1)*T;        % Time vector
N = 128;

% TODO: Form a signal containing a 77 Hz sinusoid of amplitude 0.7 and a 43Hz sinusoid of amplitude 2.
S = 0.7 * sin(2 * pi * 77 * t) +  2 * sin(2 * pi * 43 * t);

% Corrupt the signal with noise 
X = S + 2*randn(size(t));

% Plot the noisy signal in the time domain. It is difficult to identify the frequency components by looking at the signal X(t). 
plot(1000*t(1:50), X(1:50))
title('Signal Corrupted with Zero-Mean Random Noise')
xlabel('t (milliseconds)')
ylabel('X(t)')

% TODO : Compute the Fourier transform of the noise corrupted signal. 
Y = fft(X);

% Take the amplitude of the normalized signal
P2 = abs(Y/L);

% Compute the single sided spectrum, i.e. reject the mirror image
P1 = P2(1:L/2+1);

% Plotting
f = Fs*(0:(L/2))/L;
plot(f,P1) 
title('Single-Sided Amplitude Spectrum of X(t)')
xlabel('f (Hz)')
ylabel('|P1(f)|')

