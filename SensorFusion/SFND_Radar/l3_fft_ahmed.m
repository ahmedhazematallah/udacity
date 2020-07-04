Fs = 1000;            % Sampling frequency                    
T = 1/Fs;             % Sampling period       
L = 1500;             % Length of signal (in samples)
t = (0:L-1)*T;        % Time vector (vector of time instances)

% TODO: Form a signal containing a 150 Hz sinusoid of amplitude 0.7 and a 43Hz sinusoid of amplitude 2.
S = 3 * sin(2*pi*150*t) % + 2 * sin(2*pi*43*t);

% Corrupt the signal with noise 
%X = S + 2*randn(size(t));

% Plot the noisy signal in the time domain. It is difficult to identify the frequency components by looking at the signal X(t). 
plot(1000*t(1:50) ,S(1:50))
title('Signal')
xlabel('t (milliseconds)')
ylabel('S(t)')

figure

% TODO : Compute the Fourier transform of the signal. 
signal_fft = fft(S);

% TODO : Compute the two-sided spectrum P2. Then compute the single-sided spectrum P1 based on P2 and the even-valued signal length L.
P2 = 2* abs(signal_fft / L);

% Take only the positive part
P1 = P2(1:L/2+1)

% Plotting
f = Fs*(0:(L/2))/L;
plot(f,P1) 
title('Single-Sided Amplitude Spectrum of X(t)')
xlabel('f (Hz)')
ylabel('|P1(f)|')

figure
