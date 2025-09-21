% length of signal (samples)
N = length (x); % Next power of 2 from length
NFFT = 2^nextpow2 (N);
% Fourier Transform of x
y = fft (x, NFFT);
% Power Spectral Density (PSD) (V^2/Hz)
Sx = abs (y)/N;
% Single-sided power spectrum of signal x: Px
Px = 2*Sx (1:NFFT/2);
% sampling frequency (Hz)
fs = 1 / dt;
% frequency division (Hz)
f = fs/2*linspace(0,1,NFFT/2);
%plot 0 to 10 Hz
figure
plot (f,Px);
axis ([0 10 0 3])
xlabel('f (Hz)');
ylabel('P_x');
