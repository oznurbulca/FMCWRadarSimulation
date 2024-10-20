close all;
clear all;
%% TO DO
%obtain parameters from hardware
%carrier frequency adjustment (fc) -->check baseband
%object information in array format (doppler, range)
%CFAR implementation
%detection results with CFAR

%%

rng('default');

%Define parameters:
B= 10e7;  %bandwidth (Hz) 
Tp = 20e-6; %single pulse period (seconds)
mu = B / Tp; %frequency sweep rate
f_c = 62e9;   %carrier frequency (Hz)
c = 3e8; %speed of light (m/s)
fs = 2*B; %sampling frequency (Hz) 

time_axis = -10*Tp : 1/fs : 10*Tp; %time axis (sec)
n=length(time_axis);

N=5; %number of pulses

SNR_val=10; %dB;

%Define object parameters:
Range=100; %meter
radial_velocity=2; %rad/s;


p = @(t) ( (-Tp/2 <= t) & (t <= Tp/2) ) .* exp(1i*pi*mu*(t.^2)); %generate p(t) as written
fm = @(t) ( (-Tp/2 <= t) & (t <= Tp/2) ) .* (f_c+ (mu*t)); 


p_t=p(time_axis);
figure;
plot(time_axis,real(p_t));
hold on
plot(time_axis,real(p(time_axis-3*Tp)))




% function [S,f] =S_Tx(N,t, Tp,f_c,p, fm)
%     S = 0; 
%     f = 0; 
%     for K=0:N
%       S=S + p(t-K*Tp).*exp(1i*2*pi*f_c*t);
%       f=f+fm(t-K*Tp);
%     end
% end
% 
% [S, f] = S_Tx(N, t, Tp, f_c, p, fm);

%%

%obtain transmit signal Srf_t:
Srf_t=0;
f = 0;

for K=0:N
   Srf_t=Srf_t + p(time_axis-K*Tp).*exp(1i*2*pi*f_c*time_axis);
   f=f+fm(time_axis-K*Tp);
end

figure;
plot(time_axis, real(Srf_t));


figure;
plot(time_axis, f);
Y = fft(Srf_t);
N = length(Y);
ff = (-N/2 : N/2-1) * (fs / N);
Y_shifted = fftshift(Y);
figure;
plot(ff , abs(Y_shifted));

n=length(time_axis);


Td= 2*Range/c; %delay introduced to the received signal (s)
R_f=0;
A=1; 
fd= 2*radial_velocity*f_c/c; %Doppler Shift introduced to signal 



fl=0;

for index=0:5
    R_f=awgn(R_f+(A* p(time_axis-index*Tp-Td).*exp(1i*2*pi*f_c*(time_axis-Td)).* exp(-1i * 2 * pi * fd )), SNR_val);
    fl=fl+f_c+fd+fm(time_axis-index*Tp-Td);
end


figure;
plot(time_axis,real(R_f))
title(["Plot of Received signal for SNR= "], SNR_val);
xlabel("time (sec)");
ylabel("R_f(t)");


% 
figure;
plot(time_axis,f,'b');
hold on;
plot(time_axis,fl,'r');


%Autocorrelation part:
Y_received=conj(fft(Srf_t,length(time_axis))).*(fft(R_f,length(time_axis))); %check fft size!!!
y_filtered=conv(conj(flip(Srf_t)), R_f); %autocorrelation in time domain 


figure;
t2 = -20*Tp : 1/fs : 20*Tp;
plot(t2,real(y_filtered));


function Srf_t = S_tx()


end


%% combine with index wise alg.:

%Define parameters:
B= 10e7;  %bandwidth (Hz) 
Tp = 20e-6; %single pulse period (seconds)
mu = B / Tp; %frequency sweep rate
f_c = 60;   %carrier frequency (Hz)
c = 3e8; %speed of light (m/s)
fs = 2*B; %sampling frequency (Hz) 

time_axis = -10*Tp : 1/fs : 10*Tp; %time axis (sec)
w=wgn(1,n,0.01,'linear'); %white noise 

N=5; %number of pulses
sampling_time=1/fs; %seconds

w=wgn(1,n,0.01,'linear'); %white noise 
SNR_value=10; %dB;


%object parameters:
T_delay=Range/2*c;
radial_velocity=2; %rad/s


%transmit signal:
unit_delay=T_delay/sampling_time;
fd= 2*radial_velocity*f_c/c; %Doppler Shift introduced to signal 
