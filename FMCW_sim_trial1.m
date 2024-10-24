close all;
clear all;
%% ----TO DO
%obtain parameters from hardware (fs,
%carrier frequency adjustment (fc) -->check baseband
%object information in array format (doppler, range)---
%CFAR implementation
%detection results with CFAR

%% -----

rng('default');

%Define parameters:
B= 10e7;  %bandwidth (Hz) 
Tp = 20e-6; %single pulse period (seconds)
mu = B / Tp; %frequency sweep rate
f_c = 62e9;   %carrier frequency (Hz)
c = physconst('LightSpeed'); %speed of light (m/s)
fs = 2*B; %sampling frequency (Hz) 

%for random objects:
max_range=200; %maximum available range for radar (m) (TO BE CHANGED)
rad_freq_range=20; %for calculating the doppler shift range for objects (rad/s)


time_axis = -10*Tp : 1/fs : 10*Tp; %time axis (sec)
n=length(time_axis);

K=5; %number of pulses to transmit in one period

SNR_val=10; %dB;
N=1;% number of objects, keep 1 for ease

%Define parameters objects: [Range, Radial Frequency] 
object_parameters=zeros(N,2);
for k=1:N
    object_parameters(k,1)=rand()*max_range;
    object_parameters(k,2)=rand()*rad_freq_range;

end



p = @(t) ( (-Tp/2 <= t) & (t <= Tp/2) ) .* exp(1i*pi*mu*(t.^2)); %generate p(t) as written
fm = @(t) ( (-Tp/2 <= t) & (t <= Tp/2) ) .* (f_c+ (mu*t)); 


%obtain transmit signal, and frequency val of transmit signal
[Srf_t, freq_values] = S_Tx(K, time_axis, Tp, f_c, p, fm);


R_rf=zeros(); %store the received signals from different objects in different rows

%obtain received signal from objects
for object_num=1:N

    R_rf_i=0; %received signal from object i 

    range_object_i=object_parameters(object_num,1);
    radial_vel_object_i=object_parameters(object_num,2);

    %delay due to object i:
    T_i=2*range_object_i/c;

    %doppler shift introduced to object i:
    fd= 2*radial_vel_object_i*f_c/c;

    %Amplitude constant for object i:
    A_i=1/(range_object_i^2);

    %obtain p(t) part for ith object in a seperate sum:
    p_t=zeros();

    for k=0:K

        p_t=p_t+p(time_axis-k*Tp-T_i);

    end

    %before with fc: R_rf=R_rf+A_i*exp(1i*2*pi*f_c*T_i)*exp(-1i*2*pi*fd)*p_t; 
    R_rf(object_num)=R_rf_i+A_i*exp(-1i*2*pi*fd)*p_t; %excluded fc


end

% add noise to received signals:
R_rf=awgn(R_rf,SNR_val);


%Match Filter operation:
Y_filtered=conj(fft(Srf_t)).*(fft(R_rf)); %check fft size!!!
y_filtered=conv(conj(flip(Srf_t)), R_rf); %autocorrelation in time domain 


figure;
subplot(3,1,1)
plot(real(fft(Srf_t)));
title("Plot of Transmit Signal S_rf")
subplot(3,1,2)
plot(real(fft(R_rf)));
title("Plot of Received Signal R_rf")
subplot(3,1,3)
plot(real(Y_filtered));
title("Plot of Filtered Signal Y_rf")


%% match filter operation for received signal for each Tp:

Y_filtered=zeros(K,length_pulsePeriod);

for pulse_number=1:K

    %Y_filtered(pulse_number,:)=conv(conj(flip(Srf_t(1:length_pulsePeriod))), R_rf(1+(pulse_number-1)*length_pulsePeriod:length_pulsePeriod*pulse_number));
    Y_filtered(pulse_number,:)=conj(fft(Srf_t(40002:length_pulsePeriod+40001))).*(fft(R_rf(1+(pulse_number-1)*length_pulsePeriod:length_pulsePeriod*pulse_number)));

end

%autocorrealtion result:
Srf_autocorr_res=conj(fft(Srf_t(40002:length_pulsePeriod+40001))).*fft(Srf_t(40002:length_pulsePeriod+40001));
figure;
title("FT domain results for Matched Filter")
subplot(2,1,1)
plot(real(Srf_autocorr_res))
title("Autocorrelation Result of Srf (for one pulse period)")

subplot(2,1,2)
plot(real(Y_filtered(4,:)))
title("Matched Filter output of received signal")







%% detect range from FFT

y_filtered=ifft(Y_filtered(1,:));
figure;
subplot(2,1,1)
plot(real(y_filtered))
title("Filtered Signal y(n)")
subplot(2,1,2)
plot(real(Srf_t(40002:length_pulsePeriod+40001)))
title("Sent Signal Srf(n) for one period")




%% detect doppler

%% CFAR








% FUNCTIONS


% function for obtaining the transmit signal S_rf(t), and frequency axis
function [S,f] =S_Tx(K,t, Tp,f_c,p, fm)
    S = 0; 
    f = 0; 
    for k=0:K
      S=S + p(t-k*Tp).*exp(1i*2*pi*f_c*t);
      f=f+fm(t-k*Tp);
    end
end

%function for obtaining the received signal R_rf(t)

%--now R-rf(t) is calculated above for clarity, will be transformed into function form later 
