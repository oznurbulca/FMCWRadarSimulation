close all;
clear all;

rng('default');

%Define parameters:
B= 10e7;  %bandwidth (Hz) 
Tp = 20e-6; %single pulse period (seconds) (PRI)
mu = B / Tp; %frequency sweep rate
f_c = 62e9;   %carrier frequency (Hz)
c = physconst('LightSpeed'); %speed of light (m/s)
fs = 2*B; %sampling frequency (Hz)

K=64; %number of pulses to transmit in one period 
SNR_val=10; %dB;
lambda=c/f_c; %wavelength 

%for random object parameters:
max_range=c/(2*B); %maximum available range for radar (m) (TO BE CHANGED)
max_radial_velocity=lambda/(4*Tp); %Objects' radial velocity (rad/s) (TO BE CHANGED)


sent_waveform=phased.FMCWWaveform(SampleRate=fs,SweepBandwidth=B,SweepTime=Tp,SweepDirection="Up",SweepInterval="Positive",OutputFormat="Sweeps",NumSweeps=K);
% figure;
% plot(sent_waveform)

Srf_n=transpose(sent_waveform()); %indexwise

%Short FT to observe freq. change over time:
[STFT_Srf,Frequency_content,Time_instants] = spectrogram(Srf_n,32,16,32,sent_waveform.SampleRate);

% image(T,fftshift(F),fftshift(mag2db(abs(S))))
% xlabel('Time (sec)')
% ylabel('Frequency (Hz)')


%number of indexes  =Tp*K*fs
sampleNum_for_singlePulse=round(Tp*fs); %index length (sample number of single pulse)
sampleNum_for_Srf=round(Tp*fs*K); %index length (sample number of sent signal)
SinglePulse_n=Srf_n(1,sampleNum_for_singlePulse+1:sampleNum_for_singlePulse*2);

%observe sent signal:
figure;
title("Plot of Sent Signal (Srf(n)")
subplot(2,1,1)
plot(1:sampleNum_for_singlePulse,real(SinglePulse_n))
xlabel("n")
ylabel("Amplitude")

subplot(2,1,2)
plot(1:sampleNum_for_Srf,real(Srf_n))
xlabel("n")
ylabel("Amplitude")

figure;
title("Plot of FT of Sent Signal (Srf(f)")
FT_Srf_n=fft(Srf_n);
plot(1:sampleNum_for_Srf,real(FT_Srf_n))
xlabel("n")
ylabel("Amplitude")



%Object definition:

N=1;% number of objects, keep 1 for ease

%Define parameters objects: [Range (m), Radial Frequency (rad/s)] 
object_parameters=zeros(N,2);
for k=1:N
    object_parameters(k,1)=rand()*max_range;
    object_parameters(k,1)=0.1;
    object_parameters(k,2)=rand()*max_radial_velocity;

end


R_rf=zeros(); %store the received signals from different objects in different rows

%obtain received signal from objects
%N=number of objects
for object_num=1:N

    R_rf_i=0; %received signal from object i 

    range_object_i=object_parameters(object_num,1);
    radial_vel_object_i=object_parameters(object_num,2);

    %CHECK HERE!!
    %delay due to object i:
    T_i=2*range_object_i/c;
    Ti_n=round(T_i*fs); %delay in terms of index n

    %doppler shift introduced to object i:
    fd= 2*radial_vel_object_i*f_c/c;

    %Amplitude constant for object i:
    A_i=1/(range_object_i^2);


    %R_rf=R_rf+A_i*exp(-1i*2*pi*fd)*p_t; %excluded fc
    R_rf=R_rf+ A_i*exp(-1i*2*pi*fd)*[zeros(1,Ti_n) Srf_n(1:end-Ti_n)]; %delayed signal


end

% add noise to received signals:
R_rf=awgn(R_rf,SNR_val);



% match filter operation for received signal for each Tp:
%FT and Time domain results are obtained. Time domain results are used for
%implementation

Y_filtered=zeros(K,sampleNum_for_singlePulse);
FT_SinglePulse=fft(Srf_n(1:sampleNum_for_singlePulse));

for pulse_number=1:K

    pulse_start_index=1+(pulse_number-1)*sampleNum_for_singlePulse;
    pulse_ending_index=sampleNum_for_singlePulse*pulse_number;

    FT_Rrf_i=fft(R_rf(pulse_start_index:pulse_ending_index)); %FT of received signal for ith pulse period
    
    Y_filtered(pulse_number,:)=FT_Rrf_i.*conj(FT_SinglePulse);

    y_filtered_v2(pulse_number,:)=conv(conj(flip(SinglePulse_n)), R_rf(pulse_start_index:pulse_ending_index));
    %Y_filtered(pulse_number,:)=conj(fft(Single_pulse_n)).*(fft(R_rf(1+(pulse_number-1)*single_pulse_length:single_pulse_length*pulse_number)));

end


%autocorrealtion result:
%in Fourier Domain:
%SinglePulse_autocorr=conj(fft(SinglePulse_n)).*fft(SinglePulse_n);

%in time domain:
SinglePulse_AutoC_res=conv(SinglePulse_n,conj(flip(SinglePulse_n)));


figure;
plot(abs(SinglePulse_AutoC_res))
title("Autocorrelation Result")

% figure;
% plot(abs(y_filtered_v2(1,:)))
% title("Matched Filter Result of received signal for first Pulse")

%limit the first half of the matched filter output:
y_filtered_signal=y_filtered_v2(:,sampleNum_for_singlePulse+1:end);
y_filtered_signal=[y_filtered_signal zeros(K,1)]; %additional zero to match the single pulse length


%Plot Matched filter outputs for different pulses
figure;
for pulsenumber=1:5
    subplot(5+1,1,pulsenumber+1)
    plot(abs(y_filtered_signal(pulsenumber,:)))

end


% %take IFT for modeling filtered signals r[n]:
% figure;
% title("Filtered Output for first pulse")
% y_filtered=ifft(Y_filtered);
% plot(abs(y_filtered(1,:)))






% % Range detection from FFT (to be asked)

RangeDoppler_Map=(fft(y_filtered_signal, [], 1)); %FFT along columns for Doppler Freq.  (x axis: Range, y axis: Doppler

RangeDoppler_Map=(fft2(y_filtered_signal,K,sampleNum_for_singlePulse)); %FFT along columns for Doppler Freq. 


frequency_axis=linspace(-fs/2, fs/2, sampleNum_for_singlePulse);
velocity_axis = (-K/2:K/2-1)*lambda/(2*K*Tp);
range_axis = (abs(frequency_axis)*c)/(2*mu); %range vals for frequency

figure();
title("Range-Doppler Map")
imagesc(range_axis, velocity_axis, 20*log10(abs(RangeDoppler_Map)));
xlabel('Range (m)');
ylabel('Velocity (m/s)');
title('Range-Doppler Map');
colorbar;
axis xy;




% 
% %FFT on each row of filtered signal:
% 
% frequency_axis=linspace(-fs/2, fs/2, sampleNum_for_singlePulse);
% range_axis = (abs(frequency_axis)*c)/(2*mu); %range vals for frequency 
% 
% %FFT accross each row:
% range_FFT=fft(y_filtered,[],2);



% range_doppler_map = fftshift(fft(range_FFT, [], 1), 1);

% % Calculate axes
% 
% velocity_axis = (-K/2:K/2-1)*lambda/(2*K*Tp);
% 
% figure('Name', 'Matched Filter Output');
% plot(range_axis, abs(y_filtered(1,:)));
% xlabel('Range (m)');
% ylabel('Amplitude');
% title('Matched Filter Output (First Chirp)');
% 
% % Plot Range-Doppler map
% figure('Name', 'Range-Doppler Map');
% imagesc(range_axis, velocity_axis, 20*log10(abs(range_doppler_map)));
% xlabel('Range (m)');
% ylabel('Velocity (m/s)');
% title('Range-Doppler Map');
% colorbar;
% axis xy;














%%  Range-Doppler FFT










%% CFAR



