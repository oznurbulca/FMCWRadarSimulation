close all;
clear all;

rng('default');

%%TO BE ADDED:
%idle time Ti,Tp,ooverall PRI _B/2 den B/2



%Define parameters:
B= 100e6;  %bandwidth (Hz) 
Tp = 20e-6; %single pulse period (seconds) (PRI)
mu = B / Tp; %frequency sweep rate
f_c = 62e9;   %carrier frequency (Hz)
c = physconst('LightSpeed'); %speed of light (m/s)
fs = 4*B; %sampling frequency (Hz)
PRI=24e-6;

K=64; %number of pulses to transmit in one period 
SNR_val=10; %dB;
lambda=c/f_c; %wavelength 

idle_time=Tp/5; %idle time at the beginning of the pulse (s)
idle_time_end=0; %idle time at the end of the pulse (s)

%indexwise idle durations:
Tidle_begin_n=round(idle_time*fs); %idle time at the beginnig of the pulse
Tidle_end_n=round(idle_time_end*fs); %idle time at the end of the pulse


%for random object parameters:
max_range=c/(2*B); %maximum available range for radar (m) (TO BE CHANGED)
max_radial_velocity=lambda/(4*Tp); %Objects' radial velocity (rad/s) (TO BE CHANGED)



sent_waveform=phased.FMCWWaveform(SampleRate=fs,SweepBandwidth=B,SweepTime=Tp,SweepDirection="Up",SweepInterval="Positive",OutputFormat="Sweeps",NumSweeps=K);
% figure;
% plot(sent_waveform)

Srf_n=transpose(sent_waveform()); %indexwise

% %Short FT to observe freq. change over time:
% [STFT_Srf,Frequency_content,Time_instants] = spectrogram(Srf_n,32,16,32,sent_waveform.SampleRate);
% 
% image(Time_instants,fftshift(Frequency_content),fftshift(mag2db(abs(STFT_Srf))))
% xlabel('Time (sec)')
% ylabel('Frequency (Hz)')



%number of indexes  =Tp*K*fs
sampleNum_for_singlePulse=round(Tp*fs); %index length (sample number of single pulse)
sampleNum_for_Srf=round(Tp*fs*K); %index length (sample number of sent signal)
SinglePulse_n=Srf_n(sampleNum_for_singlePulse+1:sampleNum_for_singlePulse*2);

%observe sent signal:
figure;
title("Plot of Sent Signal (Srf(n))")
subplot(2,1,1)
plot(1:sampleNum_for_singlePulse,imag(SinglePulse_n))
xlabel("index (n)")
ylabel("Amplitude")

subplot(2,1,2)
plot(1:sampleNum_for_Srf,abs(Srf_n))
xlabel("n")
ylabel("Amplitude")


%Adjust frequency axis!!!!!
figure;
title("Plot of FT of Sent Signal (Srf(f))")
FT_Srf_n=(fft(Srf_n));
plot(linspace(-B/2,B/2,sampleNum_for_Srf),abs(FT_Srf_n))
ylabel("Amplitude")
xlabel('Frequency (Hz)')


%Adjust Sent Signal (Add idle time)
Srf_n;
total_idle_samples=Tidle_begin_n+Tidle_end_n;
Padded_singlePulse_n=[zeros(1,Tidle_begin_n) SinglePulse_n zeros(1,Tidle_end_n)];

Srf_n = repmat(Padded_singlePulse_n, 1, K); %Sent signal with idle times


sampleNum_for_singlePulse=length(Padded_singlePulse_n);
sampleNum_for_Srf=length(Srf_n);
time=0:1/fs:PRI*K-1/fs;


% %plot zero padded signal: (freq)



%Object definition:

N=1;% number of objects, keep 1 for ease

%Define parameters objects: [Range (m), Radial Frequency (rad/s)] 
object_parameters=zeros(N,2);
for k=1:N
    object_parameters(k,1)=rand()*max_range;

    object_parameters(k,1)=1500; %for trial 
    %object_parameters(k,2)=rand()*max_radial_velocity;
    object_parameters(k,2)=10;

end


R_rf=zeros(); %store the received signals from different objects in different rows

%obtain received signal from objects
%N=number of objects
for object_num=1:N


    R_rf_i=0; %received signal from object i 

    range_object_i=object_parameters(object_num,1);
    radial_vel_object_i=object_parameters(object_num,2);

    
    %delay due to object i:
    T_i=2*range_object_i/c;
    Ti_n=round(T_i*fs); %delay in terms of index n

    %doppler shift introduced to object i:
    %fd= 2*radial_vel_object_i*f_c/c;

    fd= 2*radial_vel_object_i/lambda;
    

    %Amplitude constant for object i:
    if range_object_i ~=0
        A_i=1;
    else
        A_i=1;

    end

    %R_rf=R_rf+A_i*exp(-1i*2*pi*fd)*p_t; %excluded fc
    R_rf=R_rf+ A_i*exp(1i*2*pi*fd*time).*[zeros(1,Ti_n) Srf_n(1:end-Ti_n)]; %delayed signal


end

% add noise to received signals:
R_rf=awgn(R_rf,SNR_val);



% match filter operation for received signal for each Tp:
%FT and Time domain results are obtained. Time domain results are used for
%implementation



%conj, flip 
%Pulse_for_convn=conj(flip(SinglePulse_n)); %
Pulse_for_convn=conj(flip(Padded_singlePulse_n));

for pulse_number=1:K

    pulse_start_index=1+(pulse_number-1)*sampleNum_for_singlePulse;
    pulse_ending_index=sampleNum_for_singlePulse*pulse_number;

    y_filtered_v2(pulse_number,:)=conv(Pulse_for_convn, R_rf(pulse_start_index:pulse_ending_index));

end


%autocorrealtion result:
SinglePulse_AutoC_res=conv(SinglePulse_n,conj(flip(SinglePulse_n)));


figure;
plot(abs(SinglePulse_AutoC_res))
title("Autocorrelation Result")
ylabel("Amplitude")
xlabel("index (n)")

%deneme
figure;
plot(abs(SinglePulse_AutoC_res(sampleNum_for_singlePulse:end)))
title("Autocorrelation Result")
ylabel("Amplitude")
xlabel("index (n)")

% figure;
% plot(abs(y_filtered_v2(1,:)))
% title("Matched Filter Result of received signal for first Pulse")

%obtain the second half of the matched filter output (convolution output):
y_filtered_signal=y_filtered_v2(:,sampleNum_for_singlePulse+1:end);

%define range axis:
range_axis=1:1:sampleNum_for_singlePulse-1;
range_axis=range_axis*c/(fs*2);

figure;
plot(range_axis,abs(y_filtered_signal(3,:)))
title(["Matched Filter output for 3rd Pulse Duration for Target at Range= "+num2str(range_object_i)+"m"])
ylabel("Amplitude")
xlabel("Range (m)")





%estimate range: (TO BE CHANGED)
[~ ,peak_loc_all]=max(abs(y_filtered_signal), [],2);
peak_location=mode(peak_loc_all);

Range_estimation=peak_location/fs*c/2
range_object_i


%Find error:
Percentage_error=abs(range_object_i-Range_estimation)/abs(range_object_i)*100







%Plot Matched filter outputs for different pulses
figure;

for pulsenumber=1:5
    subplot(5,1,pulsenumber)
    plot(range_axis,abs(y_filtered_signal(pulsenumber,:)))
    ylabel("Amplitude")
    xlabel("Range (m)")

end
sgtitle('Matched Filter Outputs for Random Chosen 5 Pulses')






%Range Doppler Map:

RangeDoppler_Map=fftshift(fft(y_filtered_signal, [], 1),1); %FFT along columns for Doppler Freq.  (x axis: Range, y axis: velocity)
frequency_axis=linspace(-fs/2, fs/2, sampleNum_for_singlePulse);
velocity_axis = (-K/2:K/2-1)*lambda/(2*K*PRI);


figure();
%surf

imagesc(range_axis, velocity_axis, 20*log10(abs(RangeDoppler_Map)));
xlabel('Range (m)');
ylabel('Velocity (m/s)');
title(["Range-Doppler Map (Target with Velocity="+num2str(radial_vel_object_i)+"m/s" "& Range="+num2str(range_object_i)+"m"]);
colorbar;

xlim([0 3000]) %limit according to observable range
caxis([-20, 100]);

cb = colorbar(); 
ylabel(cb,'Power (db)','Rotation',270)
axis xy;


%Store for investigation:
%33th row=> velocity=0
% [~, max_index_map]=max(abs(RangeDoppler_Map(33,:)));








