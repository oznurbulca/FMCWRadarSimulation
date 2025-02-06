close all
clear all

%Define parameters:
B= 3461e6;  %bandwidth (Hz) 
Tp = 49e-6; %single pulse period (seconds) RAMP END TIME 
mu = B/Tp; %frequency sweep rate (Hz/sec)
f_c = 60e9;   %carrier frequency (Hz)
c = physconst('LightSpeed'); %speed of light (m/s)
f_max=f_c+B; %maximum freq of chirp signal (Hz)
fs = 2*f_max+B/2; %sampling frequency for cont. time signal(Hz)
lambda=c/f_c; %wavelength 


%chirp signal parameters: (Assuming Baseband)
start_frequency=0;
end_frequency=start_frequency+B;
fs=2*end_frequency+200;
idle_time=7e-6; %idle time at the beginning of the pulse (s)
PRI=idle_time+Tp; %pulse repetition imnterval (s)
time_axis_chirp=0:1/(fs):Tp-1/fs;  %time axis for single chirp

%indexwise idle duration:
Tidle_n=round(idle_time*fs); %idle time at the beginnig of the pulse


K=54; %number of chirps to transmit in one period 
SNR_val=10; %dB;

%Antenna parameters
num_tx = 3; % Number of transmitters 3
num_rx = 4; % Number of receivers
V_num = num_tx*num_rx; %5mm
% V_num=16;
tx_spacing = lambda; % Transmitter spacing
rx_spacing = lambda / 2; % Receiver spacing

% Generate array positions
tx_positions = (0:num_tx-1) * tx_spacing;
rx_positions = (0:num_rx-1) * rx_spacing;

virtual_positions = zeros(num_tx, num_rx);
for i = 1:num_tx
    for j = 1:num_rx
        virtual_positions(i, j) = tx_positions(i) + rx_positions(j);
    end
end



%ADC parameters:
%for single chirp, ADC parameters are:
fs_ADC=5.914e6; %ADC sampling frequency (Hz)
ADC_start_time=5.7e-6; 
ADC_sampling_duration=42.66e-6; % Sampling duration (s)
ADC_end_time=ADC_start_time+ADC_sampling_duration; %end of sampling time (s)

ADC_sampling_time=ADC_start_time:1/fs_ADC:ADC_end_time-1/fs; % Time axis for ADC samples


max_beat_frequency=mu*(ADC_end_time); %calculated
max_beat_frequency_hw=4.731e6; %(Hz) from hardware

%object paramters:
max_range_calc=c*max_beat_frequency_hw/(mu*2); %maximum available range for radar (m) (TO BE CHANGED)
max_range=10; %(m) from hardware
max_velocity=lambda/(4*Tp); %Objects' radial velocity (rad/s) (TO BE CHANGED)




%obtain chirp signal:
chirp_signal=chirp(time_axis_chirp,start_frequency,time_axis_chirp(end),end_frequency,"linear");

chirp_signal=exp(1i*pi*mu*time_axis_chirp.^2);

% display the chirp without idle times:
% figure;
% pspectrum(chirp_signal,fs,"spectrogram");
% title("Chirp Signal, Baseband")


%% Add idle time:

Single_Pulse_n=[zeros(1,Tidle_n) chirp_signal];

time_axis_pulse=0:1/fs:PRI-1/fs;
sampleNum_for_singlePulse=length(Single_Pulse_n);


%display chirp w/idle time:
% figure;
% pspectrum(Single_Pulse_n,fs,"spectrogram",OverlapPercent=99);
% title("Chirp with Idle time, Baseband")

%stft(Single_Pulse_n,fs) %gives + and - components? 
% transmittedsignalsfromdifferenttx=zeros(num_tx,length(Single_Pulse_n)*K);
% for tx = 1:num_tx

%obtain transmit signal:
Srf_n = repmat(Single_Pulse_n, 1, K);
    %Sent signal with idle times

sampleNum_for_Srf=length(Srf_n);
time_axis_Srf=0:1/fs:PRI*K-1/fs;

% %Observe Transmit Signal:
% figure;
% pspectrum(Srf_n,fs,"spectrogram");
% title("Transmit Signal (54 Chirps)")



%% Object definition:

N=1;% number of objects, keep 1 for ease

%Define parameters objects: [Range (m), Velocity(m/s)] 
object_parameters=zeros(N,3);
for k=1:N
    object_parameters(k,1)=rand()*max_range;

    object_parameters(k,1)=2; %for trial 
    %object_parameters(k,2)=rand()*max_radial_velocity;
    object_parameters(k,2)=0; %velocity
    object_parameters(k,3)=deg2rad(30); %angle 

end

angle_axis = -60:1:60; % Angle range for AoA estimation in degrees
angle_axis = linspace(-60, 60, V_num);
% angle_axis= cos(angle_axis)*2*pi;

% Calculate steering vector for each angle of arrival


%% Received Signal:

R_rf=zeros(V_num,length(Srf_n)); %store the received signals from different objects in different rows

%obtain received signal from objects
%N=number of objects
% Steering vector calculation based on angle of arrival
for object_num = 1:N
    range_object_i = object_parameters(object_num,1);
    radial_vel_object_i = object_parameters(object_num,2);
    angle_object_i = object_parameters(object_num,3); % Object angle

    % Delay due to object i
    T_i = 2 * range_object_i / c;
    Ti_n = round(T_i * fs); % Delay in terms of index n
    time_axis_targeti = [zeros(1, Ti_n) time_axis_Srf(Ti_n+1:end)];

    % Doppler shift
    fd = 2 * radial_vel_object_i / lambda;  % Doppler frequency

    % Amplitude constant for object i
    A_i = 1; % Assuming a constant amplitude

    for v = 1:V_num
        % Calculate the steering vector for angle of arrival (azimuth)
        steering_vector = exp(-1j * 2 * pi * (v - 1) * rx_spacing / lambda * sin(angle_object_i));

        % Update the received signal with azimuth effect
        R_rf(v, :) = R_rf(v, :) + A_i * exp(-1j * 2 * pi * fd * time_axis_Srf) .* ...
                     [zeros(1, Ti_n) Srf_n(1:end - Ti_n)] .* steering_vector; % Applying steering vector
    end
end


% add noise to received signals:
% for v =1:V_num
R_rf=awgn(R_rf,SNR_val);
% end
% % % % % % down sample eklediginizde noise ekleyin
% %Observe Received Signal:
% figure;
% subplot(2,1,2)
% pspectrum(R_rf(1,1:sampleNum_for_singlePulse*4),fs,"spectrogram");
% title(["Received Signal (First 4 Chirps) For Delay="+T_i*10^6+"us"])
% 
% 
% 
% 
% % Observe Received Signal:
% figure;
% subplot(2,1,2)
% pspectrum(R_rf(1:sampleNum_for_singlePulse*4),fs,"spectrogram");
% title(["Received Signal (First 4 Chirps) For Delay="+T_i*10^6+"us"])
% 
% 
% subplot(2,1,1)
% pspectrum(Srf_n(1:sampleNum_for_singlePulse*4),fs,"spectrogram");
% title("Transmit Signal (First 4 Chirps)")



%% Mixer design:

%R_matrix=reshape(R_rf,K,sampleNum_for_singlePulse);
IF_signal = zeros(V_num, length(Srf_n));
for l = 1:V_num
    IF_signal(l, :) = (R_rf(l, :)) .* conj(Srf_n); % Mixer output for antenna l
end
%filter mixed signal:
[b,a]=butter(5,max_beat_frequency_hw/(fs/2));

% IF_matrix=[];
IF_matrix = zeros(V_num, K, sampleNum_for_singlePulse); % Store IF signal for each antenna
for l = 1:V_num
    for pulse_number = 1:K
        start_index = 1 + (pulse_number - 1) * sampleNum_for_singlePulse;
        end_index = sampleNum_for_singlePulse * pulse_number;
        % Filter the signal
        IF_matrix(l, pulse_number, :) = filter(b, a, IF_signal(l, start_index:end_index));
    end
end
% figure;
% pspectrum(IF_signal(1,1:sampleNum_for_singlePulse*4),fs,"spectrogram");
% title("Spectrum of Mixed Signal (First 4 Chirps) ")

% 
% % %observe mixed signal:
% % N = 2^nextpow2(length(IF_signal)); % FFT size
% % figure;
% % pspectrum(IF_signal(1:1:sampleNum_for_singlePulse*4),fs,"spectrogram");
% % title("Spectrum of Mixed Signal (First 4 Chirps) ")
% 
% % 
% % figure;
% % pspectrum(IF_signal(1:sampleNum_for_singlePulse*4),fs,"spectrogram");
% % title("Spectrum of Mixed Signal (First 4 Chirps) ")
% % % 
% % % 
% figure;
% f = fs * (-N/2:N/2-1) / N;
% plot(f,abs(real(fftshift(fft(IF_signal(1,:),N)))))
% title("Absolute of FFT of Mixed Signal")
% xlabel("frequency (Hz)");
% ylim([-10000 10000])
% % 
% % 
% f_beat=(frange-fd)*10^-9
% fd
% 
% 
% 
% %% ADC Sampling
% 
% 
% 
% %time_axis_chirp=0:1/fs:PRI; %time axis for chirp
% 
% % Initialize IF_matrix_ADC for all antennas
IF_matrix_ADC = zeros(V_num, K, length(ADC_sampling_time)); % Dimensions: Antennas x Pulses x ADC Samples
% 
% Loop through each antenna
for l = 1:V_num
    % Process signals for antenna l
    IF_signal_ADC = 0; % Reset for each antenna


    for pulse_number=1:K

        start_index=1+(pulse_number-1)*sampleNum_for_singlePulse;
        end_index=sampleNum_for_singlePulse*(pulse_number);

        IF_single_chirp=IF_signal(l,start_index:end_index);
        IF_signal_ADC=interp1(time_axis_pulse,IF_single_chirp,ADC_sampling_time,"linear");

        IF_matrix_ADC(l, pulse_number, :) = IF_signal_ADC;
    end


end

%Range Doppler Maps from ADC output 
ADC_sample_num=length(ADC_sampling_time);

% define range axis and velocity axis according to ADC sampling:
range_resolution = c*fs_ADC/(2*mu*ADC_sample_num);


range_axis = (0:ADC_sample_num/2-1) * range_resolution;



%c*max_beat_frequency_hw/(mu*2)

freq_axis=linspace(0,max_beat_frequency,ADC_sample_num);

%range_axis= (0:ADC_sample_num-1);

velocity_axis = (-K/2:K/2-1)*-lambda/(2*K*PRI);
snum=round(ADC_sample_num/2-1);
RDMaps = zeros(V_num,  length(velocity_axis),snum);
% load('SingleLargeVariable.mat');
% RDMaps = zeros(V_num,  1,snum);
for l = 1:V_num

    IF_matrix_AD=squeeze(IF_matrix_ADC(l,:,:));


    window_function = hann(ADC_sample_num).';
    range_spectrum = fftshift(fft(IF_matrix_AD .* window_function, [], 2), 2);


    %observe range fft results:


    range_spectrum=fliplr(range_spectrum(:,1:ADC_sample_num/2));
    RangeDoppler_Map=fftshift(fft(range_spectrum,[],1),1); %fft along slow time
    
    RDMaps(l, :, :) = RangeDoppler_Map;
    % 
    % figure;
    % surf(range_axis, velocity_axis, 20*log10(abs(RangeDoppler_Map)), 'EdgeColor', 'none');
    % title(["Range-Angle Map (Target with Angle=" + num2str(rad2deg(object_parameters(3))) + ...
    %        "deg & Range=" + num2str(object_parameters(1)) + "m)"]);
    % xlabel('Range (m)');
    % ylabel('Angle (deg)');
    % zlabel('Amplitude (dB)');
    % colorbar;
    % caxis([-20, 100]);
    % caxis([-20, 100]);

    %frequency_axis=linspace(-fs/2, fs/2, ADC_sample_num);



    % %observe range doppler map:
    % figure();
    % 
    % imagesc(range_axis, velocity_axis, 20*log10(abs(RangeDoppler_Map)));
    % xlabel('Range (m)');
    % ylabel('Velocity (m/s)');
    % title(["Range-Doppler Map (Target with Velocity="+num2str(radial_vel_object_i)+"m/s" "& Range="+num2str(range_object_i)+"m)"]);
    % colorbar;
    % 

end

% Third FFT over antennas

% window = ones(V_num, 1); % Rectangular window
% window = window(:); 
% 
% 
% % Apply windowing to the Range-Doppler map along the angular dimension
% RDMaps_windowed = RDMaps .* window;
% angular_spectrum = fftshift(fft(RDMaps_windowed, [], 1), 1);
% 
% 
% cube = abs(angular_spectrum);
% 
% velocity_bin = 28;  % Try different velocity bins
% 
% range_angle_image = squeeze(cube(:,  velocity_bin, :));
% % figure;
% % imagesc( range_axis,-angle_axis, 20*log10(range_angle_image));
% % title(["Range-Angle Map (Target with Angle="+num2str(rad2deg(object_parameters(3))+"m/s" "& Range="+num2str(object_parameters(1))+"m)"]);
% % 
% % xlabel('Range (m)');
% % ylabel('Angle (deg)');
% % 
% % colorbar;
% % caxis([-20, 100]);
% 
% % 
% figure;
% surf(range_axis, -angle_axis, 20*log10(range_angle_image), 'EdgeColor', 'none');
% title(["Range-Angle Map (Target with Angle=" + num2str(rad2deg(object_parameters(3))) + ...
%        "deg & Range=" + num2str(object_parameters(1)) + "m)"]);
% xlabel('Range (m)');
% ylabel('Angle (deg)');
% zlabel('Amplitude (dB)');
% colorbar;
% caxis([-20, 100]);
% % view(2); % Change view for better perspective
% 
% 
% 
% % figure;
% % surf(range_axis, velocity_axis, 20*log10(abs(RangeDoppler_Map)), 'EdgeColor', 'none');
% % title(["Range-Angle Map (Target with Angle=" + num2str(rad2deg(object_parameters(3))) + ...
% %        "deg & Range=" + num2str(object_parameters(1)) + "m)"]);
% % xlabel('Range (m)');
% % ylabel('Angle (deg)');
% % zlabel('Amplitude (dB)');
% % colorbar;
% % caxis([-20, 100]);
% 
% % Compute the maximum values along range and angle axes
% max_range_values = max(20*log10(range_angle_image), [], 2); % Max across columns (angle axis)
% max_angle_values = max(20*log10(range_angle_image), [], 1); % Max across rows (range axis)
% 

% figure;
% 

% subplot(2,1,1); % First subplot
% plot(-angle_axis, max_range_values, 'b', 'LineWidth', 2);
% xlabel('Range (m)');
% ylabel('Max Amplitude (dB)');
% title('Maximum Values Along Angle Axis');
% grid on;

% subplot(2,1,2); % Second subplot
% plot(range_axis, max_angle_values, 'r', 'LineWidth', 2);
% xlabel('Angle (deg)');
% ylabel('Max Amplitude (dB)');
% title('Maximum Values Along Range Axis');
% grid on;

% sgtitle(['1D Maximum Value Plots for Target at Angle=' num2str(rad2deg(object_parameters(3))) ...
%          ' deg & Range=' num2str(object_parameters(1)) ' m']);


window = hann(V_num);
window = window(:); 

RDMaps_windowed = RDMaps .* window;
angular_spectrum = fftshift(fft(RDMaps_windowed, [], 1), 1);


cube = abs(angular_spectrum);

velocity_bin = 28;  % Try different velocity bins

range_angle_image = squeeze(cube(:,  velocity_bin, :));
% figure;
% imagesc( range_axis,-angle_axis, 20*log10(range_angle_image));
% title(["Range-Angle Map (Target with Angle="+num2str(rad2deg(object_parameters(3))+"m/s" "& Range="+num2str(object_parameters(1))+"m)"]);
% 
% xlabel('Range (m)');
% ylabel('Angle (deg)');
% 
% colorbar;
% caxis([-20, 100]);

% 
figure;
surf(range_axis, -angle_axis, 20*log10(range_angle_image), 'EdgeColor', 'none');
title(["Range-Angle Map (Target with Angle=" + num2str(rad2deg(object_parameters(3))) + ...
       "deg & Range=" + num2str(object_parameters(1)) + "m)"]);
xlabel('Range (m)');
ylabel('Angle (deg)');
zlabel('Amplitude (dB)');
colorbar;
caxis([-20, 100]);
% view(2); % Change view for better perspective



% figure;
% surf(range_axis, velocity_axis, 20*log10(abs(RangeDoppler_Map)), 'EdgeColor', 'none');
% title(["Range-Angle Map (Target with Angle=" + num2str(rad2deg(object_parameters(3))) + ...
%        "deg & Range=" + num2str(object_parameters(1)) + "m)"]);
% xlabel('Range (m)');
% ylabel('Angle (deg)');
% zlabel('Amplitude (dB)');
% colorbar;
% caxis([-20, 100]);

max_range_values = max(20*log10(range_angle_image), [], 2); % Max across columns (angle axis)
max_angle_values = max(20*log10(range_angle_image), [], 1); % Max across rows (range axis)

figure;

subplot(2,1,1); % First subplot
plot(-angle_axis, max_range_values, 'b', 'LineWidth', 2);
xlabel('Range (m)');
ylabel('Max Amplitude (dB)');
title('Maximum Values Along Angle Axis');
grid on;

subplot(2,1,2); % Second subplot
plot(range_axis, max_angle_values, 'r', 'LineWidth', 2);
xlabel('Angle (deg)');
ylabel('Max Amplitude (dB)');
title('Maximum Values Along Range Axis');
grid on;

sgtitle(['1D Maximum Value Plots for Target at Angle=' num2str(rad2deg(object_parameters(3))) ...
         ' deg & Range=' num2str(object_parameters(1)) ' m']);
