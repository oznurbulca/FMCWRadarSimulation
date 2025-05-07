close all
clear all
%Change Folder name to where you save
Folder_Name="/Users/irmakfitoz/Documents/IndustrialDesignProject/SIMULATION_TRAINING_DATASET/MULTIPLE_TARGETS/10DB_SNR/TARGET";
Folder_Name2="/Users/irmakfitoz/Documents/IndustrialDesignProject/SIMULATION_TRAINING_DATASET/MULTIPLE_TARGETS/10DB_SNR/TARGET_MASKED"; %for saving the binary data
NUMBER_OF_MAPS=300; %number of maps to be generated
%adjust line 178 for the number of objects per map if necessary (Current: Randomly generated betwen 2-10)
%Adjust line 35 for the SNR value  (Current: 10dB)
%%

%Define parameters:
B= 3919e6;  %bandwidth (Hz)
Tp = 40e-6; %single pulse period (seconds) RAMP END TIME
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
idle_time=10e-6; %idle time at the beginning of the pulse (s)
PRI=idle_time+Tp; %pulse repetition imnterval (s)
time_axis_chirp=0:1/(fs):Tp-1/fs;  %time axis for single chirp

%indexwise idle duration:
Tidle_n=round(idle_time*fs); %idle time at the beginnig of the pulse


K=16; %number of chirps to transmit in one period
SNR_val=10; %dB;

%Antenna parameters
num_tx = 3; % Number of transmitters 3
num_rx = 4; % Number of receivers
V_num = num_tx*num_rx; %5mm

% V_num=12;
tx_spacing = lambda; % Transmitter spacing
rx_spacing = lambda / 2; % Receiver spacing

% Generate array positions
tx_positions = (0:num_tx-1) * tx_spacing;
rx_positions = (0:num_rx-1) * rx_spacing;

virtual_positions = zeros(num_tx, num_rx);
for index_i = 1:num_tx
    for index_j = 1:num_rx
        virtual_positions(index_i, index_j) = tx_positions(index_i) + rx_positions(index_j);
    end
end


%Generate virtual array positions (1D vector)
virtual_positions = zeros(V_num, 1);
idx = 1;
for tx_idx = 1:num_tx
    for rx_idx = 1:num_rx
        virtual_positions(idx) = (tx_idx-1) * tx_spacing + (rx_idx-1) * rx_spacing;
        idx = idx + 1;
    end
end


% virtual_positions(9)=virtual_positions(5);
% virtual_positions(10)=virtual_positions(6);
% virtual_positions(5)=virtual_positions(3);
% virtual_positions(6)=virtual_positions(4);
% virtual_positions(11)=virtual_positions(7);
% virtual_positions(12)=virtual_positions(8);


% angular_resolution=rad2deg(lambda/2*(V_num-1));
% angle_axis = -60:1:60; % Angle range for AoA estimation in degrees
% angle_axis = linspace(-90, 90, V_num)*angular_resolution;
% % angle_axis= cos(angle_axis)*2*pi;
%
N_angle_bins = V_num;
N_virtual=V_num;


% spatial_freq2 = (-floor(N_angle_bins/2):ceil(N_angle_bins/2)-1) / N_virtual;
% angle_axis2 = asind(2*spatial_freq);



N_angle_bins = V_num;
angle_axis = asind(2 *(-N_angle_bins/2:N_angle_bins/2-1) / 12);
angle_axis=linspace(-60,60,12);

angle_axis_vals=[-50 -40 -25 -10 10 25 35 50];
angle_num=length(angle_axis_vals);



%ADC parameters:
%for single chirp, ADC parameters are:
fs_ADC=2*4.4e6; %ADC sampling frequency (Hz)
ADC_start_time=idle_time+10e-6;
ADC_sampling_duration=29.2e-6; % Sampling duration (s)
ADC_end_time=ADC_start_time+ADC_sampling_duration; %end of sampling time (s)

ADC_sampling_time=ADC_start_time:1/fs_ADC:ADC_end_time-1/fs; % Time axis for ADC samples
ADC_sample_num=128*2;

if length(ADC_sampling_time)<ADC_sample_num
    ADC_sampling_time=[zeros(1,ADC_sample_num-length(ADC_sampling_time)) ADC_sampling_time];
elseif length(ADC_sampling_time)>ADC_sample_num
    ADC_sampling_time=ADC_sampling_time(1,1+length(ADC_sampling_time)-ADC_sample_num:end);
end

max_beat_frequency=mu*(ADC_end_time); %calculated
max_beat_frequency_hw=4.731e6; %(Hz) from hardware

%object paramters:
max_range_calc=c*max_beat_frequency_hw/(mu*2); %maximum available range for radar (m) (TO BE CHANGED)
max_range=10; %(m) from hardware
max_velocity=lambda/(4*Tp); %Objects' radial velocity (rad/s) (TO BE CHANGED)
min_range=0.5; %meters, modify accordingly
max_range=6; %meters, modify accordingly



%obtain chirp signal:
%chirp_signal=chirp(time_axis_chirp,start_frequency,time_axis_chirp(end),end_frequency,"linear");

chirp_signal=exp(1i*pi*mu*time_axis_chirp.^2);

% display the chirp without idle times:
% figure;
% pspectrum(chirp_signal,fs,"spectrogram");
% title("Chirp Signal, Baseband")


%% Add idle time:

Single_Pulse_n=[zeros(1,Tidle_n) chirp_signal];

time_axis_pulse=0:1/fs:PRI-1/fs;
sampleNum_for_singlePulse=length(Single_Pulse_n);

%
% display chirp w/idle time:
% figure;
% pspectrum(Single_Pulse_n,fs,"spectrogram");
% title("Chirp with Idle time, Baseband")

%stft(Single_Pulse_n,fs) %gives + and - components?
% transmittedsignalsfromdifferenttx=zeros(num_tx,length(Single_Pulse_n)*K);
% for tx = 1:num_tx

%obtain transmit signal:
Srf_n = repmat(Single_Pulse_n, 1, K);
%Sent signal with idle times

sampleNum_for_Srf=length(Srf_n);
time_axis_Srf=0:1/fs:PRI*K-1/fs;

% correction for non-matching time axis:
if length(Srf_n)~=length(time_axis_Srf)
    Srf_n=[zeros(1,length(time_axis_Srf)-length(Srf_n)) Srf_n];
end

% %Observe Transmit Signal:
% figure;
% pspectrum(Srf_n,fs,"spectrogram");
% title("Transmit Signal (16 Chirps)")


for map_idx=1:NUMBER_OF_MAPS

    %% Object definition:

    N=randi([2 10]); % number of targets in the map

    %Define parameters objects: [Range (m), Velocity(m/s)]
    object_parameters=zeros(N,3);
    for k=1:N

        object_parameters(k, 1) = rand()*(max_range-min_range)+min_range; % Random range
        object_parameters(k, 2) = 0; % Assume zero velocity for simplicity
        randomIndex = randi(angle_num, 1);
        object_parameters(k, 3) = deg2rad(angle_axis_vals(randomIndex)); % Random angle


    end


    % Calculate steering vector for each angle of arrival


    %% Received Signal:

    R_rf=zeros(V_num,length(Srf_n)); %store the received signals from different objects in different rows

    %obtain received signal from objects
    %N=number of objects
    % Steering vector calculation based on angle of arrival
    % for object_num = 1:N
    %     range_object_i = object_parameters(object_num,1);
    %     radial_vel_object_i = object_parameters(object_num,2);
    %     angle_object_i = object_parameters(object_num,3); % Object angle
    %
    %     % Delay due to object i
    %     T_i = 2 * range_object_i / c;
    %     Ti_n = round(T_i * fs); % Delay in terms of index n
    %     time_axis_targeti = [zeros(1, Ti_n) time_axis_Srf(Ti_n+1:end)];
    %
    %     % Doppler shift
    %     fd = 2 * radial_vel_object_i / lambda;  % Doppler frequency
    %
    %     % Amplitude constant for object i
    %     A_i = 1; % Assuming a constant amplitude
    %
    %     for v = 1:V_num
    %         % Calculate the steering vector for angle of arrival (azimuth)
    %         steering_vector = exp(-1j * 2 * pi * (v - 1) * rx_spacing / lambda * sin(angle_object_i));
    %
    %         % Update the received signal with azimuth effect
    %         R_rf(v, :) = R_rf(v, :) + A_i * exp(-1j * 2 * pi * fd * time_axis_Srf) .* ...
    %                      [zeros(1, Ti_n) Srf_n(1:end - Ti_n)] .* steering_vector; % Applying steering vector
    %     end
    % end

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

            steering_vector = exp(-1j * 2 * pi * virtual_positions(v) * sin(angle_object_i) / lambda);


            % Update the received signal with azimuth effect
            R_rf(v, :) = R_rf(v, :) + A_i * exp(-1j * 2 * pi * fd * time_axis_Srf) .* ...
                [zeros(1, Ti_n) Srf_n(1:end - Ti_n)] .* steering_vector; % Applying steering vector


        end


    end



    %% Add noise to received signal:
    % for v =1:V_num
    R_rf=awgn(R_rf,SNR_val);

    % end
    % % % % % % down sample eklediginizde noise ekleyin
    %
    %Observe Received Signal:
    % figure;
    %
    % pspectrum(R_rf(1,1:sampleNum_for_singlePulse*4),fs,"spectrogram");
    % title(["Received Signal (First 4 Chirps) For Delay="+T_i*10^6+"us"])

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
    IF_matrix_ADC = zeros(V_num, K, ADC_sample_num); % Dimensions: Antennas x Pulses x ADC Samples
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

    IF_matrix_AD=squeeze(IF_matrix_ADC(1,:,:));

    IF_signal=[squeeze(IF_matrix_AD(1,:)) squeeze(IF_matrix_AD(2,:)) squeeze(IF_matrix_AD(3,:)) squeeze(IF_matrix_AD(4,:))]; %single chirp

    % figure;
    % pspectrum(IF_signal,fs_ADC,"spectrogram");
    % title("IF signal after ADC Sampling Simulation")
    % caxis([-20 50])


    %% Range Doppler Maps from ADC output
    %ADC_sample_num=length(ADC_sampling_time);

    ADC_Bw=ADC_sampling_duration*mu; % ADC Bandwidth

    % define range axis and velocity axis according to ADC sampling:
    range_resolution = c*fs_ADC/(2*mu*ADC_sample_num);
    %range_resolution=c/(2*ADC_Bw); %same as above

    range_axis = (1:ADC_sample_num/2) * range_resolution;



    %c*max_beat_frequency_hw/(mu*2)

    freq_axis=linspace(0,max_beat_frequency,ADC_sample_num);

    %range_axis= (0:ADC_sample_num-1);

    velocity_axis = (-K/2:K/2-1)*-lambda/(2*K*PRI);
    snum=round(ADC_sample_num/2-1);

    snum=length(range_axis);
    RDMaps = zeros(V_num,  length(velocity_axis),snum);
    % load('SingleLargeVariable.mat');
    % RDMaps = zeros(V_num,  1,snum);
    for l = 1:V_num

        IF_matrix_AD=squeeze(IF_matrix_ADC(l,:,:));


        window_function = hann(ADC_sample_num).';
        range_spectrum = fftshift(fft(IF_matrix_AD .* window_function, [], 2), 2);

        range_spectrum = fftshift(fft(IF_matrix_AD, [], 2), 2);


        %observe range fft results:


        range_spectrum=fliplr(range_spectrum(:,1:(ADC_sample_num/2)));
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



        %observe range doppler map:
        % figure();
        %
        % imagesc(range_axis, velocity_axis, 20*log10(abs(RangeDoppler_Map)));
        % xlabel('Range (m)');
        % ylabel('Velocity (m/s)');
        % title(["Range-Doppler Map (Target with Velocity="+num2str(radial_vel_object_i)+"m/s" "& Range="+num2str(range_object_i)+"m)"]);
        % colorbar;
        % %

    end

    % figure();
    % 
    % imagesc(range_axis, velocity_axis, 20*log10(abs(RangeDoppler_Map)));
    % xlabel('Range (m)');
    % ylabel('Velocity (m/s)');
    % title(["Range-Doppler Map (Target with Velocity="+num2str(radial_vel_object_i)+"m/s" "& Range="+num2str(range_object_i)+"m)"]);
    % colorbar;

    %% angle processing

    % Angle Processing (for 9 bins):
    angle_cells=9;
    angle_axis = linspace(-60, 60, angle_cells); %  angle bins
    zero_doppler_idx = K/2 + 1; % Zero velocity bin
    range_slices = squeeze(RDMaps(:, zero_doppler_idx, :)); % [12 virtual elements x range bins]

    % Beamforming with steering vectors
    steering_matrix = zeros(angle_cells, V_num);
    for a_idx = 1:angle_cells
        angle_rad = deg2rad(angle_axis(a_idx));
        steering_matrix(a_idx, :) = exp(-1j * 2 * pi * virtual_positions' * sin(angle_rad) / lambda);
    end
    range_angle_map = abs(steering_matrix * range_slices); % [angles x range bins]

    % % Plot Range-Angle Map
    % figure;
    % surf(range_axis, -angle_axis, 20*log10(range_angle_map), 'EdgeColor', 'none');
    % title(['Range-Angle Map ']);
    % xlabel('Range (m)');
    % ylabel('Angle (deg)');
    % zlabel('Amplitude (dB)');
    % colorbar;
    % view(0, 90);
    % caxis([-20, 120]);


    %Angle Processing (for 9 bins):
    angle_cells=9;
    angle_axis = linspace(-60, 60, angle_cells); %  angle bins
    zero_doppler_idx = K/2 + 1; % Zero velocity bin
    range_slices = squeeze(RDMaps(:, zero_doppler_idx, :)); % [12 virtual elements x range bins]
    angle_cells = 9;  % Number of cells (regions)
    angle_edges = linspace(-60, 60, angle_cells+1);  % Defines the edges of angle bins

    %Calculate the bin centers (for display and steering vector calculation)
    angle_centers = (angle_edges(1:end-1) + angle_edges(2:end))/2;

    % Beamforming with steering vectors
    steering_matrix = zeros(angle_cells, V_num);
    for a_idx = 1:angle_cells
        angle_rad = deg2rad(angle_axis(a_idx));
        steering_matrix(a_idx, :) = exp(-1j * 2 * pi * virtual_positions' * sin(angle_rad) / lambda);
    end
    range_angle_map = abs(steering_matrix * range_slices); % [angles x range bins]


    %% observe max vals:

    max_ANGLE_values = max(20*log10(range_angle_map), [], 2); % Max across columns (angle axis)
    max_RANGE_values = max(20*log10(range_angle_map), [], 1); % Max across rows (range axis)

    % figure;
    %
    % subplot(2,1,1); % First subplot
    % plot(-angle_axis, max_ANGLE_values, 'b', 'LineWidth', 2);
    % xlabel('Range (m)');
    % ylabel('Max Amplitude (dB)');
    % title('Maximum Values Along Angle Axis');
    % grid on;
    %
    % subplot(2,1,2); % Second subplot
    % plot(range_axis, max_RANGE_values, 'r', 'LineWidth', 2);
    % xlabel('Angle (deg)');
    % ylabel('Max Amplitude (dB)');
    % title('Maximum Values Along Range Axis');
    % grid on;
    %
    % sgtitle(['1D Maximum Value Plots for Target at Angle=' num2str(rad2deg(object_parameters(3))) ...
    %          ' deg & Range=' num2str(object_parameters(1)) ' m']);
    %%
    range_angle_map_DB=20*log10(range_angle_map);

    % Initialize ground truth map
    Ground_truth_NN = zeros(size(range_angle_map));

    % Map each object's range and angle to the corresponding bins
    for obj_idx = 1:N
        % Get object parameters
        range_obj = object_parameters(obj_idx, 1); % Range in meters
        angle_obj = rad2deg(object_parameters(obj_idx, 3)); % Angle in degrees

        % Find the closest range bin
        [~, range_bin] = min(abs(range_axis - range_obj));

        % Find the closest angle bin
        [~, angle_bin] = min(abs(-angle_centers - angle_obj));

        % Set the corresponding bin to 1
        Ground_truth_NN(angle_bin, range_bin) = 1;
    end



    % % Plot Range-Angle Map
    % figure;
    % imagesc(range_axis, -angle_centers, 20*log10(range_angle_map));
    % xlabel('Range (m)');
    % ylabel('Angle (deg)');
    % zlabel('Amplitude (dB)');
    % colorbar;
    % view(0, 90);
    % caxis([-20, 120]);
    % 
    % range_angle_map_DB=20*log10(range_angle_map);
    % [max_val,  max_idx] = max(range_angle_map_DB(:));
    % 
    % [row, col] = ind2sub(size(range_angle_map_DB), max_idx); % Convert linear index to row and column
    % 
    % 
    % figure;
    % imagesc(range_axis, -angle_centers, Ground_truth_NN);
    % xlabel('Range (m)');
    % ylabel('Angle (deg)');
    % colorbar;
    % view(0, 90);



    %% Save the image to matlab file:

    range_angle_map_DB=20*log10(range_angle_map);
    % filename = sprintf('range_angle_map_dB_obj_range%.1f_angle%.1f.mat', ...
    %         range_object_i, rad2deg(angle_object_i));
    % saveas(range_angle_image_dB, "deneme1");
    %
    % save('range_angle_image.mat', "range_angle_image_dB");


    %file_name=["RangeAngle_dB_Angle_Angle_"+rad2deg(angle_object_i)+"Range_"+range_object_i+".mat"];
    file_name=['RangeAngle_dB' num2str(N) '_Objects'];
    full_file_name=fullfile(Folder_Name,file_name);
    save(full_file_name,"range_angle_map_DB")


    % save binary data:
    file_name=['GroundTruth' num2str(N) '_Objects'];
    full_file_name2=fullfile(Folder_Name2,file_name);
    save(full_file_name2,"Ground_truth_NN")
end
