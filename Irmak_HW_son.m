close all
clear all

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
rx_spacing = lambda / 2; % Receiver spacing

N_angle_bins = V_num;
angle_axis = asind(2 *(-N_angle_bins/2:N_angle_bins/2-1) / 12);

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




%Generate virtual array positions (1D vector)
virtual_positions = zeros(V_num, 1);
idx = 1;
for tx_idx = 1:num_tx
    for rx_idx = 1:num_rx
        virtual_positions(idx) = (tx_idx-1) * tx_spacing + (rx_idx-1) * rx_spacing;
        idx = idx + 1;
    end
end


%ADC parameters:
%for single chirp, ADC parameters are:
fs_ADC=4.4e6; %ADC sampling frequency (Hz)
ADC_start_time=idle_time+10e-6; 
ADC_sampling_duration=29.2e-6; % Sampling duration (s)
ADC_end_time=ADC_start_time+ADC_sampling_duration; %end of sampling time (s)

% ADC_sampling_time=ADC_start_time:1/fs_ADC:ADC_end_time-1/fs_ADC;
ADC_sampling_time=ADC_start_time:1/fs_ADC:ADC_end_time-1/fs; % Time axis for ADC samples
ADC_sample_num=128;

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

% PARAMETERS FOR SIMULATION
min_range=0.5; %meters, modify accordingly
max_range=6; %meters, modify accordingly
max_angle=40; %degrees, modify accordingly
max_velocity=lambda/(4*Tp); %Objects' radial velocity (rad/s) (TO BE CHANGED)


angle_axis_vals=[-40 -30 -20 -10 0 10 20 30 40];
angle_num=length(angle_axis_vals);



%% Mixer design:

%R_matrix=reshape(R_rf,K,sampleNum_for_singlePulse);

% Load data and extract matrix
%adcDataStruct = load('1_300_0.mat'); 
%adcDataStruct = load('1_200_L30.mat'); 
% adcDataStruct = load('1_300_0 (1).mat'); 
% adcDataStruct = load('1_300_40L.mat');
% adcDataStruct = load('1_300_40L.mat');
% adcDataStruct = load('1_300_0.mat');
% adcDataStruct = load('1_100_0.mat');
% adcDataStruct = load('1_250_0.mat');
% adcDataStruct = load('1_150_0.mat');
% adcDataStruct = load('matlab.mat'); 
% adcDataStruct = load('1_100_40R.mat');
%adcDataStruct = load('1_200_0.mat');
%adcDataStruct = load('1_100_0.mat');
%adcDataStruct = load('1_200_L20.mat'); 
%adcDataStruct = load('1_200_0.mat');
% adcDataStruct = load('1_100_0.mat');
% adcDataStruct = load('1_200_L20.mat'); 
% adcDataStruct = load('1_200_L30.mat'); 
file_name=["RangeAngle_dB_Angle_Angle_"+45+"Range_"+1.5+".mat"];
% adcDataStruct = load('1_100_0.mat');
% adcDataStruct = load('1_300_0.mat');
% adcDataStruct = load('cl_kucuk_200cm.mat');
% %adcDataStruct = load('bos_oda.mat'); 
% adcDataStruct = load('1_300_0.mat');
% adcDataStruct = load('cl_kucuk_200cm.mat');
% adcDataStruct = load('cl_buyuk_150cm.mat');
% adcDataStruct = load('cl_kucuk_200cm.mat');
% adcDataStruct = load('cl_buyuk_200cm.mat');
% adcDataStruct = load('cl_buyuk_150cm.mat');
% adcDataStruct = load('1_300_0.mat');
% adcDataStruct = load('cl_buyuk_150cm.mat');
adcDataStruct = load('150cm_45deg.mat');
% adcDataStruct = load('cl_buyuk_200cm.mat');
% adcDataStruct = load('cl_kucuk_200cm.mat');
% adcDataStruct = load('cl_kucuk_150cm.mat');
% adcDataStruct = load('1_300_0.mat');
% adcDataStruct = load('1_200_L30.mat'); 
% adcDataStruct = load('1_300_0.mat');
% adcDataStruct = load('1_100_0.mat');
% adcDataStruct = load('1_200_0.mat');
% adcDataStruct = load('cl_buyuk_150cm.mat');
% adcDataStruct = load('1_100_0.mat');
% adcDataStruct = load('1_200_L30.mat');
fieldNames = fieldnames(adcDataStruct);
adcData = adcDataStruct.(fieldNames{1}); 

[rows, columns]= size(adcData);
%adcData=adcData(:,)
num_rx = 4; 
num_tx = 3; 
K = 16;
total_data_points = size(adcData, 2);
numFrames = 1;
ADC_Samples= 128;

numFrames =total_data_points/(ADC_Samples*num_tx*K);
% Frame, TX, K, ADCsamp 
% Reshape data into (RX, Chirps per TX, TX, Frames)
% Reshape data into (RX, ADC samples, Pulses, TX, Frames)
reshapedData = reshape(adcData, num_rx, ADC_Samples, K, num_tx, numFrames);

% Reorder data into (RX, TX, ADC samples, Pulses, Frames)
reorderedData = permute(reshapedData, [1, 4, 2, 3, 5]);

% Reshape into (12, 64, 16, 100), where 12 = RX * TX
RDMapsRxTx = reshape(reorderedData, V_num, ADC_Samples, K, numFrames);
RDMapsRxTx=RDMapsRxTx(:,:,:,1);

% Squeeze the frame dimension: (12, 16, 1024) 

RDMapsRxTx = permute(RDMapsRxTx, [1, 3, 2]);
% % Check the resulting size
% disp(size(RDMapsRxTx));

% reshapedData = reshape(adcData, num_rx, num_tx, K, ADC_Samples, numFrames);
IF_matrix_ADC = RDMapsRxTx;
% Reshape into (12 antennas, 16 pulses, data)
% IF_matrix_ADC = reshape(reshapedData, num_rx * num_tx, K, ADC_Samples * numFrames);
% 
% IF_matrix_ADC=reshapedData;


%ADC_Bw=ADC_sampling_duration*mu; 

range_resolution = c*fs_ADC/(2*mu*ADC_sample_num);
% range_resolution=c/(2*ADC_Bw); %same as above

range_axis = (0:ADC_sample_num-1) * range_resolution;




%c*max_beat_frequency_hw/(mu*2)

freq_axis=linspace(0,max_beat_frequency,ADC_sample_num);

%range_axis= (0:ADC_sample_num-1);

velocity_axis = (-K/2:K/2-1)*-lambda/(2*K*PRI);
snum=round(ADC_sample_num-1);

snum=length(range_axis);
RDMaps = zeros(V_num,  length(velocity_axis),snum);
% load('SingleLargeVariable.mat');
% RDMaps = zeros(V_num,  1,snum);

for l = 1:V_num

    IF_matrix_AD=squeeze(IF_matrix_ADC(l,:,:));


    window_function = hann(ADC_sample_num).';
    range_spectrum = fft(IF_matrix_AD .* window_function, [], 2);

    range_spectrum = fft(IF_matrix_AD.* window_function, [], 2);
    
    

    %observe range fft results:


    range_spectrum=range_spectrum(:,1:ADC_sample_num);
    RangeDoppler_Map=fftshift(fft(range_spectrum,[],1),1); %fft along slow time
    
    RDMaps(l, :, :) = RangeDoppler_Map;
    
    

end

figure();

imagesc(range_axis, velocity_axis, 20*log10(abs(RangeDoppler_Map)));
xlabel('Range (m)');
ylabel('Velocity (m/s)');
title(["Range-Doppler Map "]);
colorbar;
    % 

figure;
surf(range_axis, velocity_axis, 20*log10(abs(RangeDoppler_Map)), 'EdgeColor', 'none');
% title(["Range-Angle Map (Target with Angle=" + num2str(rad2deg(object_parameters(3))) + ...
%        "deg & Range=" + num2str(object_parameters(1)) + "m)"]);
xlabel('Range (m)');
ylabel('Angle (deg)');
zlabel('Amplitude (dB)');
colorbar;
caxis([-20, 100]);
caxis([-20, 100]);

frequency_axis=linspace(-fs/2, fs/2, ADC_sample_num);

% figure;
% surf(range_axis,velocity_axis,20*log(abs(RangeDoppler_Map)));
% surf(20*log(abs(RangeDoppler_Map)));
% title(['Range-Doppler Map ']);
% xlabel('Range (m)');
% ylabel('Velocity (deg)');
% zlabel('Amplitude (dB)');
% colorbar;
% view(0, 90); 


IF_matrix_AD=squeeze(IF_matrix_ADC(1,:,:));

IF_signal=[squeeze(IF_matrix_AD(1,:)) squeeze(IF_matrix_AD(2,:)) squeeze(IF_matrix_AD(3,:)) squeeze(IF_matrix_AD(4,:))]; %single chirp

% figure;
% pspectrum(IF_signal,fs_ADC,"spectrogram");
% title("IF signal after ADC Sampling- Hardware")
% caxis([-20 50])


window = hann(V_num);
window = window(:); 

RDMaps_windowed = RDMaps .* window;
%RDMaps_windowed = RDMaps;


% Angle Processing:
angle_cells=9
angle_axis = linspace(-60, 60, angle_cells); %  angle bins
zero_doppler_idx = K/2+1; % Zero velocity bin 
range_slices = squeeze(RDMaps_windowed(:, zero_doppler_idx, :)); % [12 virtual elements x range bins]

% Beamforming with steering vectors
steering_matrix = zeros(angle_cells, V_num);
for a_idx = 1:angle_cells
    angle_rad = deg2rad(angle_axis(a_idx));
    steering_matrix(a_idx, :) = exp(-1j * 2 * pi * virtual_positions' * sin(angle_rad) / lambda);
end

range_angle_map = abs(steering_matrix * range_slices); % [angles x range bins]

% Plot Range-Angle Map
figure;
surf(range_axis, angle_axis, 20*log10(range_angle_map), 'EdgeColor', 'none');
title(['Range-Angle Map ']);
xlabel('Range (m)');
ylabel('Angle (deg)');
zlabel('Amplitude (dB)');
colorbar;
view(0, 90); 
caxis([-20, 120]);

%%


max_range_values = max(20*log10(range_angle_map), [], 2); % Max across columns (angle axis)
max_angle_values = max(20*log10(range_angle_map), [], 1); % Max across rows (range axis)

[peaks, locs] = findpeaks(max_angle_values, 'MinPeakHeight', 90); 
first_peak_idx = locs(1); 
mask = [1,2,3,4,5].* (first_peak_idx <= 5); %if under index 5 mask 
noise_mean = 90; % Mean of Gaussian noise
noise_std = 20;  % Standard deviation of Gaussian noise

% Generate random noise
noise = noise_mean + noise_std * randn(size(mask));

% Replace unwanted peaks with noise
max_angle_values(mask) = noise;
% Create a logical mask (example: mask first N indices)


% Define the mask (example: mask the first N indices)
  % Mask the first 5 indices along the first dimension (rows)

% Define the noise value
noise_mean = 3000;  % Mean of Gaussian noise
noise_std = 20;   % Standard deviation of Gaussian noise
noisesize=8;
noise = noise_mean + noise_std * randn(9, noisesize); 
range_angle_map(:, 1:noisesize) = noise;

% 
% Plot the range-angle image

range_angle_map_DB=20*log10(range_angle_map);
% filename = sprintf('range_angle_map_dB_obj_range%.1f_angle%.1f.mat', ...
%         range_object_i, rad2deg(angle_object_i));
% saveas(range_angle_image_dB, "deneme1");
%
% save('range_angle_image.mat', "range_angle_image_dB");


% Find the maximum value and its index
[max_val, max_idx] = max(range_angle_map_DB(:));

[row, col] = ind2sub(size(range_angle_map_DB), max_idx); % Convert linear index to row and column

%Create a binary matrix of the same size initialized with zeros
Ground_truth_NN = zeros(size(range_angle_map));

%Set the position of the maximum value to 1
Ground_truth_NN(max_idx) = 1;

figure;
surf(range_axis, -angle_axis, Ground_truth_NN);




Folder_Name="\Users\oznur\Downloads\";
Folder_Name2="\Users\oznur\Downloads\";


full_file_name=fullfile(Folder_Name,file_name);

save(full_file_name,"range_angle_map_DB")


% save binary data:
full_file_name2=fullfile(Folder_Name2,file_name);
save(full_file_name2,"Ground_truth_NN")


% figure;

% subplot(2,1,1); % First subplot
% plot(-angle_axis, max_range_values, 'b', 'LineWidth', 2);
% xlabel('Angle(deg)');
% ylabel('Max Amplitude (dB)');
% title('Maximum Values Along Angle Axis');
% grid on;
% hold on;
% 
% subplot(2,1,2); % Second subplot
% plot(range_axis, max_angle_values, 'b', 'LineWidth', 2);
% xlabel('Range (m)');
% ylabel('Max Amplitude (dB)');
% title('Maximum Values Along Range Axis');
% grid on;

%%
% angular_spectrum =fft(RDMaps_windowed, [], 1);
% 
% 
% cube = abs(angular_spectrum);
% 
% A= abs(RangeDoppler_Map);
% [max_val, linear_index] = max(A(:)); 
% [row, col] = ind2sub(size(A), linear_index);
% 
% velocity_bin = row;  % Try different velocity bins
% 
% range_angle_image = squeeze(cube(:,  velocity_bin, :));
% 
% 
% max_range_values = max(20*log10(range_angle_image), [], 2); % Max across columns (angle axis)
% max_angle_values = max(20*log10(range_angle_image), [], 1); % Max across rows (range axis)
% 
% [peaks, locs] = findpeaks(max_angle_values, 'MinPeakHeight', 100); 
% first_peak_idx = locs(1); 
% mask = [1,2,3,4,5].* (first_peak_idx <= 5); %if under index 5 mask 
% noise_mean = 90; % Mean of Gaussian noise
% noise_std = 20;  % Standard deviation of Gaussian noise
% 
% % Generate random noise
% noise = noise_mean + noise_std * randn(size(mask));
% 
% % Replace unwanted peaks with noise
% max_angle_values(mask) = noise;
% % Create a logical mask (example: mask first N indices)
% 
% 
% % Define the mask (example: mask the first N indices)
%   % Mask the first 5 indices along the first dimension (rows)
% 
% % Define the noise value
% noise_mean = 3000;  % Mean of Gaussian noise
% noise_std = 20;   % Standard deviation of Gaussian noise
% noisesize=8;
% % Generate random noise with the same size as the section you want to mask
% noise = noise_mean + noise_std * randn(12, noisesize);  % Create noise of size 12x5x5
% 
% % Apply noise to the desired indices (first 5 rows, across all velocity bins, and depth slices)
% range_angle_image(:, 1:noisesize) = noise;
% 
% % Plot the range-angle image
% % figure;
% % surface(range_axis,-angle_axis,20*log10(range_angle_image))
% % title(["Range-Angle Map (Target with Angle=" + num2str(rad2deg(angle_object_i)) + ...
% %    "deg & Range=" + num2str(range_object_i) + "m)"]);
% % xlabel('Range (m)');
% % ylabel('Angle (deg)');
% % zlabel('Amplitude (dB)');
% % colorbar;
% % caxis([-20, 100]);
% % 
% 
% 
% figure;
% surface(range_axis,-angle_axis,20*log10(range_angle_image))
% title(["Range-Angle Map (Target with Angle=" + "??" + ...
%     "deg & Range=" + "??m)"]);
% xlabel('Range (m)');
% ylabel('Angle (deg)');
% zlabel('Amplitude (dB)');
% colorbar;
% caxis([-20, 100]);
% 
% figure;
% 
% subplot(2,1,1); % First subplot
% plot(-angle_axis, max_range_values, 'b', 'LineWidth', 2);
% xlabel('Angle(deg)');
% ylabel('Max Amplitude (dB)');
% title('Maximum Values Along Angle Axis');
% grid on;
% hold on;
% 
% subplot(2,1,2); % Second subplot
% plot(range_axis, max_angle_values, 'b', 'LineWidth', 2);
% xlabel('Range (m)');
% ylabel('Max Amplitude (dB)');
% title('Maximum Values Along Range Axis');
% grid on;
% 
% % sgtitle(['1D Maximum Value Plots for Target at Angle=' num2str(rad2deg(object_parameters(3))) ...
%          % ' deg & Range=' num2str(object_parameters(1)) ' m']);
% hold on;