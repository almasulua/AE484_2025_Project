%   Modifications by Anıl Cem Almasulu <almasulua@gmail.com>

clc
clear
close all
matlabrc
format long;

% Paths to functions
addpath C:\Users\User\Desktop\METU\AE484\Proje\YildizNAV-main\Yildiz_Localization_INS_VOR_DME\ins
addpath C:\Users\User\Desktop\METU\AE484\Proje\YildizNAV-main\Yildiz_Localization_INS_VOR_DME\ins-gnss
addpath C:\Users\User\Desktop\METU\AE484\Proje\YildizNAV-main\Yildiz_Localization_INS_VOR_DME\conversions
addpath C:\Users\User\Desktop\METU\AE484\Proje\YildizNAV-main\Yildiz_Localization_INS_VOR_DME\performance-analysis
addpath C:\Users\User\Desktop\METU\AE484\Proje\YildizNAV-main\Yildiz_Localization_INS_VOR_DME\plot
addpath C:\Users\User\Desktop\METU\AE484\Proje\YildizNAV-main\Yildiz_Localization_INS_VOR_DME\allan-variance
addpath C:\Users\User\Desktop\METU\AE484\Proje\YildizNAV-main\Yildiz_Localization_INS_VOR_DME\Radio-Navigation

%% NOTES
%%


%%
%% CODE EXECUTION PARAMETERS

INS_GNSS    = 'ON';
PLOT        = 'ON';

if (~exist('INS_GNSS','var')),  INS_GNSS = 'OFF'; end
if (~exist('PLOT','var')),      PLOT = 'OFF'; end
if (~exist('IEXPLORER','var')), IEXPLORER = 'OFF'; end

%% CONSTANTS
%%
G =  9.80665;       % Gravity constant, m/s^2
D2R = (pi/180);     % Degrees to radians
R2D = (180/pi);     % Radians to degrees
G2T = 1E-4;         % Gauss to Tesla

fprintf(' INS/GNSS integration for μINS Dual IMU from Inertial Sense... \n')

%% TIME INTERVAL
%%
tmin = 1715182623;     % GNSS first time
tmax = 1715183023 ;   % GNSS last time 

%% REFERENCE
%%
load ref_zedf9p.mat
ref_data = struct2table(ref_data);

% ref data structure
ref.lat = ref_data.Latitude * D2R;
ref.lon = ref_data.Longitude * D2R;
ref.t = double(ref_data.Time);
ref.h = ref_data.Altitude;
% Serveral fields are missing: roll, pitch, yaw,vel etc.    

%% IMU DATA
%%

load imu_xsense_delivers.mat
imu_data = struct2table(imu_data);

% IMU data structure
imu.t = double(imu_data.Time);
imu.fb = [imu_data.Ax imu_data.Ay imu_data.Az ] ;
imu.wb = [imu_data.Gx imu_data.Gy imu_data.Gz ] ;


% IMU frequency
imu.freq = get_freq(imu.t);

%% GNSS DATA 
%%

load zedf9p_gnss_delivers.mat
load gnss_vel_data_delivers.mat

gnss_data = struct2table(gnss_data);
gnss_vel_data = struct2table(gnss_vel_data);


% IMU data structure
gnss.t = double(gnss_data.Time);
gnss.lat = gnss_data.Latitude * D2R;
gnss.lon = gnss_data.Longitude * D2R;
gnss.h = gnss_data.Altitude;
gnss.vel = [ gnss_vel_data.Velx gnss_vel_data.Vely gnss_vel_data.Velz] ;

% GNSS frequency
gnss.freq = get_freq(gnss.t);

%% IMU ERROR PROFILE
%%
% Error profile from Allan variance
% Apply to system just one time for taking error profile
% imu_allan = allan_imu(imu);
% 
 imu.ini_align = [ -9.9864e-05 0.0103 0.2256 ];
 imu.ini_align_err = [0.5 0.5 1.5] * D2R;
 imu.vrw = [0.357052112936883,0.961774519858292,0.097937677583613];
 imu.arw = [0.042121868141229,0.009525679451282,0.215964553168146];
 imu.vrrw = [4.690958161815616,0.025017042332123,8.151900100864590];
 imu.arrw = [0.365296716563698,0.652790283467330,0.332242479057422];
 imu.ab_dyn = [0.317843621047626,0.269877573663284,0.043008085334205];
 imu.gb_dyn = [0.119085569975979,0.183160823036258,0.119983749586203];
 imu.ab_corr = [8.658268346330733,1.489702059588082,14.137172565486901];
 imu.gb_corr = [0.209958008398320,0.129974005198960,4.809038192361528];
 imu.ab_psd = [0.935252807786899,0.329394623420481,0.161707956271642];
 imu.gb_psd = [0.054566407491244,0.066032970944379,0.263118595909555];
 imu.ab_sta = [70e-06 70e-06 70e-06];
 imu.gb_sta = [0.00166667 0.00166667 0.00166667];
%% GNSS ERROR PROFILE
%%

% Some GNSS profile

gnss.stdm = ones(1,3) .* 0.01 ;
gnss.stdv = ones(1,3) .* 0.05 ; %
gnss.larm = [0 0.03974 0.00851]';

gnss.zupt_th  = 0.1;       % ZUPT threshold (m/s).
gnss.zupt_win = 2;         % ZUPT time window (seconds)
gnss.eps = mean(diff(imu.t)) / 2;

% Converts GPS standard deviation from meters to radians
gnss = gnss_m2r(gnss.lat(1), gnss.h(1), gnss);


%% NAVIGATION TIME
%%
to = (gnss.t(end) - gnss.t(1));

fprintf(' navigation time is %.2f minutes or %.2f seconds. \n', (to/60), to)

%% TRAVELED DISTANCE
%%
distance = gnss_distance (gnss.lat, gnss.lon);

fprintf(' distance traveled by the vehicle is %.2f meters or %.2f km. \n', distance, distance/1000)

%% INS/GNSS INTEGRATION
%%

if strcmp(INS_GNSS, 'ON')
    
    % fprintf(' INS/GNSS integration... \n')
    nav_veri = ins_gnss(imu, gnss, 'quaternion');
    % ---------------------------------------------------------------------
    
    save yavuz_nav nav_veri
else
    
    load yavuz_nav
end


%% Eq. of RTK Orientation(Orientation is not provided from rtk)

 ref.yaw = nav_veri.yaw;
 ref.roll =nav_veri.roll;
 ref.pitch = nav_veri.pitch;

%% INTERPOLATION OF INS/GNSS DATASET
%%

% INS/GNSS estimates and GNSS data are interpolated according to the
% reference dataset.

[nav_i,  ref_n] = interpolation (nav_veri, ref);
[gnss_i, ref_g] = interpolation (gnss, ref);

%% NAVIGATION RMSE
%%

rmse_v = print_rmse (nav_i, gnss_i, ref_n, ref_g, 'Inertial Sense INS/GNSS');

%% PLOTS
%%

if (strcmp(PLOT,'ON'))
    
    plot_main (ref, gnss, nav_veri, gnss_i, nav_i, ref_g, ref_n)
end

%% SUMMARY
%%


