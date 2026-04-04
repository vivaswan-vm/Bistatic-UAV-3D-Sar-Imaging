function[radar,platform]= CommData_function(t_slow,x_points_e ,y_points_e ,z_points_e,struc_constants, struc_trajectory, radar, platform, target)
x_points_e2 = x_points_e;
y_points_e2 = y_points_e;
z_points_e2 = z_points_e;
% Bytes of data to be transmitted
PSDULength = 120;%input("Enter the size of data to be transmitted (in Bytes): ");%1260

%Converting bytes into bits
data.data_size_bytes = PSDULength;
data.data_size_bits = PSDULength*8;
% Generating  data bits equivalent to the data bits 
% 
% ORDERED PARAMETER LIST
%% =======================
param_list = [
    radar.range_res
    radar.del_cr
    radar.PRF
    radar.Tp
    radar.SNR
    radar.Phi_mono
    radar.Phi_bi
    radar.lambda
    radar.Bw
    radar.Fs
    radar.PRI
    platform.Rc
    platform.Rc_2
    platform.vc
    platform.vx
    platform.vz
    platform.zc
    platform.zc_2
    platform.xc
    platform.xc_2
    platform.yc
    platform.yc_2
    platform.SAL_mono
    platform.SAL_bi
    platform.SAT_mono
    platform.SAT_bi
    platform.Delta_X
    x_points_e2
    y_points_e2
    z_points_e2
];

%% =======================
% FLOAT32 → IEEE-754 BITS
%% =======================
bitstream = '';

for k = 1:length(param_list)
    val_single = single(param_list(k));           % float32
    val_uint32 = typecast(val_single, 'uint32');  % raw bits
    val_bits   = dec2bin(val_uint32, 32);         % 32-bit binary
    bitstream  = [bitstream, val_bits];           %#ok<AGROW>
end

total_bits = length(bitstream);


%disp('Generating data bits equivalent to the data bits to be transmitted');
data.data_bits = bitstream - '0';
data.data_bits  = data.data_bits.'; 
txPSDU = data.data_bits;

[tx_short,dmg_short] = packet_generation(data,PSDULength);
%disp('Packet generation done');       




% Initialization of short packet parameters
[struc_ofdm_info_short,struc_packet_par_short] = initialisation_frame_parameters_func(PSDULength,...
    dmg_short,struc_constants);



    % Get Target position (azimuth and range)
    [trajec_angle,range_tgt] = get_target_position(t_slow,struc_trajectory);
     % Calculation of BER
    [BERarray,snr_db,rxPSDU] = allProcessing_ber_func_V1(tx_short,txPSDU,trajec_angle,range_tgt,...
                struc_constants,struc_packet_par_short,struc_ofdm_info_short,dmg_short);
   
 %% Conversion of bit to information       

 bitstream_char = char(rxPSDU + '0');
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% received bitstream
num_params = length(param_list);   % should be 30
bit_matrix = reshape(bitstream_char, 32, []).';   % N x 32

uint_vals  = zeros(1, num_params, 'uint32');
double_arr = zeros(1, num_params);            % 1×32 double

for k = 1:num_params
    uint_vals(k)  = uint32(bin2dec(bit_matrix(k, :)));
    double_arr(k) = double(uint_vals(k));
end
%%%%%%%%%%%%%%%%%%%%%%%%%
%% =======================
% DOUBLE → FLOAT32 VALUES
%% =======================

float_vals = zeros(1, num_params, 'single');

for k = 1:num_params
    float_vals(k) = typecast(uint32(double_arr(k)), 'single');
end
%% =======================
% RECONSTRUCT PARAMETERS
%% =======================

idx = 1;

radar.range_res_ext = float_vals(idx); idx = idx + 1;
radar.del_cr_ext    = float_vals(idx); idx = idx + 1;
radar.PRF_ext      = float_vals(idx); idx = idx + 1;
radar.Tp_ext       = float_vals(idx); idx = idx + 1;
radar.SNR_ext       = float_vals(idx); idx = idx + 1;
radar.Phi_mono_ext  = float_vals(idx); idx = idx + 1;
radar.Phi_bi_ext    = float_vals(idx); idx = idx + 1;
radar.lambda_ext    = float_vals(idx); idx = idx + 1;
radar.Bw_ext        = float_vals(idx); idx = idx + 1;
radar.Fs_ext        = float_vals(idx); idx = idx + 1;
radar.PRI_ext       = float_vals(idx); idx = idx + 1;

platform.Rc_ext     = float_vals(idx); idx = idx + 1;
platform.Rc_2_ext   = float_vals(idx); idx = idx + 1;

platform.vc_ext     = float_vals(idx); idx = idx + 1;
platform.vx_ext     = float_vals(idx); idx = idx + 1;
platform.vz_ext     = float_vals(idx); idx = idx + 1;

platform.zc_ext     = float_vals(idx); idx = idx + 1;
platform.zc_2_ext   = float_vals(idx); idx = idx + 1;

platform.xc_ext     = float_vals(idx); idx = idx + 1;
platform.xc_2_ext   = float_vals(idx); idx = idx + 1;

platform.yc_ext     = float_vals(idx); idx = idx + 1;
platform.yc_2_ext   = float_vals(idx); idx = idx + 1;

platform.SAL_mono_ext = float_vals(idx); idx = idx + 1;
platform.SAL_bi_ext   = float_vals(idx); idx = idx + 1;

platform.SAT_mono_ext = float_vals(idx); idx = idx + 1;
platform.SAT_bi_ext   = float_vals(idx); idx = idx + 1;

platform.Delta_X_ext  = float_vals(idx); idx = idx + 1;

platform.x_points_e_ext = float_vals(idx); idx = idx + 1;
platform.y_points_e_ext = float_vals(idx); idx = idx + 1;
platform.z_points_e_ext = float_vals(idx);     
end