% TDMS file import example for Intel LIDAR Streaming project
% Purpose: 
% - Load only a part of a large TDMS file.
% - Perform bit unpacking if needed
% - Scale the data to Volts
% IMPORTANT: Works only with the file type used in the LIDAR example.

clear all 
close all
clc

% File Info
%file_path = 'Card_0_CH0.tdms';         % File to Open - int16 version
file_path = 'Card_0_CH0_packed.tdms';   % File to Open - bitpacked version
is_packed = 1;                          % If file was saved with BitPacking set this to 1
scaling = 0.0002442002442;              % Get Scaling factor from LabVIEW API. This is dependent on bitness and Vmax. This for 1V 12bit.
%scaling = 1.526251526E-5;               % 16 bit
fs = 800e6;                             % Just needed for the time axis


% Custom read settings for the given file
offset_samples = 0; % no. of samples to skip from the start
read_length = 10002; % no. of samples to read. Should be integer multiple of 3 if packed.

% Constants
MACHINE_FORMAT  = 'ieee-le'; %NOTE: Eventually this could be passed into
STRING_ENCODING = 'UTF-8';
START_OF_DATA = 4096;
DATA_TYPE = 'int16';
SIZE_OF_DATA_TYPE = 2;

% File Open, read and Close.
fid = fopen(file_path,'r',MACHINE_FORMAT,STRING_ENCODING);
fseek(fid,START_OF_DATA+(offset_samples*SIZE_OF_DATA_TYPE),'bof');
data = fread(fid,read_length,DATA_TYPE);
fclose(fid);


% Unpack the data if needed
if is_packed
   data = cast(data,'int16')';
   data_0 = data(1 : 3 : end);
   data_1 = data(2 : 3 : end);
   data_2 = data(3 : 3 : end);
   
   % Re-format data variable
   data = int16(zeros(1,4*length(data_0)));   
   data(1:4:end) = bitshift(bitshift(data_0,4),-4);
   data(2:4:end) = bitor(bitand(int16(0x000F),bitshift(data_0,-12)), bitshift(bitshift(data_1,8),-4));
   data(3:4:end) = bitor(bitand(int16(0x00FF),bitshift(data_1,-8)), bitshift(bitshift(data_2,12),-4));
   data(4:4:end) = bitshift(data_2,-4);   
end

% Scale Data
scaled_data = double(data).*scaling;

%Plot Block
figure;
plot(data);
title('Raw Data')

figure;
t = (0:1/fs:(1/fs)*(length(scaled_data)-1)).*1e6;
plot(t,scaled_data)
title('Scaled Data (Volt)')
xlabel('Time(us)')
ylabel('Amplitude (V)')


