dataBME = csvread('log02/log02_N9Boardcore10BME280DataE.csv');
dataGPS = csvread('log02/log02_N9Boardcore12UbloxGPSDataE.csv');

pkg load signal

yBME = medfilt1(dataBME(:, 4), 10)

plot(dataGPS(:, 1),dataGPS(:, 4));
