nas = readtable('log09/log09_Boardcore::NASState.csv');
imu = readtable('log09/log09_Boardcore::MPU9250Data.csv');
baro = readtable('log09/log09_Boardcore::BME280Data.csv');
gps = readtable('log09/log09_Boardcore::UBXGPSData.csv');

gps = gps(gps.gpsTimestamp / 1e6 > 1750, :);
nas = nas(nas.timestamp / 1e6 > 1750, :);
imu = imu(imu.accelerationTimestamp / 1e6 > 1750, :);

figure
plot(nas.timestamp / 1e6, nas.vd)
plot(imu.accelerationTimestamp / 1e6, movmean(imu.accelerationX, 100))
plot(imu.accelerationTimestamp / 1e6, movmean(imu.accelerationY, 100))
plot(imu.accelerationTimestamp / 1e6, movmean(imu.accelerationZ, 100))
hold on
grid on
legend('NAS vertical velocity', 'Acceleration X', 'Acceleration Y', 'Acceleration Z')

figure
plot(gps.gpsTimestamp / 1e6, gps.height)
hold on
grid on

figure
plot3(gps.latitude, gps.longitude, gps.height)
grid on