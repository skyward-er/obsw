dataAlgo = readtable('log02/log02_Parafoil::WingAlgorithmData.csv');

figure
hold on
grid on
plot(dataAlgo.WingAlgorithmTimestamp / 1e6, (dataAlgo.targetAngle - dataAlgo.velocityAngle) / pi * 180)
% plot(dataAlgo.WingAlgorithmTimestamp / 1e6, dataAlgo.servo1Angle)
plot(dataAlgo.WingAlgorithmTimestamp / 1e6, dataAlgo.servo2Angle)