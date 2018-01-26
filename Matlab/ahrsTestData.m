% ahrsTestData.m
%
% This script utilizes logged data from an IMU (LSM9DS1) to test if the
% Madgwick algorithm works with the raw data output of the sensor. This is
% currently just for debugging purposes to help narrow down where an error
% is occuring on some embedded code.

%% Start of script

addpath('quaternion_library');      % include quaternion library
addpath('TestData/RollThenPitch');   % include raw test data from IMU
close all;                          % close all figures
clear;                              % clear all variables
clc;                                % clear the command terminal

%% Import and plot sensor data
readLen = 4000;
time = (0:readLen)';

ax = csvread('ax.csv', 0, 1, [0, 1, readLen, 1]); ax = ax / 9.8;
ay = csvread('ay.csv', 0, 1, [0, 1, readLen, 1]); ay = ay / 9.8;
az = csvread('az.csv', 0, 1, [0, 1, readLen, 1]); az = az / 9.8;
Accelerometer = horzcat(ax, ay, az);

gx = csvread('gx.csv', 0, 1, [0, 1, readLen, 1]);
gy = csvread('gy.csv', 0, 1, [0, 1, readLen, 1]);
gz = csvread('gz.csv', 0, 1, [0, 1, readLen, 1]);
Gyroscope = horzcat(gx, gy, gz);

mx = csvread('mx.csv', 0, 1, [0, 1, readLen, 1]);
my = csvread('my.csv', 0, 1, [0, 1, readLen, 1]);
mz = csvread('mz.csv', 0, 1, [0, 1, readLen, 1]);
Magnetometer = horzcat(mx, my, mz);

figure('Name', 'Sensor Data');
axis(1) = subplot(3,1,1);
hold on;
plot(time, Gyroscope(:,1), 'r');
plot(time, Gyroscope(:,2), 'g');
plot(time, Gyroscope(:,3), 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Angular rate (deg/s)');
title('Gyroscope');
hold off;
axis(2) = subplot(3,1,2);
hold on;
plot(time, Accelerometer(:,1), 'r');
plot(time, Accelerometer(:,2), 'g');
plot(time, Accelerometer(:,3), 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Acceleration (g)');
title('Accelerometer');
hold off;
axis(3) = subplot(3,1,3);
hold on;
plot(time, Magnetometer(:,1), 'r');
plot(time, Magnetometer(:,2), 'g');
plot(time, Magnetometer(:,3), 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Flux (G)');
title('Magnetometer');
hold off;
linkaxes(axis, 'x');

%% Process sensor data through algorithm

AHRS = MadgwickAHRS('SamplePeriod', 1/75, 'Beta', 0.1);
%AHRS = MahonyAHRS('SamplePeriod', 1/256, 'Kp', 0.5);

quaternion = zeros(length(time), 4);
for t = 1:length(time)
    AHRS.Update(Gyroscope(t,:) * (pi/180), Accelerometer(t,:), Magnetometer(t,:));	% gyroscope units must be radians
    quaternion(t, :) = AHRS.Quaternion;
end

%% Plot algorithm output as Euler angles
% The first and third Euler angles in the sequence (phi and psi) become
% unreliable when the middle angles of the sequence (theta) approaches ±90
% degrees. This problem commonly referred to as Gimbal Lock.
% See: http://en.wikipedia.org/wiki/Gimbal_lock

euler = quatern2euler(quaternConj(quaternion)) * (180/pi);	% use conjugate for sensor frame relative to Earth and convert to degrees.

figure('Name', 'Euler Angles');
hold on;
plot(time, euler(:,1), 'r');
plot(time, euler(:,2), 'g');
plot(time, euler(:,3), 'b');
title('Euler angles');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('\phi', '\theta', '\psi');
hold off;