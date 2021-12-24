close all;
clear all;
%% Create IMU Sensor object
% Usage: [measuredAcceleration, measuredAngularVelocity] = IMU(acceleration, angularVelocity)
%accparams = accelparams('MeasurementRange',19.6,'Resolution',0.598e-3,'ConstantBias',0.49,'NoiseDensity',3920e-6,'TemperatureBias',0.294,'TemperatureScaleFactor',0.02,'AxesMisalignment',2);
%IMU = imuSensor('Accelerometer', accparams); % realistic imu sensor
imuFs = 100;
IMU = imuSensor('SampleRate',imuFs); % ideal imu sensor
%% Create a dummy trajectory
 wp = [0,0,0;10,0,0;10,10,0;0,10,0;0,0,0]; % Waypoints
 traj = waypointTrajectory(wp,0:4,'SampleRate',100);
 
 %% Get data out of trajectory
 traj.reset()
 [pos,orient,vel,acc,angVel] = traj.step();
 ang = quat2eul(orient);
 cnt = 1;
 spf = traj.SamplesPerFrame;
 while ~isDone(traj)
     idx = (cnt+1):(cnt+spf);
     [pos(idx,:),orient(idx,:),vel(idx,:),acc(idx,:),angVel(idx,:)] = traj.step();
     ang(idx,:) = quat2eul(orient(idx,:));
     cnt = cnt+1;
 end
 
 
%% Plot trajectory and points
figure('Name','Groundtruth Position', 'NumberTitle','off');
plot(pos(:,1),pos(:,2), wp(:,1),wp(:,2), '--o')
title('Position')
xlabel('X (m)')
ylabel('Y (m)')
legend({'Position', 'Waypoints'})
axis equal
figure('Name','Groundtruth Acceleration', 'NumberTitle','off');
subplot(3,1,1); 
plot(acc(:,1))
title('X-Acceleration')
xlabel('Time')
ylabel('Acc (m/s^2)')
subplot(3,1,2);
plot(acc(:,2))
title('Y-Acceleration')
xlabel('Time')
ylabel('Acc (m/s^2)')
subplot(3,1,3);
plot(acc(:,3))
title('Z-Acceleration')
xlabel('Time')
ylabel('Acc (m/s^2)')

figure('Name','Groundtruth Speed', 'NumberTitle','off');
subplot(3,1,1); 
plot(vel(:,1))
title('X-Speed')
xlabel('Time')
ylabel('Acc (m/s)')
subplot(3,1,2);
plot(vel(:,2))
title('Y-Speed')
xlabel('Time')
ylabel('Acc (m/s)')
subplot(3,1,3);
plot(vel(:,3))
title('Z-Speed')
xlabel('Time')
ylabel('Acc (m/s)')

%% get IMU Data
[accm, angVelm] = IMU(acc, angVel);

figure('Name','IMU Acceleration', 'NumberTitle','off');
subplot(3,1,1); 
plot(accm(:,1))
title('X-Acceleration')
xlabel('Time')
ylabel('Acc (m/s^2)')
subplot(3,1,2);
plot(accm(:,2))
title('Y-Acceleration')
xlabel('Time')
ylabel('Acc (m/s^2)')
subplot(3,1,3);
plot(accm(:,3))
title('Z-Acceleration')
xlabel('Time')
ylabel('Acc (m/s^2)')

%% Add gaussian noise to the position
imuPerGlob = 1;

var = 0.2;
rPos = [normrnd(pos(1,1),var),normrnd(pos(1,2),var),normrnd(pos(1,3),var)];
for i=2:size(pos,1)
    if mod(i-1,imuPerGlob)==0
        rPos(((i-1)/imuPerGlob)+1,:) = [normrnd(pos(i,1),var),normrnd(pos(i,2),var),normrnd(pos(i,3),var)];
    end
end

%figure('Name','Vision Position', 'NumberTitle','off');
%plot(rPos(:,1),rPos(:,2), wp(:,1),wp(:,2), '--o')

filt = KalmanFusionFilt()
%filt.x(1) = vel(1,1);
%filt.x(2) = vel(1,2);

% numImuSamples = size(accm,1);
% estPos = zeros(numImuSamples,2);
% estVel = zeros(numImuSamples,2);
% estAng = zeros(numImuSamples,2);
% gpsIdx = 1;
% for idx = 1:numImuSamples
%     if(mod(idx,imuPerGlob)==0)        
%         filt.correct(angVel(idx,3),rPos(gpsIdx,1), rPos(gpsIdx,2),ang(gpsIdx,1))
%         gpsIdx = gpsIdx + 1;
%     end
%     filt.predict(1/100,accm(idx,1),accm(idx,2));
%     
%     estPos(idx,:) = filt.getPos();
%     estVel(idx,:) = filt.getVel();
%     estAng(idx,:) = filt.getAng();
% end
% figure('Name','Estimated Position', 'NumberTitle','off');
% plot(estPos(:,1),estPos(:,2), wp(:,1),wp(:,2), '--o', pos(:,1),pos(:,2),'r')
% 
% figure('Name','Estimated Angle', 'NumberTitle','off');
% hold('on')
% plot(estAng(:,2))
% plot(ang(:,1),'r')
% title('Theta')
% xlabel('Time')
% 
% figure('Name','Angular velocity', 'NumberTitle','off');
% subplot(2,1,1); 
% plot(angVel(:,3))
% title('Real angular velocity (yaw)')
% xlabel('Time')
% ylabel('Vel (rad/s^2)')
% subplot(2,1,2);
% plot(angVelm(:,3))
% title('Measured angular velocity (yaw)')
% xlabel('Time')
% ylabel('Vel (rad/s^2)')
% 
% intVel = cumsum(angVel(:,3))*0.01;
% figure('Name','Angular velocity Integrated', 'NumberTitle','off');
% plot(intVel)
% 
% a = sqrt(vel(:,1).^2 + vel(:,2).^2);
% figure('Name','Speed normalized', 'NumberTitle','off');
% plot(a)
xAcc = zeros(size(acc,1),1);
yAcc = zeros(size(acc,1),1);
for i = 1:size(acc,1)
    temp = [acc(i,1),acc(i,2)]*[cos(ang(i,1)),-sin(ang(i,1));sin(ang(i,1)),cos(ang(i,1))];
    xAcc(i) = temp(1);
    yAcc(i) = temp(2);
end

figure('Name','Calc Acceleration', 'NumberTitle','off');
subplot(2,1,1); 
plot(xAcc)
title('X-Acceleration')
xlabel('Time')
ylabel('Acc (m/s^2)')
subplot(2,1,2);
plot(yAcc)
title('Y-Acceleration')
xlabel('Time')
ylabel('Acc (m/s^2)')
