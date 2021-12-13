close all;
%% Create IMU Sensor object
% Usage: [measuredAcceleration, measuredAngularVelocity] = IMU(acceleration, angularVelocity)
IMU = imuSensor(); % ideal imu sensor

%% Create a dummy trajectory
 wp = [0,0,0;10,0,0;10,10,0;0,10,0;0,0,0]; % Waypoints
 toa = 0:4;
 traj = waypointTrajectory(wp,toa);
 
 %% Get data out of trajectory
 traj.reset()
 [pos,orient,vel,acc,angVel] = traj.step();
 cnt = 1;
 spf = traj.SamplesPerFrame;
 while ~isDone(traj)
     idx = (cnt+1):(cnt+spf);
     [pos(idx,:),orient(idx,:),vel(idx,:),acc(idx,:),angVel(idx,:)] = traj.step();
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
imuPerGlob = 4;

var = 0.2;
rPos = [normrnd(pos(1,1),var),normrnd(pos(1,2),var),normrnd(pos(1,3),var)];
for i=2:size(pos,1)
    if mod(i-1,imuPerGlob)==0
        rPos(((i-1)/imuPerGlob)+1,:) = [normrnd(pos(i,1),var),normrnd(pos(i,2),var),normrnd(pos(i,3),var)];
    end
end

figure('Name','Vision Position', 'NumberTitle','off');
plot(rPos(:,1),rPos(:,2), wp(:,1),wp(:,2), '--o')