close all;
clear all;

%% Create a dummy trajectory
 %wp = [0,0,0;10,0,0;13,10,0;0,10,0]; % Waypoints
 %wp = [wp;wp;wp;wp;wp;wp;wp;0,0,0 ];
 frequency = 300;
 wp = [0,0,0;0,5,0;5,5,0;13,7,0;15,15,0;12,15,0;12,11,0;16,11,0;16,8,0;4,8,0;4,20,0;0,20,0;3,10,0;0,0,0]
 traj = waypointTrajectory(wp,0:size(wp,1)-1,'SampleRate',frequency);
 
 %% Get data out of trajectory
 traj.reset()
 [pos,orient,vel,acc,angVel] = traj.step();
 ang = quat2eul(orient);
 cnt = 1;
 piCnt = 0;
 spf = traj.SamplesPerFrame;
 while ~isDone(traj)
     idx = (cnt+1):(cnt+spf);
     [pos(idx,:),orient(idx,:),vel(idx,:),acc(idx,:),angVel(idx,:)] = traj.step();
     ang(idx,:) = quat2eul(orient(idx,:));
     cnt = cnt+1;
 end

%% Create Imu data 
accm = zeros(size(acc,1),2);

for i = 1:size(acc,1)
    rotMat = [cos(ang(i,1)),sin(ang(i,1));-sin(ang(i,1)),cos(ang(i,1))];
    temp = rotMat*[acc(i,1);acc(i,2)];
    accm(i,:) = [temp(1), temp(2)];
end


%% Add gaussian noise to the position
posvar = 0.2;
posbias = 0;
rposnoise = normrnd(0,posvar,size(pos,1),size(pos,2));
rposnoise = rposnoise - mean(rposnoise) + posbias;
rPos = pos + rposnoise;

rangvar = 0.15;
rangbias = 0;
rangnoise = normrnd(0,rangvar,size(ang,1),size(ang,2));
rangnoise = rangnoise - mean(rangnoise) + rangbias;
rAng = ang + rangnoise;

accvar = 0.01;
accbias = 0.45;
accnoise = normrnd(accbias,accvar,size(accm,1),size(accm,2));
accnoise = accnoise - mean(accnoise) + accbias;
accm = accm + accnoise;

gyrvar = 0.05;
gyrbias = 0.43;
gyrnoise = normrnd(gyrbias,gyrvar,size(angVel,1),size(angVel,2));
gyrnoise = gyrnoise - mean(gyrnoise) + gyrbias;
angVelm = angVel + gyrnoise;

figure('Name','Vision Position', 'NumberTitle','off');
plot(rPos(:,1),rPos(:,2), wp(:,1),wp(:,2), '--o')
figure('Name','Vision Angle', 'NumberTitle','off');
plot(rAng(:,1))
figure('Name','IMU', 'NumberTitle','off');
subplot(3,1,1); 
plot(accm(:,1))
title('X-Acc')
xlabel('Time')
ylabel('x-acc')
subplot(3,1,2); 
plot(accm(:,2))
title('Y-Acc')
xlabel('Time')
ylabel('y-acc')
subplot(3,1,3);
plot(angVelm(:,3))
title('Measured angular velocity (yaw)')
xlabel('Time')
ylabel('Vel (rad/s^2)')

%% run the filter
filt = KalmanFusionFilt()
filt.x(1) = vel(1,1);
filt.x(2) = vel(1,2);
filt.tx(1) = ang(1,1);

numImuSamples = size(accm,1);
imuPerGlob = 5;
estPos = zeros(numImuSamples,2);
estVel = zeros(numImuSamples,2);
estAng = zeros(numImuSamples,1);

for idx = 1:numImuSamples
    
    filt.predictTheta(1/frequency,angVelm(idx,3));
    
    
    if(mod(idx,imuPerGlob)==0)
        filt.correctTheta(rAng(idx,1));
    end
    filt.predict(1/frequency,accm(idx,1),accm(idx,2));
    if(mod(idx,imuPerGlob)==0)
        filt.correct(rPos(idx,1), rPos(idx,2))
    end
    estPos(idx,:) = filt.getPos();
    estVel(idx,:) = filt.getVel();
    estAng(idx) = filt.getAng();
end
figure('Name','Estimated Position', 'NumberTitle','off');
plot(estPos(:,1),estPos(:,2), wp(:,1),wp(:,2), '--o', pos(:,1),pos(:,2),'r',rPos(:,1),rPos(:,2),'.g');

figure('Name','Estimated Angle', 'NumberTitle','off');
hold('on')
plot(estAng)
plot(ang(:,1),'r')
title('Theta')
xlabel('Time')
figure('Name','Angle Error', 'NumberTitle','off');
plot(estAng-ang(:,1))
title('delta Theta')
xlabel('Time')
figure('Name','Position Error', 'NumberTitle','off');
plot(sqrt((estPos(:,2)-pos(:,2)).^2 + (estPos(:,1)-pos(:,1)).^2))
title('delta pos')
xlabel('Time')
% figure('Name','Angular velocity', 'NumberTitle','off');
% subplot(2,1,1); 
% plot(angVel(:,3))
% title('Real angular velocity (yaw)')
% xlabel('Time')
% ylabel('Vel (rad/s^2)')
% subplot(2,1,2);
% plot(estAng(:,1))
% title('Measured angular velocity (yaw)')
% xlabel('Time')
% ylabel('Vel (rad/s^2)')

% intVel = cumsum(angVel(:,3))*0.01;
% figure('Name','Angular velocity Integrated', 'NumberTitle','off');
% plot(intVel)
% 
% a = sqrt(vel(:,1).^2 + vel(:,2).^2);
% figure('Name','Speed normalized', 'NumberTitle','off');
% plot(a)

