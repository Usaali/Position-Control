accy = ones(1000,1);
vel = cumsum(accy)*0.1;
pos = cumsum(vel)*0.1;
figure('Name','Input signal', 'NumberTitle','off');
subplot(3,1,1)
plot(accy)
title('Acceleration')
subplot(3,1,2)
plot(vel)
title('Velocitz')
subplot(3,1,3)
plot(pos)
title('Position')

filt = KalmanFusionFilt()
filt.x(1) = 5;
theta = 0:2*pi/1000:2*pi;
%filt.x(2) = vel(1,2);

numImuSamples = size(accy,1);
estPos = zeros(numImuSamples,2);
estVel = zeros(numImuSamples,2);
estAng = zeros(numImuSamples,2);
gpsIdx = 1;
for idx = 1:numImuSamples
    if(mod(idx,1)==0)        
        %filt.correct(angVel(idx,3),rPos(gpsIdx,1), rPos(gpsIdx,2),ang(gpsIdx,1))
        gpsIdx = gpsIdx + 1;
    end
    filt.predict(1/100,0,accy(idx),theta(idx));
    
    estPos(idx,:) = filt.getPos();
    estVel(idx,:) = filt.getVel();
    estAng(idx,:) = filt.getAng();
end
figure('Name','Estimated Position', 'NumberTitle','off');
hold('on')
%plot(estPos(:,1),estPos(:,2))
quiver(estPos(:,1),estPos(:,2),estPos(:,1),estPos(:,2));
%plot(pos,'r')
figure('Name','Estimated Speed', 'NumberTitle','off');
hold('off')
subplot(2,1,1)
plot(estVel(:,1))
subplot(2,1,2)
plot(estVel(:,2))
