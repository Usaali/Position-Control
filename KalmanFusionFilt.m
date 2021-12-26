classdef KalmanFusionFilt < handle
    %KALMANFUSIONFILT A Multisensor Kalman Filter implementation
    %   Detailed explanation goes here
    
    properties
        x %state vector [6,1]: [vx,vy,x,y,xbias,ybias]
        P %covariance matrix [6,6]
        Q %process covariance
        R %measurement covariance
        tx % state vector for theta estimation [theta, thetabias]
        thetaP
        thetaQ
        thetaR
    end
    
    methods
        function obj = KalmanFusionFilt()
            %KALMANFUSIONFILT Construct an instance of this class
            %   Detailed explanation goes here
            obj.x = zeros(6,1);
            obj.P = eye(6) * 1;
            obj.Q = eye(6)*1e-5;
            obj.R = diag([1e-2,1e-2]);
            obj.tx = zeros(2,1); 
            obj.thetaP = eye(2);
            obj.thetaQ = diag([1e-4,1e-4]);
            obj.thetaR = 1e-2;
        end
        
        function predictTheta(obj,T, w)
            A = [1,-T;0,1];
            B = [T;0];
            obj.tx = A*obj.tx + B*w;
            obj.thetaP = A*obj.thetaP*A' + obj.thetaQ;
        end
        
        function correctTheta(obj, senstheta)
            %C is 1
            if abs(senstheta - obj.tx(1)) > pi
                obj.tx(1) = obj.tx(1) + 2*pi * sign(senstheta);
            end
            C = [1,0];
            y = senstheta - C*obj.tx; %Innovation
            S = C*obj.thetaP*C' + obj.thetaR; %innovation Cov
            K = (obj.thetaP*C')/S; %kalman Gain
            obj.tx = obj.tx + K*y; %state update
            obj.thetaP = (eye(2)-K*C)*obj.thetaP; %cov update
        end
        
        function predict(obj, T, ax, ay)
            theta = obj.tx(1);
            A = eye(6);
            A(1,5) = -T*cos(theta);
            A(1,6) = T*sin(theta);
            A(2,5) = -T*sin(theta);
            A(2,6) = -T*cos(theta);
            A(3,1) = T;
            A(4,2) = T;
            B = [cos(theta),-sin(theta);sin(theta),cos(theta);T/2,0;0,T/2;0,0;0,0];
            obj.x = A*obj.x + B*T*[ax;ay];
            obj.P = A*obj.P*A' + obj.Q;
        end
        
        function correct(obj, sensx, sensy)
            C = zeros(2,6);
            C(:,3:4) = eye(2);
            y = [sensx, sensy]' - C*obj.x; %Innovation
            S = C*obj.P*C' + obj.R; %innovation Cov
            K = (obj.P*C')/S; %kalman Gain
            obj.x = obj.x + K*y; %state update
            obj.P = (eye(6)-K*C)*obj.P; %cov update
        end
        
        function pos = getPos(obj)
            pos = [obj.x(3);obj.x(4)];
        end
        
        function vel = getVel(obj)
            vel = [obj.x(1);obj.x(2)];
        end
        
        function ang = getAng(obj)
            ang = obj.tx(1);
        end
        
    end
end

