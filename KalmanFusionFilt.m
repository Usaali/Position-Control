classdef KalmanFusionFilt < handle
    %KALMANFUSIONFILT A Multisensor Kalman Filter implementation
    %   Detailed explanation goes here
    
    properties
        x %state vector [6,1]: [vx,vy,x,y,w,theta]
        P %covariance matrix [6,6]
        Q %process covariance
        R %measurement covariance
    end
    
    methods
        function obj = KalmanFusionFilt()
            %KALMANFUSIONFILT Construct an instance of this class
            %   Detailed explanation goes here
            obj.x = zeros(6,1);
            obj.P = eye(6) * 1;
            obj.Q = eye(6)*1e-4;
            obj.R = eye(3)*0.002;
        end
        
        function predict(obj, T, ax, ay, theta)
            
            %theta = obj.x(6);
            A = eye(6);
            A(1,6) = (-ax*sin(theta)-ay*cos(theta)) * T;
            A(2,6) = (ax*cos(theta)-ay*sin(theta)) * T;
            A(3,1) = T;
            A(4,2) = T;
            A(6,5) = T;
            B = [cos(theta),-sin(theta);sin(theta),cos(theta);0,0;0,0;0,0;0,0];
            obj.x = A*obj.x + B*T*[ax,ay]';
%             obj.x(3) = obj.x(3) + obj.x(1)*T;
%             obj.x(4) = obj.x(4) + obj.x(2)*T;
%             obj.x(6) = obj.x(6) + obj.x(5)*T;
%             obj.x(1) = obj.x(1) + (ax*cos(theta)-ay*sin(theta)) * T;
%             obj.x(2) = obj.x(2) + (ax*sin(theta)+ay*cos(theta)) * T;
            
            obj.P = A*obj.P*A' + obj.Q;
        end
        
        function correct(obj, w, sensx, sensy, senstheta)
            C = zeros(3,6);
            C(:,3:5) = eye(3);
           % C(6,6)=0;
            y = [sensx, sensy, w]' - C*obj.x; %Innovation
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
            ang = [obj.x(5);obj.x(6)];
        end
        
    end
end

