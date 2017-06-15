classdef EKF_WITH_KNOWN_CORRESPONDENCE < handle
    %UNTITLED4 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        x;  % State vector
        P;  % Covariance matrix
        Q;  % Process noise covariance matrix
        s;  % Landmark signature vector
        
        % EKF Parameter Values
        C = 0.2;    % Process Noise Constant
        Rc = [.01,5];   % Measurement Noise Constants
        s_cost=.00000000001;
        % Landmark append threshold
        s_thresh=1000000000
        
        landmark_list;
    end
    
    methods
        % Constructor - initialize state variables and covariances
        function h = EKF_WITH_KNOWN_CORRESPONDENCE()
            % State Vector
            h.x = [0,0,0];
            % Covariance Matrix
            h.P = eye(length(h.x)).*0.1;
            h.P(1,1) = 0.1; P(2,2) = 0.1; P(3,3) = 0.1;
            
            h.landmark_list = RANSAC();
        end
        
        % Prediction Phase of EKF
        % Pass current state vector, covariance matrix, control vector, and
        % prediction noise covariance matrix. Returns predicted state
        % vector and covariance matrix.
        function EKF_SLAM_Prediction(h,u)
            % Get noise covariance matrix for control signal
            W = [u(1)*cosd(h.x(3)) u(1)*sind(h.x(3)) u(2)]';
            h.Q = zeros(size(h.P));
            h.Q(1:3,1:3) = W*h.C*W';
            
            [h.x,F] = h.f(h.x,u);
            h.P = F*h.P*F' + h.Q;
            
            % Safety first! Ensure orientation doesn't pass 360:
            h.x(3) = wrapTo360(h.x(3));
        end
        
        % Non-linear prediction model
        % Pass current state vector and control vector. Returns predicted
        % state vector and Jacobian of prediction model.
        function [x_new,F] = f(h,x,u)
            x_new = x;
            x_new(1:3) = [x(1) + u(1)*cosd(x(3)+u(2)); ...
                x(2) + u(1)*sind(x(3)+u(2)); ...
                x(3) + u(2)];
            % Jacobian F
            F = eye(length(x));
            F(1,3) = -1*u(1)*sind(x(3));
            F(2,3) = u(1)*cosd(x(3));
        end
        
        % Measurement Phase of EKF
        % Pass current state vector, covariance matrix, measurement vector,
        % measurement noise covariance matrix, and landmark association.
        function EKF_SLAM_Measurement(h,z,R,idx)
            [x_mm,H] = h.h(h.x,idx);
            y = z' - x_mm';
            %     y(1) = z(1)' - x_mm(1)';
            y(2) = angdiff(deg2rad(x_mm(2)),deg2rad(z(2)));
            S = H*h.P*H' + R;
            K = h.P*H'*(S\eye(size(S)));
            h.x = h.x + (K*y)';
            h.P = (eye(size(K*H)) - K*H)*h.P;
        end
        
        % Non-linear Measurement model
        % Pass current state vector, landmark index/correspondence. Returns
        % the state variables in the measurement space and the Jacobian of
        % the measurement model.
        function [x_measure,H] = h(h,x,idx)
            lmx = x((idx-1)*2 + 4);
            lmy = x((idx-1)*2 + 5);
            % idx is the index of the observed node
            x_measure(1) = sqrt((x(1) - lmx)^2 + (x(2) - lmy)^2);
            x_measure(2) = atan2d((lmy - x(2)),(lmx - x(1))) - x(3);
            
            % Jacobian of h
            H = zeros(2,length(x));
            H(1,1) = (x(1) - lmx)/sqrt((lmx - x(1))^2 + (lmy - x(2))^2);
            H(1,2) = (x(2) - lmy)/sqrt((lmx - x(1))^2 + (lmy - x(2))^2);
            H(1,(idx-1)*2 + 4) = -((x(1) - lmx)/sqrt((x(1) - lmx)^2 + (x(2) - lmy)^2));
            H(1,(idx-1)*2 + 5) = -((x(2) - lmy)/sqrt((x(1) - lmx)^2 + (x(2) - lmy)^2));
            
            H(2,1) = (lmy - x(2))/((lmx - x(1))^2 + (lmy - x(2))^2);
            H(2,2) = (lmx - x(1))/((lmx - x(1))^2 + (lmy - x(2))^2);
            H(2,3) = -1;
            H(2,(idx-1)*2 + 4) = -((lmy - x(2))/((lmx - x(1))^2 + (lmy - x(2))^2));
            H(2,(idx-1)*2 + 5) = -((lmx - x(1))/((lmx - x(1))^2 + (lmy - x(2))^2));
        end
        
        function append(h,u,R,landmarkPos,signature)
            % Append signature
            %--------------------------------------------------------------
            h.s = [h.s;signature];
            
            % Append landmark x,y position
            %--------------------------------------------------------------
            numOfLandmarks = (length(h.x) - 3) / 2;
            
            n = length(h.P);
            
            % Append landmark to x
            h.x = [h.x , landmarkPos(1), landmarkPos(2)];
            
            % Append landmark covariances to P
            %--------------------------------------------------------------
            % Get SLAM-Specific Jacobians (as defined by SLAM for Dummies!)
            jxr = [1 0 -u(1)*sind(h.x(3)); ...
                0 1  u(1)*cosd(h.x(3))];
            
            jz = [cosd(u(2)) -u(1)*sind(u(2)); ...
                sind(u(2)) u(1)*cosd(u(2))];
            
            % Append landmark to P (again as defined by SLAM for Dummies)
            h.P(n+1:n+2,n+1:n+2) = jxr*h.P(1:3,1:3)*jxr' + jz*R*jz';    %C
            h.P(1:3,n+1:n+2) = h.P(1:3,1:3)*jxr';                       %I
            h.P(n+1:n+2,1:3) = h.P(1:3,n+1:n+2)';                       %H
            for idx = 1: numOfLandmarks
                h.P((n+1):(n+2),((idx-1)*2+4):((idx-1)*2+5)) = jxr*h.P(((idx-1)*2+4):((idx-1)*2+5),1:3)';   %F
                h.P(((idx-1)*2+4):((idx-1)*2+5),(n+1):(n+2)) = h.P((n+1):(n+2),((idx-1)*2+4):((idx-1)*2+5))';%G
            end
        end
        
        function measureUnknownCorrespondence(h, laserData, u)
            % Search for landmarks
            [observed_LL] = h.landmark_list.getLandmark(laserData,h.x);
            
            % Apply measurement update in EKF if landmarks are observed
            if(~isempty(observed_LL))
                numOfObservedLandmarks = size(observed_LL,1);
                for ii = 1:numOfObservedLandmarks  % "8: For all observed features..."
                    R = zeros(2,2); R(1,1) = observed_LL(ii,1)*h.Rc(1); R(2,2) = observed_LL(ii,2)*h.Rc(2);
                    %check if state has no landmarks
                    if(length(h.x)<4)
                        h.append(u,R,h.landmark_list.landmark(find([h.landmark_list.landmark.index])).loc,1);
                    else
                        %estimate correspondence
                        
                        % Measurement vector (Range and relative orientation)
                        z = observed_LL(ii,:);
                        numOfLandmarks=(length(h.x)-3)/2;
                        if(z(3)>numOfLandmarks)
                            %append new landmark
                            h.append(u,R,h.landmark_list.landmark(find([h.landmark_list.landmark.index]==idx)).loc,idx);
                        else
                            mu = h.x(1:2)' + z(1)*[cosd(z(2) + h.x(3));sind(z(2) + h.x(3))];
                            mu_k = [h.x(4+(idx-1)*2);h.x(4+(idx-1)*2+1)];
                            delta_k=mu_k-h.x(1:2)';
                            q_k=delta_k'*delta_k;
                            
                            %Line 13
                            z_k=[sqrt(q_k); wrapTo360(atan2d(delta_k(2),delta_k(1))-(h.x(3)))];
                            
                            %Line 14
                            numOfLandmarks=(length(h.x)-3)/2;
                            F_k = zeros(5,numOfLandmarks*2+3);F_k(1:3,1:3) = eye(3);F_k(4:5,(4+(idx-1)*2):(5+(idx-1)*2)) = eye(2);
                            
                            %Line 15
                            H_k = (1/q_k)*[-sqrt(q_k)*delta_k(1), -sqrt(q_k)*delta_k(2), 0, sqrt(q_k)*delta_k(1), sqrt(q_k)*delta_k(2); ...
                                delta_k(2), -delta_k(1), -q_k, -delta_k(2), delta_k(1)]*F_k;
                            
                            %Line 16
                            phi_k = H_k*h.P*H_k' + R;
                            
                            K=h.P*H_k'*phi_k^-1;
                            h.x = h.x + (K*(z(1:2)' - z_k))';
                            h.P = (eye(size(h.P)) - K*H_k)*h.P;
                        end
                        
                    end
                end
            end
        end
    end
end

