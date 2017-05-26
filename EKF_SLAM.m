classdef EKF_SLAM < handle
    %UNTITLED4 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        x;  % State vector
        P;  % Covariance matrix
        Q;  % Process noise covariance matrix
        s;  % Landmark signature vector
        
        % EKF Parameter Values
        C = 0.2;    % Process Noise Constant
        Rc = [10,1];   % Measurement Noise Constants
        
        % Landmark append threshold
        alpha = 1;
        
        landmark_list;
    end
    
    methods
        % Constructor - initialize state variables and covariances
        function h = EKF_SLAM()
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
        
        function [idx] = argmin(h, pi)
            idx = find(pi == min(pi));
        end
        
        function measureFromLandmarks(h, laserData, u)
            % Search for landmarks
            [observed_LL] = h.landmark_list.getLandmark(laserData,h.x);
            
            % Apply measurement update in EKF if landmarks are observed
            if(~isempty(observed_LL))
                numOfObservedLandmarks = size(observed_LL,1);
                for ii = 1:numOfObservedLandmarks  % "8: For all observed features..."
                    % Measurement vector (Range and relative orientation)
                    z = observed_LL(ii,:);
                    % Get expected x,y position of observed landmark
                    % (r,theta -> x,y)
                    x_bar = [h.x(1:2),z(3)] + z(1)*[cosd(z(2) + h.x(3));sind(z(2) + h.x(3)); 0]; % Line 9
                    % Loop through all landmarks in state vector
                    numOfLandmarks = (length(h.x)-3)/2;
                    if(numOfLandmarks == 0)
                        R = zeros(2,2); R(1,1) = observed_LL(ii,1)*h.Rc(1); R(2,2) = observed_LL(ii,2)*h.Rc(2);
                        h.append(u,R,h.landmark_list.landmark(find([h.landmark_list.landmark.index])).loc,1);
                    else
                        for jj = 1:numOfLandmarks
                            % Calculate difference between expected location
                            % and state vector position
                            delta_k = [h.x((jj-1)*2 + 4) - x_bar(1); h.x((jj-1)*2 + 5) - x_bar(2)];
                            q_k = delta_k'*delta_k;
                            % state vector landmark x,y postion ->
                            % range,bearing, signature
                            z_hat(jj) = {[sqrt(q_k);angdiff(h.x(3),atan2d(delta_k(2),delta_k(1)));h.s(jj)]};
                            % DIMENSION MISMATCH HERE
                            F = zeros(size(h.P));F(1:3,1:3) = eye(3);F(4+(jj-1)*3:4+(jj-1)*3+3,4+(jj-1)*3:4+(jj-1)*3+3) = eye(3);

                            H(jj) = {(1/q_k)*[-sqrt(q_k)*delta_k(1), -sqrt(q_k)*delta_k(2), 0, sqrt(q_k)*delta_k(1), sqrt(q_k)*delta_k(2), 0; ...
                                         delta_k(2), -delta_k(1), -q_k, -delta_k(2), delta_k(1), 0;
                                         0, 0, 0, 0, 0, q_k]*F};
                            R = zeros(2,2); R(1,1) = observed_LL(ii,1)*h.Rc(1); R(2,2) = observed_LL(ii,2)*h.Rc(2);
                            S(jj) = {cell2mat(H(jj))*h.P*cell2mat(H(jj))' + R};
                            pi(jj) = (z - cell2mat(z_hat(jj)))'*inv(cell2mat(S(jj)))*(z - cell2mat(z_hat(jj)));
                        end

                        idx = argmin(pi);
                        if(idx >= h.alpha)
                            h.append(u,idx,R,h.landmark_list.landmark(idx).loc);
                        else
                            K = h.P*cell2mat(H(idx))'*inv(cell2mat(S(idx)));
                            h.x = h.x + K*(z - cell2mat(z_hat(idx)));
                            h.P = (eye(size(h.P)) - K*cell2mat(H(idx)))*h.P;
                        end

%                         % Measurement vector (Range and relative orientation)
%                         z = [observed_LL(ii,1), observed_LL(ii,2)];
%                         % Measurement noise covariance matrix
%                         R = zeros(2,2); R(1,1) = observed_LL(ii,1)*h.Rc(1); R(2,2) = observed_LL(ii,2)*h.Rc(2);
%                         % Landmark index / correspondence
%                         idx = observed_LL(ii,3);
%                         
%                         % if landmark is new, append to x and P
%                         idx2 = find([h.landmark_list.landmark(:).index]==idx);  % Landmark of correspondence idx
%                         h.append(u,idx,R,h.landmark_list.landmark(idx2).loc);
%                         
%                         % Apply EKF measurement update
%                         h.EKF_SLAM_Measurement(z,R,idx);
                    end
                end
            end
            
            
        end
    end
end

