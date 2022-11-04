clc;
clear all;
close all;


x = [0,0,0];;  % State vector
P = eye(length(h.x)).*0.1;  % Covariance matrix
P(1,1) = 0.1; P(2,2) = 0.1; P(3,3) = 0.1;
Q;  % Process noise covariance matrix
s;  % Landmark signature vector

% EKF Parameter Values
C = 0.2;    % Process Noise Constant
Rc = [.01,5];   % Measurement Noise Constants
s_cost=.00000000001;
% Landmark append threshold
s_thresh=1000000000

landmark_list = RANSAC();
observed



function [x, P] = predict(x, u, P, C)
    % Get noise covariance matrix for control signal
    W = [u(1)*cosd(x(3)) u(1)*sind(x(3)) u(2)]';
    Q = zeros(size(P));
    Q(1:3,1:3) = W*C*W';
    
    [x,F] = f(x,u);
    P = F*P*F' + Q;
    
    % Safety first! Ensure orientation doesn't pass 360:
    x(3) = wrapTo360(x(3));
end

 function [x_new,F] = f(x,u)
    x_new = x;
    x_new(1:3) = [x(1) + u(1)*cosd(x(3)+u(2)); ...
        x(2) + u(1)*sind(x(3)+u(2)); ...
        x(3) + u(2)];
    % Jacobian F
    F = eye(length(x));
    F(1,3) = -1*u(1)*sind(x(3));
    F(2,3) = u(1)*cosd(x(3));
end