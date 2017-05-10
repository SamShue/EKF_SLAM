function [x_new, P_new] = EKF_SLAM_Prediction(x,P,u,Q)
% Prediction Phase of EKF
% Pass current state vector, covariance matrix, control vector, and
% prediction noise covariance matrix. Returns predicted state
% vector and covariance matrix.
[x_new,F] = f(x,u);
P_new = F*P*F' + Q;
end

function [x_new,F] = f(x,u)
% Non-linear prediction model
% Pass current state vector and control vector. Returns predicted
% state vector and Jacobian of prediction model.
x_new = x;
x_new(1:3) = [x(1) + u(1)*cosd(x(3)+u(2)); ...
    x(2) + u(1)*sind(x(3)+u(2)); ...
    x(3) + u(2)];
% Jacobian F
F = eye(length(x));
F(1,3) = -1*u(1)*sind(x(3));
F(2,3) = u(1)*cosd(x(3));
end
