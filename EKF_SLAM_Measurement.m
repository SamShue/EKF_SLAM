function [x_new, P_new] = EKF_SLAM_Measurement(x,P,z,R,idx)
% Measurement Phase of EKF
% Pass current state vector, covariance matrix, measurement vector,
% measurement noise covariance matrix, and landmark association.
[x_mm,H] = h(x,idx);
y = z' - x_mm';
%     y(1) = z(1)' - x_mm(1)';
y(2) = angdiff(deg2rad(x_mm(2)),deg2rad(z(2)));
S = H*P*H' + R;
K = P*H'*(S\eye(size(S)));
x_new = x + (K*y)';
P_new = (eye(size(K*H)) - K*H)*P;
end

function [x_measure,H] = h(x,idx)
% Non-linear Measurement model
% Pass current state vector, landmark index/correspondence. Returns
% the state variables in the measurement space and the Jacobian of
% the measurement model.
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