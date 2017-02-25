function [x_new_new, P] = EKF(x,P,z,u,idx,R,Q,landmarkObserved)
    %Prediction
    [x_new,F] = f(x,u);
    P = F*P*F' + Q;
    
    if(landmarkObserved == 1)
        % Measurement Update
        [x_mm,H] = h(x_new,idx);
        y = z' - x_mm';
        S = H*P*H' + R;
        K = P*H'*(S\eye(size(S)));
        x_new_new = x_new + (K*y)';
        P = (eye(size(K*H)) - K*H)*P;
    end
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

function [x_measure,H] = h(x,idx)
    lmx = x((idx-1)*2 + 4);
    lmy = x((idx-1)*2 + 5);
    % idx is the index of the observed node
    x_measure(1) = sqrt((x(1) - lmx)^2 + (x(2) - lmy)^2);
    x_measure(2) = atand((lmy - x(2))/(lmx - x(1))) - x(3);
    % Jacobian of h
    H = zeros(2,length(x));
    H(1,1) = (x(1) - lmx)/sqrt((lmx - x(1))^2 + (lmy - x(2))^2);
    H(1,2) = (x(2) - lmy)/sqrt((lmx - x(1))^2 + (lmy - x(2))^2);
    H(1,(idx-1)*2 + 4) = -(x(1) - lmx)/sqrt((x(1) - lmx)^2 + (x(2) - lmy)^2);
    H(1,(idx-1)*2 + 5) = -(x(2) - lmy)/sqrt((x(1) - lmx)^2 + (x(2) - lmy)^2);
    
    H(2,1) = (lmy - x(2))/(lmx - x(1))^2 + (lmy - x(2))^2;
    H(2,2) = (lmx - x(1))/(lmx - x(1))^2 + (lmy - x(2))^2;
    H(2,(idx-1)*2 + 4) = -((lmy - x(2))/(lmx - x(1))^2 + (lmy - x(2))^2);
    H(2,(idx-1)*2 + 5) = -((lmx - x(1))/(lmx - x(1))^2 + (lmy - x(2))^2);
end