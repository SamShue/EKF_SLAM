clc;                                %clear command window
clear all;
close all;                          %close all figures
rosshutdown                         %close current ros incase it is already initalized 

% Robot network variables
ipaddress = 'http://192.168.1.13:11311';         %define ipadress of turtlebot
setenv('ROS_MASTER_URI', ipaddress);
rosinit(ipaddress,'NodeHost','192.168.1.133')                  %initate ros using turtlebot IP

%still needs to be subscribed and able to access laser and odom; 
laser = rossubscriber('/scan');      %initialize a subscriber node to kinect laser scan data
%odom = rossubscriber('/robot_pose_ekf/odom_combined');  %initialize a subscriber node to odomotry data
odom = rossubscriber('/odom');

% EKF Parameter Values
C = 0.2;    % Process Noise Constant
Rc = [1000,100];   % Measurement Noise Constants

landmark_list=[]; %this is an input to the function and can be either empty or full of stuff
ekf_init = 0;
odomTS = 0;
laserTS = 0;
while(1)
    % Receive ROS Topics
    %======================================================================
    laserData  = receive(laser); %recieve a laser scan 
    odomData = receive(odom);
    % End receive ROS topics
    %----------------------------------------------------------------------
    
    % Calculate Odometry
    %======================================================================
    % odometry data calculated here is the dead-reckoned position from
    % odometry readings in ROS.
    
    p = odomData.Pose.Pose;
    x_o = p.Position.X;
    y_o = p.Position.Y;
    
    quat = p.Orientation;
    angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
    theta = wrapTo360(rad2deg(angles(1)));
    
    rot_deg = 0;  % Turtlebot's heading may be in y direction
    % Rotate frame so heading is in x.
    pr = [cosd(rot_deg) -sind(rot_deg); sind(rot_deg) cosd(rot_deg)]*[x_o,y_o]';
    
    % store current position in terms of x, y, theta (degrees) as ROS sees
    odomPose = [pr(1),pr(2),theta];
    % End calculate odometery
    %----------------------------------------------------------------------
    
    % Estimate Robot's pose
    %======================================================================

    if(~exist('x'))
        oldOdomPose = odomPose;
        oldOdomData = odomData;
        % State Vector
        x = [0,0,0];
        % Covariance Matrix
        P = eye(length(x)).*0.1; 
        P(1,1) = 0.1; P(2,2) = 0.1; P(3,3) = 0.1;
        u = [0, 0];
    else
        % Get control vector (change in linear displacement and rotation)to
        % estimate current pose of the robot
        delta_D = sqrt((odomPose(1) - oldOdomPose(1))^2 + (odomPose(2) - oldOdomPose(2))^2);
        delta_Theta = rad2deg(angdiff(deg2rad(oldOdomPose(3)),deg2rad(odomPose(3))));
        u = [delta_D, delta_Theta];
        
        % Get noise covariance matrix for control signal
        W = [u(1)*cosd(x(3)) u(1)*sind(x(3)) u(2)]';
        Q = zeros(size(P));
        Q(1:3,1:3) = W*C*W';
        
        % Update position estimate
        [x,P] = EKF_SLAM_Prediction(x,P,u,Q);
        x(3) = wrapTo360(x(3));
        % Record current odometry pose for next iteration
        oldOdomPose = odomPose;
        oldOdomData = odomData;
    end
    
    % Search for landmarks
    [observed_LL,landmark_list]=getLandmark(landmark_list,laserData,x(1:3));

    % Apply measurement update in EKF if landmarks are observed
    if(~isempty(observed_LL))
        [numOfLandmarks ~] = size(observed_LL);
        for ii = 1:numOfLandmarks
            % Measurement vector
            z = [observed_LL(ii,1), observed_LL(ii,2)];
            % Measurement noise covariance matrix
            R = zeros(2,2); R(1,1) = observed_LL(ii,1)*Rc(1); R(2,2) = observed_LL(ii,2)*Rc(2);
            % Landmark index
            idx = observed_LL(ii,3);
            
            % if landmark is new, append to x and P
            idx2 = find(landmark_list(:,4)==idx);
            [x,P] = append(x,P,u,landmark_list(idx2,:),R);
        
            % Apply EKF measurement update
            [x,P] = EKF_SLAM_Measurement(x,P,z,R,idx);
            landmark_list = updateLandmarkList(x,landmark_list);
        end
    end
    % End estimate robot's pose
    %----------------------------------------------------------------------
        
    % Plot Junk
    %=======================================================================
    % set(gcf,'Visible','on');
    % set(0,'DefaultFigureVisible','on');
    clf; hold on;
    % Plot robot
    drawRobot(x(1),x(2),x(3),0.25);
    % Plot landmarks
    for ii = 1:((length(x)-3)/2)
        scatter(x((ii-1)*2 + 4),x((ii-1)*2 + 5),'blue','x');
    end
    % Plot "unofficial" landmarks
    idx = find(landmark_list(:,4) == 0);
    scatter(landmark_list(idx,1),landmark_list(idx,2),[],[.5 .5 .5],'x');
    % Plot observed landmarks
    if(~isempty(observed_LL))
        % Plot observed landmark locations
        scatter(landmark_list(idx2,1),landmark_list(idx2,2),'o','b');
        % Plot observed landmark distances and orientations
        lineptsx = x(1) + observed_LL(:,1).*cosd(observed_LL(:,2) + x(3));
        lineptsy = x(2) + observed_LL(:,1).*sind(observed_LL(:,2) + x(3));
        for jj = 1:length(lineptsx)
            plot([x(1) lineptsx(jj)],[x(2) lineptsy(jj)],'red');
        end
    end
    
    %Plot scan data
    %scatter(landmark_list(:,1),landmark_list(:,2)); 
    cartes_data = readCartesian(laserData); %read cartesian co-ordinates
    rot = [cosd(x(3)) -sind(x(3)) x(1); sind(x(3)) cosd(x(3)) x(2); 0 0 1];
    tmp = rot*[cartes_data,ones(length(cartes_data),1)]';
    scatter(tmp(1,:),tmp(2,:),'magenta','.');
    axis([-5 5 -5 5]);
    
    % End Plot Junk
    %----------------------------------------------------------------------    
    
    hold off
end


