clc;                                %clear command window
clear all;
close all;                          %close all figures
rosshutdown                         %close current ros incase it is already initalized

% Robot network variables
ipaddress = 'http://192.168.1.16:11311';         %define ipadress of turtlebot
%setenv('ROS_MASTER_URI', ipaddress);
%rosinit(ipaddress,'NodeHost','192.168.1.133')                  %initate ros using turtlebot IP
rosinit('192.168.1.13');

laser = rossubscriber('/scan');      %initialize a subscriber node to kinect laser scan data
odom = rossubscriber('/odom');



landmark_list = RANSAC();


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
    
    % store current position in terms of x, y, theta (degrees) as ROS sees
    odomPose = [x_o,y_o,theta];
    % End calculate odometery
    %----------------------------------------------------------------------
    
    % Estimate Robot's pose
    %======================================================================
    % Initialize variables if first iteration
    if(~exist('slam'))
        oldOdomPose = odomPose;
        slam = EKF_SLAM();
        u = [0, 0];
    else
        % Get control vector (change in linear displacement and rotation)to
        % estimate current pose of the robot
        delta_D = sqrt((odomPose(1) - oldOdomPose(1))^2 + (odomPose(2) - oldOdomPose(2))^2);
        delta_Theta = rad2deg(angdiff(deg2rad(oldOdomPose(3)),deg2rad(odomPose(3))));
        u = [delta_D, delta_Theta];
        
        % Update position estimate
        slam.EKF_SLAM_Prediction(u);
        
        % Record current odometry pose for next iteration
        oldOdomPose = odomPose;
    end
    
    slam.measureFromLandmarks(laserData);
    % End estimate robot's pose
    %----------------------------------------------------------------------
    
    % Plot Junk
    %=======================================================================
    % set(gcf,'Visible','on');
    % set(0,'DefaultFigureVisible','on');
    clf; hold on;
    
    % Plot robot
    drawRobot(slam.x(1),slam.x(2),slam.x(3),0.25);
    
    % Plot landmarks
    for ii = 1:((length(slam.x)-3)/2)
        scatter(slam.x((ii-1)*2 + 4),slam.x((ii-1)*2 + 5),'blue','x');
    end
    
%     % Plot "unofficial"/pre-filtered landmarks
%     temp=[landmark_list.landmark(:).index];
%     idx = find(temp(:) == 0);
%     temp=[];
%     for mm=1:size(idx,1)
%         temp=[temp;landmark_list.landmark(idx(mm)).loc(1),landmark_list.landmark(idx(mm)).loc(1)];
%     end
%     if(~isempty(idx))
%         scatter(temp(:,1),temp(:,2),[],[.5 .5 .5],'x');
%     end
%     
%     % Plot range and orientation of observed landmarks
%     if(~isempty(observed_LL))
%         % Plot observed landmark locations
%         for ii = 1:size(observed_LL,1)
%             temp = [landmark_list.landmark(:).index];
%             idx2 = find(temp(:)==observed_LL(ii,3));  % Landmark of correspondence idx
%             scatter(landmark_list.landmark(idx2).loc(1),landmark_list.landmark(idx2).loc(2),'o','b');
%             % Plot observed landmark distances and orientations
%             lineptsx = x(1) + observed_LL(:,1).*cosd(observed_LL(:,2) + x(3));
%             lineptsy = x(2) + observed_LL(:,1).*sind(observed_LL(:,2) + x(3));
%             for jj = 1:length(lineptsx)
%                 plot([x(1) lineptsx(jj)],[x(2) lineptsy(jj)],'red');
%             end
%         end
%     end
%     
%     %Plot scan data
%     cartes_data = readCartesian(laserData); %read cartesian co-ordinates
%     rot = [cosd(x(3)) -sind(x(3)) x(1); sind(x(3)) cosd(x(3)) x(2); 0 0 1];
%     tmp = rot*[cartes_data,ones(length(cartes_data),1)]';
%     scatter(tmp(1,:),tmp(2,:),'magenta','.');
%     axis([-5 5 -5 5]);
%     
%     
%     % Plot robot and landmark covariances
%     robotSigma=[P(1,1),P(1,2);P(2,1),P(2,2)];
%     robotMu=[x(1);x(2)];
%     [eigvec,eigval]=eig(robotSigma);
%     chi_square=2.2788;
%     major=2*sqrt(chi_square*eigval(1,1));
%     minor=2*sqrt(chi_square*eigval(2,2));
%     t=-pi:0.01:pi;
%     if(eigval(1,1)>eigval(2,2))
%         arc=atan(eigvec(2,1)/eigvec(1,1));
%         robot_x=major*cos(t);
%         robot_y=minor*sin(t);
%     else
%         arc=atan(eigvec(2,2)/eigvec(1,2));
%         robot_x=minor*cos(t);
%         robot_y=major*sin(t);
%     end
%     R=[cos(arc) -sin(arc); sin(arc) cos(arc)];
%     rCoords=R*[robot_x;robot_y];
%     xr=rCoords(1,:);
%     yr=rCoords(2,:);
%     xr=xr+robotMu(1);
%     yr=yr+robotMu(2);
%     plot(xr,yr);
%     
%     for ii=4:2:size(x,2)
%         landmarkSigma=[P(ii,ii),P(ii,ii+1);P(ii+1,ii),P(ii+1,ii+1)];
%         robotMu=[x(ii);x(ii+1)];
%         
%         [eigvec,eigval]=eig(landmarkSigma);
%         chi_square=2.2788;
%         major=2*sqrt(chi_square*eigval(1,1));
%         minor=2*sqrt(chi_square*eigval(2,2));
%         t=-pi:0.01:pi;
%         if(eigval(1,1)>eigval(2,2))
%             arc=atan(eigvec(2,1)/eigvec(1,1));
%             landmark_x=major*cos(t);
%             landmark_y=minor*sin(t);
%         else
%             arc=atan(eigvec(2,2)/eigvec(1,2));
%             landmark_x=minor*cos(t);
%             landmark_y=major*sin(t);
%         end
%         R=[cos(arc) -sin(arc); sin(arc) cos(arc)];
%         rCoords=R*[landmark_x;landmark_y];
%         xr=rCoords(1,:);
%         yr=rCoords(2,:);
%         xr=xr+robotMu(1);
%         yr=yr+robotMu(2);
%         plot(xr,yr);
%     end
%     
    hold off
    % End Plot Junk
    %----------------------------------------------------------------------
end


