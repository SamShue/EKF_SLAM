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

map=robotics.BinaryOccupancyGrid(25,25,5);




global observed
laserTotal=[];
while(1)
    % Receive ROS Topics
    %======================================================================
    laserData  = receive(laser); %recieve a laser scan
    laserTotal= laserData;
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
    if(~exist('ekf_slam_without_correspondence','var'))
        oldOdomPose = odomPose;
        ekf_slam_without_correspondence = SLAM('EKF_WITHOUT_CORRESPONDENCE');
        u = [0, 0];
    else
        % Get control vector (change in linear displacement and rotation)to
        % estimate current pose of the robot
        delta_D = sqrt((odomPose(1) - oldOdomPose(1))^2 + (odomPose(2) - oldOdomPose(2))^2);
        delta_Theta = rad2deg(angdiff(deg2rad(oldOdomPose(3)),deg2rad(odomPose(3))));
        u = [delta_D, delta_Theta];
        
        % Update position estimate
        ekf_slam_without_correspondence.predict(u);
        
        % Record current odometry pose for next iteration
        oldOdomPose = odomPose;
    end
    
    ekf_slam_without_correspondence.correspondence(laserData, u);
    
    
    cartes_data = readCartesian(laserData); %read cartesian co-ordinates

    rot = [cosd(ekf_slam_without_correspondence.method.x(3)) -sind(ekf_slam_without_correspondence.method.x(3)) ekf_slam_without_correspondence.method.x(1)+12.5; sind(ekf_slam_without_correspondence.method.x(3)) cosd(ekf_slam_without_correspondence.method.x(3)) ekf_slam_without_correspondence.method.x(2)+12.5; 0 0 1];
    world_frame_laser_scan = rot*[cartes_data,ones(length(cartes_data),1)]';
    
  %  world_frame_laser_scan(1,:) = world_frame_laser_scan(1,:)+50;
   % world_frame_laser_scan(2,:) = world_frame_laser_scan(2,:)+50;

    setOccupancy(map, world_frame_laser_scan(1:2,:)',1);
    show(map);
    
%     % End estimate robot's pose
%     %----------------------------------------------------------------------
%     
%     % Plot Junk
%     %=======================================================================
%     % set(gcf,'Visible','on');
%     % set(0,'DefaultFigureVisible','on');
%     clf; hold on;
%     
%     % Plot robot
%     drawRobot(ekf_slam_without_correspondence.method.x(1),ekf_slam_without_correspondence.method.x(2),ekf_slam_without_correspondence.method.x(3),0.25);
%     
%     % Plot landmarks
%     for ii = 1:((length(ekf_slam_without_correspondence.method.x)-3)/2)
%         scatter(ekf_slam_without_correspondence.method.x((ii-1)*2 + 4),ekf_slam_without_correspondence.method.x((ii-1)*2 + 5),'blue','x');
%     end
%     
%     % Plot "unofficial"/pre-filtered landmarks
%     temp=[ekf_slam_without_correspondence.method.landmark_list.landmark(:).index];
%     idx = find(temp(:) == 0);
%     temp=[];
%     for mm=1:size(idx,1)
%         temp=[temp;ekf_slam_without_correspondence.method.landmark_list.landmark(idx(mm)).loc(1),ekf_slam_without_correspondence.method.landmark_list.landmark(idx(mm)).loc(1)];
%     end
%     if(~isempty(idx))
%         scatter(temp(:,1),temp(:,2),[],[.5 .5 .5],'x');
%     end
%     
%     % Plot range and orientation of observed landmarks
%     if(~isempty(observed))
%         % Plot observed landmark locations
%         for ii = 1:size(observed,1)
%             temp = [ekf_slam_without_correspondence.method.landmark_list.landmark(:).index];
%             idx2 = find(temp(:)==observed(ii,3));  % Landmark of correspondence idx
%             scatter(ekf_slam_without_correspondence.method.landmark_list.landmark(idx2).loc(1),ekf_slam_without_correspondence.method.landmark_list.landmark(idx2).loc(2),'o','b');
%             % Plot observed landmark distances and orientations
%             lineptsx = ekf_slam_without_correspondence.method.x(1) + observed(:,1).*cosd(observed(:,2) + ekf_slam_without_correspondence.method.x(3));
%             lineptsy = ekf_slam_without_correspondence.method.x(2) + observed(:,1).*sind(observed(:,2) + ekf_slam_without_correspondence.method.x(3));
%             for jj = 1:length(lineptsx)
%                 plot([ekf_slam_without_correspondence.method.x(1) lineptsx(jj)],[ekf_slam_without_correspondence.method.x(2) lineptsy(jj)],'red');
%             end
%         end
%     end
%     observed=[];
%     %Plot scan data
%     cartes_data = readCartesian(laserData); %read cartesian co-ordinates
%     rot = [cosd(ekf_slam_without_correspondence.method.x(3)) -sind(ekf_slam_without_correspondence.method.x(3)) ekf_slam_without_correspondence.method.x(1); sind(ekf_slam_without_correspondence.method.x(3)) cosd(ekf_slam_without_correspondence.method.x(3)) ekf_slam_without_correspondence.method.x(2); 0 0 1];
%     tmp = rot*[cartes_data,ones(length(cartes_data),1)]';
%     scatter(tmp(1,:),tmp(2,:),'magenta','.');
%     axis([-3.5 3.5 -3.5 3.5]);
%     
%     
%     % Plot robot and landmark covariances
%     robotSigma=[ekf_slam_without_correspondence.method.P(1,1),ekf_slam_without_correspondence.method.P(1,2);ekf_slam_without_correspondence.method.P(2,1),ekf_slam_without_correspondence.method.P(2,2)];
%     robotMu=[ekf_slam_without_correspondence.method.x(1);ekf_slam_without_correspondence.method.x(2)];
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
%     rCoords=R*[robot_x;robot_y]*.25;
%     xr=rCoords(1,:);
%     yr=rCoords(2,:);
%     xr=xr+robotMu(1);
%     yr=yr+robotMu(2);
%     plot(xr,yr);
%     
%     for ii=4:2:size(ekf_slam_without_correspondence.method.x,2)
%         landmarkSigma=[ekf_slam_without_correspondence.method.P(ii,ii),ekf_slam_without_correspondence.method.P(ii,ii+1);ekf_slam_without_correspondence.method.P(ii+1,ii),ekf_slam_without_correspondence.method.P(ii+1,ii+1)];
%         robotMu=[ekf_slam_without_correspondence.method.x(ii);ekf_slam_without_correspondence.method.x(ii+1)];
%         
%         [eigvec,eigval]=eig(landmarkSigma);
%         chi_square=2.2788;   %2.2788
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
%         rCoords=R*[landmark_x;landmark_y]*.50;
%         xr=rCoords(1,:);
%         yr=rCoords(2,:);
%         xr=xr+robotMu(1);
%         yr=yr+robotMu(2);
%         plot(xr,yr);
%     end
%     
%     hold off
    % End Plot Junk
    %----------------------------------------------------------------------
end


