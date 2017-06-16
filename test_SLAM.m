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
    if(~exist('s','var'))
        oldOdomPose = odomPose;
        s = SLAM('EKF_SLAM_UC');
        u = [0, 0];
    else
        % Get control vector (change in linear displacement and rotation)to
        % estimate current pose of the robot
        delta_D = sqrt((odomPose(1) - oldOdomPose(1))^2 + (odomPose(2) - oldOdomPose(2))^2);
        delta_Theta = rad2deg(angdiff(deg2rad(oldOdomPose(3)),deg2rad(odomPose(3))));
        u = [delta_D, delta_Theta];
        
        % Update position estimate
        s.predict(u);
        
        % Record current odometry pose for next iteration
        oldOdomPose = odomPose;
    end
    
    s.measure(laserData, u);
    
    cartes_data = readCartesian(laserData); %read cartesian co-ordinates

    rot = [cosd(s.slam.x(3)) -sind(s.slam.x(3)) s.slam.x(1)+12.5; sind(s.slam.x(3)) cosd(s.slam.x(3)) s.slam.x(2)+12.5; 0 0 1];
    world_frame_laser_scan = rot*[cartes_data,ones(length(cartes_data),1)]';
    
    %setOccupancy(map, world_frame_laser_scan(1:2,:)',1);
    %show(map);
   
    
    
    
    %Plot scan data
     
    cartes_data = readCartesian(laserData); %read cartesian co-ordinates
    rot = [cosd(s.slam.x(3)) -sind(s.slam.x(3)) s.slam.x(1); sind(s.slam.x(3)) cosd(s.slam.x(3)) s.slam.x(2); 0 0 1];
    tmp = rot*[cartes_data,ones(length(cartes_data),1)]';
    scatter(tmp(1,:),tmp(2,:),'magenta','.');
    axis([-3.5 3.5 -3.5 3.5]);
            
    s.slam.plot(); 
    
    
end


