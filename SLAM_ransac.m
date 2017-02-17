clc;                                %clear command window
clear all;
close all;                          %close all figures
rosshutdown                         %close current ros incase it is already initalized 
setenv('ROS_HOSTNAME', 'rahul-ThinkPad-S3-Yoga-14');
setenv('ROS_IP', '192.168.1.104');
ipaddress = '192.168.1.13';         %define ipadress of turtlebot
rosinit(ipaddress)                  %initate ros using turtlebot IP

%final_landmark_list=[]; 
landmark_list=[]; %this is an input to the function and can be either empty or full of stuff
confirmed_landmark_list=zeros(3,1); %this is an input to the function, the first time it is input it MUST have 3 empty spaces already ready

%landFig=figure(2);

%still needs to be subscribed and able to access laser and odom; 
laser = rossubscriber('/scan');      %initialize a subscriber node to kinect laser scan data
%odom = rossubscriber('/robot_pose_ekf/odom_combined');  %initialize a subscriber node to odomotry data
odom = rossubscriber('/odom');
ekf_init = 0;
while(1)
    % Get sensor information
    data  = receive(laser); %recieve a laser scan 
    odomdata = receive(odom);
    
    % Get odometry orientation
    p = odomdata.Pose.Pose;
    x_o = p.Position.X;
    y_o = p.Position.Y;
   % z = p.Position.Z;
    
    quat = p.Orientation;
    angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
    theta = rad2deg(angles(1));
    
    
    odom_pose=[x_o,y_o,theta];
    odomTime = odomdata.Header.Stamp.Sec;
    
    % Get pose
    if(ekf_init)
        pose = x(1:3) + [odom_pose - oldOdomPose];
    else
        % EKF not initialized
        if(~exist('pose'))
        	pose = [0,0,0];
        else
            pose = pose + [odom_pose - oldOdomPose];
        end
    end
    [observed_LL,landmark_list]=getLandmark(landmark_list,data,pose);
    landmark_list
%     disp('pfLL');
%     disp(observed_LL);
%     disp('landmark_list');
%     disp(landmark_list);
%     disp('confirmed');
%     disp(confirmed_landmark_list);
   % close all;                          %close all figures
   
   % Apply EKF to each observed landmark
   
   if(~isempty(observed_LL))
       [numOfLandmarks ~] = size(observed_LL);
       if ekf_init == 1
        [x,P] = append(x,P,u,observed_LL,R);
       end
       for ii = 1:numOfLandmarks
           % Apply EKF
           if(ii == 1)
               % get control vector
               delta_t = odomTime - oldOdomTime; % Change in time (seconds) (not used atm)
               delta_D = sqrt((odom_pose(1) - oldOdomPose(1))^2 + (odom_pose(2) - oldOdomPose(2))^2);
               delta_Theta = odom_pose(3) - oldOdomPose(3);
               u = [delta_D, delta_Theta];
               if(ekf_init == 0)
                   % initialize EKF variables
                   x = pose;
                   % Covariance Matrix
                    P = eye(length(x)).*0.1; 
                    P(1,1) = 0.1; P(2,2) = 0.1; P(3,3) = 0.1;
                    R = zeros(2,2); R(1,1) = observed_LL(ii,1)*10; R(2,2) = observed_LL(ii,2)*10;
                    [x,P] = append(x,P,u,observed_LL,R);
                     
                    ekf_init = 1;
               end
           else
               % get control vector
               u = [0, 0];
               R = zeros(2,2); R(1,1) = observed_LL(ii,1)*10; R(2,2) = observed_LL(ii,2)*10;
           end
           % Filter goes here
           
           % Process Noise : Always calculate after appending as
           % it changes the size of P
            C = 0.1;
            W = [u(1)*cosd(x(3)) u(1)*sind(x(3)) u(2)]';
            Q = zeros(size(P));
            Q(1:3,1:3) = W*C*W';
           [x,P] = EKF(x,P,observed_LL(ii,1:2),u,observed_LL(ii,3),R,Q);
           if(isnan(x(4)))
               x
           end
           landmark_list = updateLandmarkList(x, landmark_list);
       end
   end
   
    
    oldOdomPose = odom_pose;
    oldOdomTime = odomdata.Header.Stamp.Sec;
    
    % Print Junk
    set(gcf,'Visible','on');
    set(0,'DefaultFigureVisible','on');
    clf; hold on;
    if(ekf_init)
        scatter(x(1),x(2),'red','o');
        for ii = 1:((length(x)-3)/2)
            scatter(x((ii-1)*2 + 4),x((ii-1)*2 + 5),'blue','o');
        end
    else
        scatter(pose(1),pose(2),'red','o');
    end
    % Plot scan data
    cartes_data = readCartesian(data); %read cartesian co-ordinates
    rot = [cosd(pose(3)) sind(pose(3)) pose(1); -sind(pose(3)) cosd(pose(3)) pose(2); 0 0 1];
    tmp = rot*[cartes_data,ones(length(cartes_data),1)]';
    scatter(tmp(1,:),tmp(2,:),'magenta','.');
end