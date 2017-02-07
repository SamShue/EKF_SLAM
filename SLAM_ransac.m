clc;                                %clear command window
close all;                          %close all figures
rosshutdown                         %close current ros incase it is already initalized 
ipaddress = '192.168.1.11';         %define ipadress of turtlebot
rosinit(ipaddress)                  %initate ros using turtlebot IP

%final_landmark_list=[]; 
landmark_list=[]; %this is an input to the function and can be either empty or full of stuff
confirmed_landmark_list=zeros(3,1); %this is an input to the function, the first time it is input it MUST have 3 empty spaces already ready

%landFig=figure(2);

%still needs to be subscribed and able to access laser and odom; 
laser = rossubscriber('/scan');      %initialize a subscriber node to kinect laser scan data
odom = rossubscriber('/robot_pose_ekf/odom_combined');  %initialize a subscriber node to odomotry data

ekf_init = 0;
while(1)
    % Get sensor information
    data  = receive(laser); %recieve a laser scan 
    odomdata = receive(odom);
    
    % Get odometry orientation
    p = odomdata.Pose.Pose;
    x = p.Position.X;
    y = p.Position.Y;
   % z = p.Position.Z;
    
    quat = p.Orientation;
    angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
    theta = rad2deg(angles(1));
    
    odom_pose=[x,y,theta];
    odomTime = odomdata.Header.Stamp.Sec;
    
    % Get pose
    if(ekf_init)
        pose = x(1:3) + [odom_pose - oldOdomPose];
    else
        % EKF not initialized
        pose = odom_pose;
    end
    
    [observed_LL,landmark_list]=getLandmark(landmark_list,data,pose);
    disp('pfLL');
    disp(observed_LL);
    disp('landmark_list');
    disp(landmark_list);
    disp('confirmed');
    disp(confirmed_landmark_list);
   % close all;                          %close all figures
   
   % Apply EKF to each observed landmark
   
   if(~isempty(observed_LL))
       [numOfLandmarks ~] = size(observed_LL);
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
                   x;
                   P;
                   Q;
                   R;
                   ekf_init = 1;
               end
           else
               % get control vector
               u = [0, 0];
           end
           % Filter goes here
           % Update landmark list after filter
           landmark_list = updateLandmarkList(x, landmark_list);
       end
   end
   
    oldOdomPose = odom_pose;
    oldOdomTime = odomdata.Header.Stamp.Sec;
end