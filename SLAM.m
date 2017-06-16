classdef SLAM
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        slam;
        algorithmName;
    end
    
    methods
        
        % Constructor - initialize state variables and covariances
        function h = SLAM(inputString)
            h.algorithmName = inputString;
            switch(h.algorithmName)
                case 'EKF_SLAM'
                    h.slam = EKF_SLAM();
                case 'EKF_SLAM_UC'
                    h.slam = EKF_SLAM_UC();
                otherwise
                    
            end
        end
        
        function predict(h,u)
            switch(h.algorithmName)
                case 'EKF_SLAM'
                    h.slam.predict(u);
                case 'EKF_SLAM_UC' 
                    h.slam.predict(u);
                otherwise
            end
        end
        
        function measure(h,laserdata,u)
            switch(h.algorithmName)
                case 'EKF_SLAM'
                    h.slam.measure(laserdata,u);
                case 'EKF_SLAM_UC'
                    h.slam.measure(laserdata,u);
            end   
        end
    end
end
