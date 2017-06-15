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
                case 'EKF_WITH_CORRESPONDENCE'
                    h.slam.prediction(u);
                case 'EKF_WITHOUT_CORRESPONDENCE' 
                    h.slam.prediction(u);
                otherwise
            end
        end
        
        function correspondence(h,laserdata,u)
            switch(h.algorithmName)
                case 'EKF_WITH_CORRESPONDENCE'
                    h.slam.measurement(laserdata,u);
                case 'EKF_WITHOUT_CORRESPONDENCE'
                    h.slam.measurement(laserdata,u);
            end   
        end
    end
end
