classdef SLAM
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        slamObj;
        algorithmName;
    end
    
    methods
        
        % Constructor - initialize state variables and covariances
        function h = SLAM(inputString)
            h.algorithmName = inputString;
            switch(h.algorithmName)
                case 'EKF_WITH_CORRESPONDENCE'
                    h.slamObj = EKF_WITH_KNOWN_CORRESPONDENCE();
                case 'EKF_WITHOUT_CORRESPONDENCE'
                    h.slamObj = EKF_WITHOUT_KNOWN_CORRESPONDENCE();
                otherwise
                    
            end
        end
        
        function predict(h,u)
            switch(h.algorithmName)
                case 'EKF_WITH_CORRESPONDENCE'
                    h.slamObj.EKF_SLAM_Prediction(u);
                case 'EKF_WITHOUT_CORRESPONDENCE' 
                    h.slamObj.EKF_SLAM_Prediction(u);
                otherwise
            end
        end
        
        function correspondence(h,laserdata,u)
            switch(h.algorithmName)
                case 'EKF_WITH_CORRESPONDENCE'
                    h.slamObj.measureUnknownCorrespondence(laserdata,u);
                case 'EKF_WITHOUT_CORRESPONDENCE'
                    h.slamObj.measureUnknownCorrespondence(laserdata,u);
            end   
        end
    end
end
