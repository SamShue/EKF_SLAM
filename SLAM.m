classdef SLAM
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        method;
        method_name;
    end
    
    methods
        
        % Constructor - initialize state variables and covariances
        function h = SLAM(input_method)
            h.method_name=input_method;
            switch(h.method_name)
                case 'EKF_WITH_CORRESPONDENCE'
                    h.method= EKF_WITH_KNOWN_CORRESPONDENCE();
                case 'EKF_WITHOUT_CORRESPONDENCE'
                    h.method= EKF_WITHOUT_KNOWN_CORRESPONDENCE();
                otherwise
                    
            end
        end
        
        function predict(h,u)
            switch(h.method_name)
                case 'EKF_WITH_CORRESPONDENCE'
                    h.method.EKF_SLAM_Prediction(u);
                case 'EKF_WITHOUT_CORRESPONDENCE' 
                    h.method.EKF_SLAM_Prediction(u);
                otherwise
            end
        end
        
        function correspondence(h,laserdata,u)
            switch(h.method_name)
                case 'EKF_WITH_CORRESPONDENCE'
                    h.method.measureUnknownCorrespondence(laserdata,u);
                case 'EKF_WITHOUT_CORRESPONDENCE'
                    h.method.measureUnknownCorrespondence(laserdata,u);
            end   
        end
    end
end
