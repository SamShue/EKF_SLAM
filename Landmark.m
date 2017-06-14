classdef Landmark
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        landmarkObj;
        %correspondence;
    end
    
    methods
        function h = Landmark(method)
            switch(method)
                case strcmp(method,'RANSAC')
                    h.landmarkObj = RANSAC();
                otherwise
                    warning('Improper landmark recognition method.');
                    h.landmarkObj = RANSAC();
                    %h.correspondence = Correspondence(100000000000000000,100000000000000000);
            end
        end
        
        
        function [observed_LL] = getLandmark(h, laserdata, x)
            [observed_LL] = h.landmarkObj.getLandmark(laserdata, x);
        end
        
    end
    
end

