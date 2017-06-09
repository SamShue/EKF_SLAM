classdef Landmark
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        s;
        
        landmarkObj;
        correspondence;
    end
    
    methods
        function h = Landmark(method)
            switch(method)
                case strcmp(method,'RANSAC')
                    h.landmarkObj = RANSAC();
                otherwise
                    warning('Improper landmark recognition method.');
                    h.landmarkObj = RANSAC();
                    h.correspondence = Correspondence(100000000000000000,100000000000000000);
            end
        end
        
        
        function [observedLandmarks] = getLandmark(h, sensorData,x,P,z,R)
            if(nargin < 3)      % Known Correspondence
                % observedLandmarks = [range,orientation,landmark index,new landmark]
                [observedLandmarks] = h.landmarkObj.getLandmark(sensorData, x);
                if(~isempty(observedLandmarks))
                    % Sort in descending order, as it can conflict with
                    % prefiltered lanmark structure.
                    observedLandmarks = sortrows(observedLandmarks,3);
                    % Find landmarks which need to be appended
                    numOfLandmarks = (length(h.x)-3)/2;
                    observedLandmarks = [observedLandmarks;observedLandmarks(:,3)>numOfLandmarks];
                end
            elseif(nargin == 5) % Unknown Correspondence
                % observed_LL = [range,orienation,signature]
                [observedLandmarks_sig] = h.landmarkObj.getLandmark(sensorData, x);
                if(~isempty(observedLandmarks_sig))
                    % find landmark index through landmark correspondence
                    for ii = 1:size(observedLandmarks_sig,1)
                        z = observedLandmarks_sig(:,ii);
                        [newLL,index] = h.correspondence.estimateCorrespondence(x,P,z,R,h.s);
                        observedLandmarks(:,ii) = [observedLandmarks(1:2),index,newLL];
                        if(newLL)
                            h.s = [h.s,observedLandmarks_sig(ii,3)];
                        end
                    end
                end
            end
        end
        
    end
    
end

