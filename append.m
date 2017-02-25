function [x,P] = append(x,P,u,landmark_list,R,pose)
    % Does landmark need to be appended?
    numOfLandmarks = (length(x) - 3) / 2;
    
    if(numOfLandmarks < landmark_list(4))
        n = length(P);  
        
        % Append landmark to x
        x = [x , landmark_list(ii,1), landmark_list(ii,2)];
        
        % Get SLAM-Specific Jacobians (as defined by SLAM for Dummies!)
        jxr = [1 0 -u(1)*sind(x(3));...
            0 1 u(1)*cosd(x(3))];
        
        jz = [cosd(u(2)) -u(1)*sind(u(2));...
              sind(u(2)) u(1)*cosd(u(2))];
        
        % Append landmark to P (again as defined by SLAM for Dummies)
        P(n+1:n+2,n+1:n+2) = jxr*P(1:3,1:3)*jxr' + jz*R*jz';    %C
        P(1:3,n+1:n+2) = P(1:3,1:3)*jxr';                       %I
        P(n+1:n+2,1:3) = P(1:3,n+1:n+2)';                       %H
        for idx = 1: numOfLandmarks
            P((n+1):(n+2),((idx-1)*2+4):((idx-1)*2+5)) = jxr*P(((idx-1)*2+4):((idx-1)*2+5),1:3)';   %F
            P(((idx-1)*2+4):((idx-1)*2+5),(n+1):(n+2)) = P((n+1):(n+2),((idx-1)*2+4):((idx-1)*2+5))';%G
        end
    end
end
        
        
        
        
        
    
    
    
    % Reshape landmark_list to only contain new landmarks in ascending
    % order
    [G sortedidx] = sort(landmark_list(:,4));
    landmark_list = landmark_list(sortedidx,:);
    landmark_list(find(landmark_list(:,4)==0),:)=[];
    newidx = find(landmark_list(:,4)>numOfLandmarks);
    landmark_list = landmark_list(newidx,:);
    
    if(~isempty(landmark_list))
        for ii = 1:size(landmark_list,1)
            n = length(P);    
            x = [x , landmark_list(ii,1), landmark_list(ii,2)];
            
            P(n+1:n+2,n+1:n+2) = jxr*P(1:3,1:3)*jxr' + jz*R*jz';  %C
            P(1:3,n+1:n+2) = P(1:3,1:3)*jxr';                %I
            P(n+1:n+2,1:3) = P(1:3,n+1:n+2)';            %H
            for idx = 1: numOfLandmarks
                P((n+1):(n+2),((idx-1)*2+4):((idx-1)*2+5)) = jxr*P(((idx-1)*2+4):((idx-1)*2+5),1:3)';   %F
                P(((idx-1)*2+4):((idx-1)*2+5),(n+1):(n+2)) = P((n+1):(n+2),((idx-1)*2+4):((idx-1)*2+5))';%G
            end
         end
    end  
end
