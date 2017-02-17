function [x,P] = append(x,P,u,observed_LL,R)

    jxr = [1 0 -u(1)*sind(x(3));...
            0 1 u(1)*cosd(x(3))];
        
    jz = [cosd(u(2)) -u(1)*sind(u(2));...
            sind(u(2)) u(1)*cosd(u(2))];
    
    numOfLandmarks = (length(x) - 3) / 2;
    
    for ii = 1:length(observed_LL(:,1))
    n = length(P);    
        %check if landmark index is out of bounds
        %and append it if it is
         if observed_LL(ii,3) > numOfLandmarks
            x = [x , observed_LL(ii,1:2)];
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
