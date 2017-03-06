function [ observed_LL, output_landmark_list ] = updatedGetLandmark( input_landmark_list,laserdata,pose )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes her

%pseudo---

%1. Remove NULLS form laser data

%2. Convert laser data to cartesion coordinates

%3. Rotate cartesion coordinates from local frame to world frame 

%4. Take random point from world frame cartesion coordinates

%5. Attempt to find # of points that are within # of degrees of the random
%   sample, if not enough points can be found then take new point up to # times

%6. Create line of best fit using random point and points within # degrees 

%7. Search through rotated cartesion coordinates for any points within # of
%   the line of best fit.

%8. Create new line of best fit using all points found (including the
%   inital random point, then remove those points from the rotated
%   cartesion coordinates.

%9. Determine closest point on line of best fit that is orthoganal to the
%   origin. Add that point to a potential landmark list

%9.5 Repeat steps 4-9 until the number of points in the rotated cartesion
%    coordinates is less than # or until timeout occurs.

%10. Search through the input landmark list to see if any points on the
%    potential landmark list are within # of an input landmark. If so then
%    increment the count of the landmark on the input landmark list, else
%    add the potential landmark to the landmark list 

%11. During the previous step, if a landmark was reobserved (count was
%    incremented) then add it to the observed landmark list in the form of
%    [distance to origion, degrees from pose] 

%12. Check to see if the count of any un-indexed landmarks has exceeded #, 
%    if so then index that landmark and add it to the observed landmark
%    list, else... uh do nothing 

%13. Output the now updated input landmark list and the observed landmark
%    list


lineConsensus=200;    %number of points needed to be near a line of best 
                      %fit in order for it to be considered a potential wall 
                      
wallSearchTimeOut=5;  %maximum number of times a single laser scan will be 
                      %searched for walls 
                      
numberOfPointsWithinDegrees=10; %number of points used to create sample line

degrees=10;           %number of degrees a point must be within 
                      %for it to be considered for use when 
                      %creating the sample line
                      
lineOfBestFitDistance=.2; %distance a point can be away from the line of 
                          %best fit and still be considered part of the 
                          %line
                          
landmarkDistance=.2;  %distance a potential landmark can be away from a
                      %input landmark without being considered a new 
                      %landmark
                      
landmarkCountConsensus=10;  %number of times a landmark must be observed 
                            %before being considered an 'offical' landmark
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
%start!%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %1 remove null
    ind =  find(isnan(laserdata.Ranges)); 
    laserdata.Ranges(ind)=[]; 

    %2 convert to cartes
    localFrameCartesCords=readCartesian(laserdata); 

    %3 get points from robot local frame to world frame
    rot = [cosd(pose(3)) -sind(pose(3)) pose(1); sind(pose(3)) cosd(pose(3)) pose(2); 0 0 1];
    worldFrameCartesCords = rot*[localFrameCartesCords,ones(length(localFrameCartesCords),1)]';
    worldFrameCartesCords=worldFrameCartesCords';
    worldFrameCartesCords(:,3)=[];


    counter=0;
    potentialLineList=[];
    
    %9.5 keep searching worldFrameCartesCords for walls until not enough points are left
    %    for a wall to pass the lineConsensus (or the loop runs # times)
    while(length(worldFrameCartesCords)>lineConsensus && counter<wallSearchTimeOut)
        
        %4 & 5 Get random point and # of points within 10 degrees
        pointsWithinDegrees=findPoints(worldFrameCartesCords,numberOfPointsWithinDegrees,degrees);
        
        %the reamining steps only run if steps 4 & 5 were completed
        %succesfully
        if(~isempty(pointsWithinDegrees))
            
            %6 & 7 & 8 Using the random points try to find a wall
            [potentialLine,worldFrameCartesCords] = findPotentialLine(worldFrameCartesCords,pointsWithinDegrees,lineConsensus,lineOfBestFitDistance);
            potentialLineList=[potentialLineList;potentialLine];
        end
        counter=counter+1; 
    end
    
    %9 if any potential lines were found then find the closest point
    %  orthogonal to the origion of each potential line and set it as a 
    %  potential landmark
    if(~isempty(potentialLineList))
        potentialLandmarkList=getOrthogPoints(potentialLineList);
    
        
        %10 & 11 & 12 & 13 check to see if any of the potentialLandmarks are already on the
        %   input landmark list. if so then increment the count of that input
        %   landmark else add it to the list with an index of 0. 
        %   ALSO, if a landmark is already indexed and is 'reobserved' then add
        %   it to an observed_LL list in terms of distance to robot, angle to
        %   robot, and index.
        [observed_LL, output_landmark_list]=getOutputLandmarkListAndObservedLandmarkList(input_landmark_list,potentialLandmarkList,landmarkCountConsensus,landmarkDistance,pose);

    else
       output_landmark_list=input_landmark_list;
       observed_LL=[]; 
    end
    
    
end 

function pointsWithinDeg = findPoints(worldFrameCartesCords,numberOfPointsWithinDegrees,degrees)
    
    %take a random point from the avaiable points
    [samplePoint,~] = datasample(worldFrameCartesCords,1,'Replace',false);
    
    %get the degree max and min away from the sample point 
    randomSampleDegreeMax = atand(samplePoint(1,2)/samplePoint(1,1)) + degrees/2;
    randomSampleDegreeMin = atand(samplePoint(1,2)/samplePoint(1,1)) - degrees/2;
    
    %remove all points that are outside the # degree limit 
    ii=1;
    while ii<length(worldFrameCartesCords)
        deg = atand(worldFrameCartesCords(ii,2)/worldFrameCartesCords(ii,1));
        if( deg>randomSampleDegreeMax || deg <randomSampleDegreeMin)
            worldFrameCartesCords(ii,:)=[];
        else
            ii=ii+1;
        end
    end
    
    %take # of random samples from the points within # degrees
    listOfPoints=[];
    if(size(worldFrameCartesCords,1)>numberOfPointsWithinDegrees)
        [listOfPoints,~] = datasample(worldFrameCartesCords,numberOfPointsWithinDegrees,'Replace',false);
    end
      
    pointsWithinDeg=listOfPoints;

end

function [potentialLine,updatedWorldFrameCartesCords] = findPotentialLine(worldFrameCartesCords,pointsWithinDegrees,lineConsensus,lineOfBestFitDistance)

    sampleLine=polyfit(pointsWithinDegrees(:,1),pointsWithinDegrees(:,2),1);    
    v1=[1,polyval(sampleLine,1),0];
    v2=[2,polyval(sampleLine,2),0];
    idx = zeros(length(worldFrameCartesCords),1);

    for i= 1 : size(worldFrameCartesCords,1)
        p=[worldFrameCartesCords(i,:),0];
        a=v1-v2;
        b=p-v2;
        d =norm(cross(a,b))/norm(a);
        if d<lineOfBestFitDistance
            idx(i)=i;
        end
    end
   
    idx=idx(idx~=0);
    updatedWorldFrameCartesCords=worldFrameCartesCords;
    potentialLine=[]; 
    if length(idx)>lineConsensus

        updatedWorldFrameCartesCords=worldFrameCartesCords; 
        updatedWorldFrameCartesCords(idx,:)=[];
        
        pointsWithinDistance=worldFrameCartesCords(idx,:);
        potentialLine=polyfit(pointsWithinDistance(:,1),pointsWithinDistance(:,2),1);
        
        %plot([0,5],[polyval(potentialLine,0),polyval(potentialLine,5)]);hold on;
       
    end
       
end

function orthogPoints = getOrthogPoints(potentialLineList)
    orthogPoints=zeros(size(potentialLineList,1),2);
    for ii=1:size(potentialLineList,1)
        m= (polyval(potentialLineList(ii,:),0)-polyval(potentialLineList(ii,:),5))/(0-5);
        b=polyval(potentialLineList(ii,:),0)-m*0;
        
        syms x y 
        eqn1 = y-m*x == b;
        eqn2 = y+1/m*x == 0; 
        [A,B]=equationsToMatrix([eqn1,eqn2],[x,y]);
        X = linsolve(A,B);
        
        orthogPoints(ii,1)=X(1);
        orthogPoints(ii,2)=X(2);
    end
end

function [reobservedLandmarkList, outputLandmarkList]=getOutputLandmarkListAndObservedLandmarkList(input_landmark_list,potentialLandmarkList,landmarkCountConsensus,landmarkDistance,pose)
   
    reobservedLandmarkList=[];

    if(isempty(input_landmark_list)&&~isempty(potentialLandmarkList))
        input_landmark_list=[potentialLandmarkList(1,:),1,0];  %consider changing this once done testing
        
    elseif(~isempty(potentialLandmarkList))
    
        
        %loop through all the potential landmarks
        for ii=1:size(potentialLandmarkList,1)
            flag=0;
            %loop through all the landmarks that already exsit
            for jj=1:size(input_landmark_list,1)    
                %check to see if a potential landmark might have already
                %been observed and marked as a landmark
                d=norm(potentialLandmarkList(ii,:)-[input_landmark_list(jj,1),input_landmark_list(jj,2)]);
                
                %if so then increment that landmarks count
                if(d<landmarkDistance)
                    
                    %increment count
                    input_landmark_list(jj,3)=input_landmark_list(jj,3)+1;
                    flag=1;
                    
                    %check if the count has increased to the consensus, if
                    %so then index it
                    if(input_landmark_list(jj,3)>landmarkCountConsensus && input_landmark_list(jj,4)==0)
                        input_landmark_list(jj,4)=max(input_landmark_list(:,4))+1;
                    end
                    
                    %if the landmark is indexed then replace the landmark
                    %list value with the measured value
                    if(input_landmark_list(jj,4)~=0)
                       input_landmark_list(jj,1)=potentialLandmarkList(ii,1);
                       input_landmark_list(jj,2)=potentialLandmarkList(ii,2); 
                       
                       xBot=pose(1);
                       yBot=pose(2);
                       xLandmark=input_landmark_list(jj,1);
                       yLandmark=input_landmark_list(jj,2);
                       
                       dist=sqrt((xBot-xLandmark)^2 +(yBot-yLandmark)^2);
                       ang=atan2d(yLandmark-yBot,xLandmark-xBot); 
                       ang=ang-pose(3);
                       
                       reobservedLandmarkList=[reobservedLandmarkList;dist,ang,input_landmark_list(jj,4)];
                       
                    end
                                 
                    %break jj for loop
                    jj=size(input_landmark_list);
                    
                end
            end
            %if the potential landmark was not associated then add it to
            %the list
            if(flag==0)
               input_landmark_list=[input_landmark_list;potentialLandmarkList(ii,:),1,0]; 
            end
       
        end
        
    end
    
    outputLandmarkList=input_landmark_list; 
end