function [observed_LL, output_landmark_list]= getLandmark(landmark_list,laserdata,pose)
%w = warning ('off','all');
%GETLANDMARK Determine the location of wall landmarks based on a laser
%scan and the pose of the turtlebot. Then determine if that wall landmark seen
%by this sweep of the laser scan has been previously seen by the robot
%based on the landmark_list. If so then increment the count of that wall
%landmark within the landmark_list, else add the new wall landmark to the
%landmark_list. Output the updated landmark_list as output_landmark_list
%Also, if a landmark has been reobserved from the current
%laserdata and that landmark has been observed a number of times greater
%than the consensus then determine the distance of the robot from that
%landmark and the angle between the robot and the landmark with respect to
%the world frame. Output the distance, angle, and the index of the
%reobserved landmark as observed_LL
%
%LANDMARK_LIST Contains the x and y coordinate of each landmark, the number
%of times the landmark has been observed, and the index of the landmark in
%the state vector (index will be 0 if landmark has not been added to
%statevector). If new landmarks are observed or past landmarks are
%reobserved then the list is updated accordingly.
%
%LASERSCAN Contains the laser scan readings from the kinect sensor on the
%turtlebot. The data provided by laserscan is operated on to determine the
%locations of walls within the laserscan data. 
%
%POSE Contains the x,y,theta of the turtlebot in relation to the world
%frame. Used to apply rotation matrix to landmarks that are initally found
%in the LASERSCAN data converting them from the turtlebots from to the
%world frame. 
%
%OBSERVED_LL A list of landmarks that have already been observed enough
%times to meet the consensus and have also been reobserved from the current
%laserscan data. observed_LL is in the form of distance from the robot to the
%landmark, angle of the robot to the landmark with respect to the world
%frame, and index of the landmark in the state vector 
%
%OUTPUT_LANDMARK_LIST Contains the updated landmark list based on the new
%reading from the laserscan. Data is of the same format as landmark_list. 
%

    timeout= 5; %number of times program will search through a laserscan data before stopping
    iteration=1; %counts the current number that the program has looped
    num_samples =25; %number of points to be initally sampled and used to create inital line of best fit
    degrees = 5;%degree space that inital samples can be taken from (5 on both sides)
    distance=.1; %max distance that a point can be away from inital line of best fit and still be included in the line
    consensus = 250; % number of points needed to be near line of best fit in order for line to be approved 
    confirmed_consensus=10; %number of times a wall must be seen before it will be indexed on the state vector
    
    association_threshold=1;   % Threshold value for reassociated observed landmarks to recorded landmarks
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
    
    
    %used for the prefiltered landmark list towards bottom of code
    pfLL_index=[];
    pfLL=[];


    %update landmark_list based on confirmed_landmark_list
    


    world_line_list=[]; 
    
    
    laserdata.Ranges=removeNAN(laserdata.Ranges); %remove NAN readings from scan data
    
    counter=1;
    landmark_line_list=zeros(20,2); %prabably want to make this dynamic, it doesnt seem fesable for there to be a case where more then 10 walls will be seen at once
    
    
    unassociated_readings = readCartesian(laserdata); %convert laser readings to cartesian plane and set unassocaited_readings 
    samples=[]; 
    
    while(isempty(unassociated_readings)==false && iteration<timeout && length(unassociated_readings)>consensus && length(unassociated_readings)>num_samples ) %loop while there are still unassocated readings, less than x trails have taken place, and there are still more unassociated readings then the consensus size 
        
        samples=findInitSamples(unassociated_readings,degrees,num_samples); %get random samples within degrees of each other
        
        
        %figure(1);
        %ex=plot(data); %needed for function below
        %xlim=get(gca,'XLim'); %needed to extend the best fit line created in getLine()
        %disp(xlim);
        
        
        xlim=[0 10]; %used for graphing purposes
        if(~isempty(samples))
            sample_line=getSampleLine(samples,xlim); 
    
            [unassociated_readings, landmark_line]=getLandmarkLine(sample_line,unassociated_readings,distance,consensus,xlim);
            samples=[];
        end
    
    
       % scatter(samples(:,1),samples(:,2)); hold on;
       %line([sample_line(1,1) sample_line(1,2)],[sample_line(2,1),sample_line(2,2)])
       % plot(data); 
       % line([landmark_line(1,1),landmark_line(1,2)],[landmark_line(2,1),landmark_line(2,2)]);
       % scatter(unassociated_readings(:,1),unassociated_readings(:,2));

    
    
    
    
        if(~isempty(landmark_line))
            %line([landmark_line(1,1),landmark_line(1,2)],[landmark_line(2,1),landmark_line(2,2)]);

            landmark_line_list(2*counter-1,1)=landmark_line(1,1);
            landmark_line_list(2*counter-1,2)=landmark_line(1,2);
        
            landmark_line_list(2*counter,1)=landmark_line(2,1);
            landmark_line_list(2*counter,2)=landmark_line(2,2);
            counter=counter+1;
        end
        iteration=iteration+1; 
    end
    
    
    
    
    
    
% figure(1);   
 %  plot(data); hold on;


%line([landmark_line_list(3,1),landmark_line_list(3,2)],[landmark_line_list(3+1,1),landmark_line_list(3+1,2)]); hold on;


%    if(~isempty(samples))
 %       for i=1 : length(landmark_line_list)/2
   %         line([landmark_line_list(2*i-1,1),landmark_line_list(2*i-1,2)],[landmark_line_list(2*i,1),landmark_line_list(2*i,2)]); hold on;
  %      end
  %  end     

%hold off; 
    counter=counter-1;
    real_landmark_line_list=zeros(2*counter,2);


    for i=1 : length(real_landmark_line_list)

        real_landmark_line_list(i,1)=landmark_line_list(i,1);
        real_landmark_line_list(i,2)=landmark_line_list(i,2);
    end 

    


%fix that real_landmark_line_list and landmark_line_list stuff
    
   
    
    
    x = pose(1);
    y = pose(2);
    theta =pose(3);
   
   % [x,y,z];

      
   
    
    formated_landmark_line_list=zeros(length(real_landmark_line_list),2);
    
    for i=1 : length(formated_landmark_line_list)/2
        formated_landmark_line_list(2*i-1,1)= real_landmark_line_list(2*i-1,1);
        formated_landmark_line_list(2*i,1)= real_landmark_line_list(2*i-1,2); 
       
        formated_landmark_line_list(2*i-1,2)= real_landmark_line_list(2*i,1);
        formated_landmark_line_list(2*i,2)= real_landmark_line_list(2*i,2);
                
    end
    
%    if(theta~=0)
 %       formated_landmark_line_list
  %  end
    
    for i = 1 : 1 : length(formated_landmark_line_list) % Loops through observed landmarks for this /scan
        
        rot = [cosd(theta) -sind(theta) x ; sind(theta) cosd(theta) y; 0 0 1];
        tr = [formated_landmark_line_list(i,1);formated_landmark_line_list(i,2); 1];
        transform = rot*tr;
        final_transform = transform';

        
        world_line_list = [world_line_list; final_transform];
        
    end
%    if(theta~=0)
 %       world_line_list
  %  end
    %disp('+++++++++++++++++++++++++++');
    %disp(world_line_list);
    %disp('---------------------------');
    %disp(formated_landmark_line_list);
    %disp('+++++++++++++++++++++++++++');
   
    
    possible_landmark_list=[length(world_line_list),2];
    
    
    for i = 1 : length(world_line_list)/2
        
        x1=world_line_list(2*i-1,1);
        x2=world_line_list(2*i,1);
        y1=world_line_list(2*i-1,2);
        y2=world_line_list(2*i,2); 
        
        m= (x1-x2)/(y1-y2); 
        b= x1-m*y1;
        
        m_perp=-1*1/m; %redundent
        b_perp=0; 
        
        syms x y 
        eqn1 = x-m*y == b;
        eqn2 = x+1/m*y == 0; 
        
        [A,B]=equationsToMatrix([eqn1,eqn2],[x,y]);
        
        X = linsolve(A,B);
        possible_landmark_list(i,1)=X(1);
        possible_landmark_list(i,2)=X(2);
        
    end
    %need to determine shortest distance from each line on world plane to the orig.
    %(assuming 0,0)
    
    
  
     
   % disp('possible_landmark_list');
  %  disp(possible_landmark_list);
   
         if isempty(landmark_list) %if list is empty, add landmark to list
             if(~isempty(possible_landmark_list))
                  x1 = possible_landmark_list(1,1);
                  y1 = possible_landmark_list(1,2);
                
                  x2 = 0;
                  y2 = 0;
                 
                
                  distance = sqrt(((x2-x1)^2)+((y2-y1)^2));
               
                  
              landmark_list = [landmark_list; possible_landmark_list(1,:),1,0];
             end 
         else
              distance_list = [];
              checker=1;
              pfLL_index=[];
              for m=1: size(possible_landmark_list)
                  %disp('possible landmark list');
                  %disp(possible_landmark_list);
                  x1 = possible_landmark_list(m,1);
                  y1 = possible_landmark_list(m,2);
                  checker=0;
                  for n=1: size(landmark_list)
                    x2 = landmark_list(n,1);
                    y2 = landmark_list(n,2);
                    distance = sqrt(((x2-x1)^2)+((y2-y1)^2));
                    if distance < association_threshold
                        checker=1;
                      %  landmark_list(n,:)=possible_landmark_list(m,:); %seems....odd
                          
                      landmark_list(n,3)=landmark_list(n,3)+1; %increment
                      
                      %check to see if the landmark that was observed has
                      %already been confirmed, if so then add it to the
                      %pfLL index
                      if(landmark_list(n,3)>confirmed_consensus&&landmark_list(n,4)~=0)
                        pfLL_index=[pfLL_index,n];
                      end
                      %count
                          
                    end
          
                  end
                  
                  if checker==0
                     
                     distance_list=[distance_list,m];
                  end
                   
              
              end
            
            
            if ~isempty(possible_landmark_list)
              if ~isempty(distance_list)
              %  disp('distance list');
               % disp(distance_list);
                add_these=[possible_landmark_list(distance_list,:) ones(length(distance_list),1) zeros(length(distance_list),1)];
              %  disp('add these');
              %  disp(add_these);
                
                
                
                if add_these(1,1)==0 && add_these(1,2)==2
                    
                else
                    landmark_list=[landmark_list;add_these ];
                end
                
              end
              
            end 
             
           
              
            
              

         end
         
      
      for ii=1:size(landmark_list)
         % disp('land length');
         % disp(size(landmark_list));
          if(landmark_list(ii,3)>confirmed_consensus && landmark_list(ii,4)==0)
            %landmark_list(i,4)=size(confirmed_landmark_list,1)+1;
            %confirmed_landmark_list=[confirmed_landmark_list; landmark_list(i,1); landmark_list(i,2)] ;
            
            
            
            
            pfLL_index=[pfLL_index,ii];
            landmark_list(ii,4)=max(landmark_list(:,4))+1;
            
          end
      end
      
      
      pfLL=[];
      if(~isempty(pfLL_index))
         
          pfLL=zeros(length(pfLL_index),3); %pfLL_index is in terms of the landmark_list not the confirmed list
          pfLL(:,3)=landmark_list(pfLL_index,4);
          
          for ii=1:size(pfLL) 
             x1=pose(1);
             x2=landmark_list(pfLL_index(ii),1);
             y1=pose(2);
             y2=landmark_list(pfLL_index(ii),2); 
             
             d=sqrt((x1-x2)^2 +(y1-y2)^2); 
             pfLL(ii,1)=d;
             %gives angle in degrees between vectors counterclockwise from
             %y1x1 to y2x2
             %deg=atan2d((x1*y2-y1*x2),(x2*x1+y2*y1));
             deg=atan2d(y2-y1,x2-x1); 
             deg=deg-pose(3); 
             pfLL(ii,2)=deg;
              
          end
         
      %    for ii=1:size(pfLL)
       %       pfLL
      end 
              
    %  confirmed_landmark_list(1,1)=pose(1);
    %  confirmed_landmark_list(2,1)=pose(2);
    %  
    %  confirmed_landmark_list(3,1)=pose(3); 
      
         
      %prefiltered 
      
      
        
      
    
      %set(gcf,'Visible','on');
    %set(0,'DefaultFigureVisible','on');
     % figure(2);
      %   s=scatter(landmark_list(:,2),landmark_list(:,1));hold on
       %  scatter(pose.Position.Y,pose.Position.X);
        % axis([-6 6 -6 6]);
         %hold off;
         
      %  x = pose.Position.X;
    %y = pose.Position.Y;
        

     %   disp('distance list:');
     %   disp(distance_list);
         
       %  disp('landmark list:');
       %  disp(landmark_list);
         
         
       
       %return confrimed and land_list
       
       % set(gcf,'Visible','On');
       % set(0,'DefaultFigureVisible','On');
       
       
       
       %basic selection sort for pfLL based on index
       pfLL = sort(pfLL,4); %added assignment after sort - Sam   
    
       %output_confirmed_landmark_list=confirmed_landmark_list;
       output_landmark_list=landmark_list;
       observed_LL=pfLL;
%        if(~isempty(observed_LL))
%         observed_LL(:,2) = wrapTo360(observed_LL(:,2));
%        end
       clf;
end

function y = removeNAN(x)
range_data = x; %temp variable to store the range data from the laser scan
ind =  find(isnan(range_data)); %find the index of each NAN value in scan data
range_data(ind)=[]; %change the value of each NAN to [] 
y = range_data; 
end

function z = findInitSamples(x,degrees,num_samples) 
points=x;
[randomSample,idxRandSamp] = datasample(points,1,'Replace',false); %take a random sample
%check if there are at least inital_sample points within degrees of randomSample 

randomSampleDegreeMax = atand(randomSample(1,2)/randomSample(1,1)) + degrees/2;
randomSampleDegreeMin = atand(randomSample(1,2)/randomSample(1,1)) - degrees/2;

sample_points = zeros(2,num_samples); 
sample_pointsIdx = zeros(1,num_samples); 

for i = 1 : length(points)-1   %not exactly random, imrpve
    deg = atand(points(i,2)/points(i,1));
    if( deg>randomSampleDegreeMax || deg <randomSampleDegreeMin)
      points(i,:)=0;
    end
end


points=points(points~=0);
points=reshape(points,[],2);
%need to add if statement for redudency in case somehow the num_samples is
%greater than the sample points

sample_points=[];
if(length(points)>num_samples)
    [sample_points,sample_pointsIdx] = datasample(points,num_samples,'Replace',false);
end

z=sample_points;
end 

function z=getSampleLine(samples,xlim)

%  set(gcf,'Visible','off');
%   set(0,'DefaultFigureVisible','off');


hold off;
mhm=scatter(samples(:,2),samples(:,1));
h=lsline;
%ee=polyfit(samples(:,2),samples(:,1),1);




YDat=h.YData;   
XDat=h.XData;
%XDat=[1;2];
%YDat=[polyval(ee,XDat(1));polyval(ee,XDat(2))]; 

m = (XDat(2)-XDat(1))/(YDat(2)-YDat(1));

n= XDat(2)-YDat(2)*m;
y1=m*xlim(1)+n;
y2=m*xlim(2)+n;

%line([xlim(1) xlim(2)],[y1,y2])

sample_line=[xlim(1) xlim(2); y1 y2];
%disp(sample_line)


z=sample_line;
end

function [points, land_line]= getLandmarkLine(line,unassociated_readings,distance,consensus,xlim)
readings=unassociated_readings; 

v1=[line(1,1),line(2,1),0];
v2=[line(1,2),line(2,2),0];

idx = zeros(length(readings),1);

for i= 1 : length(readings)-1

    p=[readings(i,:),0];

    a=v1-v2;
    b=p-v2;
    d =norm(cross(a,b))/norm(a);

    if d<distance
        idx(i)=i;
    end
end

idx=idx(idx~=0);
rem=readings;
landmark_line=[];
if length(idx)>consensus
   
    points_to_be_removed=readings(idx,:);
    rem=readings; %remaining unassociated landmarks 
    rem(idx,1)=NaN;
    rem(idx,2)=NaN;
    
    rem= rem(~isnan(rem));
    
    
    rem=reshape(rem,[],2);
%     set(gcf,'Visible','off');
%     set(0,'DefaultFigureVisible','off');
     hold off    
     merp=  scatter(points_to_be_removed(:,1),points_to_be_removed(:,2)); 
     h=lsline;
     YDat=h.XData;   
     XDat=h.YData;
    
    
 %  ee=polyfit(points_to_be_removed(:,1),points_to_be_removed(:,2),1);

 %   XDat=[1;2];
 %   YDat=[polyval(ee,XDat(1));polyval(ee,XDat(2))];
    
    
    m = (XDat(2)-XDat(1))/(YDat(2)-YDat(1));
    n= XDat(2)-YDat(2)*m;
    y1=m*xlim(1)+n;
    y2=m*xlim(2)+n;



    landmark_line=[xlim(1) xlim(2); y1 y2];
%landmark_line=[h.XData(1), h.YData(1);h.XData(2),h.YData(2)];
    



end



points=rem;
land_line=landmark_line;
w = warning ('on','all');
end