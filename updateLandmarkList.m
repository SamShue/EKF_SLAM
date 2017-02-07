function[updated_landmark_list] = updateLandmarkList(state_vector, input_landmark_list)    
%UPDATELANDMARKLIST The function will update the input_landmark_list x and
%y coordinates based on the thier index within the state_vector. It will
%then output the updated input_landmark_list as updated_landmark_list
%
%STATE_VECTOR Will hold landmark coordinates that the input_landmark_list
%will be updated with.
%
%INPUT_LANDMARK_LIST Will hold landmark coordinates with indexes that
%represent thier position in the state_vector. If a input_landmark_list
%coordinate does not match the indexed state_vector coordinate then the
%input_landmark_list will be updated using the state_vector
%
%UPDATED_LANDMARK_LIST The output will be the updated input_landmark_list
%that was updated based on the state vector 

    landmark_list=input_landmark_list; 

    if(size(state_vector)>3)
        %loop through all of the coordinates in the confirmed landmark list
        for ii=4:size(state_vector)
            %update the landmark_list coordinates based on the confirmed
            %landmark list coordinates 
            for jj=1:size(landmark_list)  %FIX: will search through entire list even after it finds the coordinates
                if(landmark_list(4,jj)==ii)
                    landmark_list(1,jj)=state_vector(ii);
                    landmark_list(2,jj)=state_vector(ii+1);
                    jj=size(landmark_list); %NOT TESTED
                end
            end 
            %each pair of coordinates occupies two locations in the vector
            %so need to iterate location by 2
            ii=ii+1;
        end
        
    end
    
    updated_landmark_list=landmark_list; 
end 