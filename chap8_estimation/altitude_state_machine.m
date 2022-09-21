function y = altitude_state_machine(u,P)
% Define the altitude-control state machine to determine type of
% longitudinal control depending on current altitude
persistent altitude_state
    % Read Inputs
    h = u(1); h_c = u(2);
    
    % Determine zone
    % In order to determine the zone, you will need to follow the state
    % machine structure in the book, and use the conditions on each zone to
    % determine where the vehicle is, and then set the alitutde_state
    % variable to the following:
%     altitude_state = 1; % Take-off zone
%     altitude_state = 2; % Climb zone
%     altitude_state = 3; % Altitude hold zone
%     altitude_state = 4; % Descend zone

    % initialize persistent variable
       if isempty(altitude_state)
       altitude_state =1;
   end
   
   switch altitude_state
       case 1
           if h>=P.altitude_take_off_zone   
            altitude_state = 2;
           end
           
       case 2
        if h<P.altitude_hold_zone
            altitude_state = 1;
        elseif h>=(h_c-P.altitude_hold_zone)
            altitude_state = 3;
        end
        
       case 3
           if h<(h_c-P.altitude_hold_zone)
               altitude_state = 2;
           elseif h>=(h_c+P.altitude_hold_zone)
               altitude_state = 4;
           end
           
       case 4
           if h < (h_c+P.altitude_hold_zone)
               altitude_state = 3;
           end
   end
   
    
% persistent initialize_integrator;
%    if isempty(initialize_integrator),
%    
%         if h<=P.altitude_take_off_zone,     
%             altitude_state = 1;
%         elseif h<=h_c-P.altitude_hold_zone, 
%             altitude_state = 2;
%         elseif h>=h_c+P.altitude_hold_zone, 
%             altitude_state = 3;
%         else
%             altitude_state = 4;
%         end
%         initialize_integrator = 1;
%    end

    
    
    
y = altitude_state;