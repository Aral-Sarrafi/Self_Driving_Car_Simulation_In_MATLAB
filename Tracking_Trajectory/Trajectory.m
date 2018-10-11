classdef Trajectory
   
    properties
        
        WP_World_Coordinate;
        Nearst_Points
        
    end
    
    methods
        %% Constructor
        function obj = Trajectory(obj, way_points)
            
            obj.WP_World_Coordinate = way_points;
            
        end
        %% Coordinate change
        wp_Car_Coordinate = world_to_car(obj,Car)
        %% This function finds the nearest points to the car
        function nearest_points(obj,Car, nump)
           
            obj.Nearst_Points = 0;
            
        end
        
        %% This function fits a polynomial to the nearest points
        function coeff = poy_fit(obj,points)
            coeff = 0;
        end
        
        %% This function computes the track error which will used later by the PID controller
        function cte = compute_error(obj,Car, coeff)
            
            cte = 0;
            
        end
        
    end
    
    
    
    
end