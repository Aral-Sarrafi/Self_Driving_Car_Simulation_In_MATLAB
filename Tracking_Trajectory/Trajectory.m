classdef Trajectory<handle
   
    properties
        
        WP_World_Coordinate;
        nump = 8;
        Nearst_Points;
        show_nearest = true;
        show_fit = true;
        Coeff_fit;
        
    end
    
    methods
        %% Constructor
        function obj = Trajectory(way_points)
            
            obj.WP_World_Coordinate = way_points;
            obj.Nearst_Points = zeros(obj.nump,2);
            obj.Coeff_fit = [0 0 0];
            
        end
        
        %% Show waypoints
        function show(obj)
            
            plot(obj.WP_World_Coordinate(:,1), obj.WP_World_Coordinate(:,2))
            hold on
            plot(obj.WP_World_Coordinate(:,1), obj.WP_World_Coordinate(:,2),'sq')
            hold on
            
            if (obj.show_nearest)
                plot(obj.Nearst_Points(:,1), obj.Nearst_Points(:,2),'sq','MarkerFaceColor','b')
                hold on
            end
            
            if (obj.show_fit)
                
                xmin = min(obj.Nearst_Points(1,:))-20;
                xmax = max(obj.Nearst_Points(1,:))+20;
                
                x = xmin:0.1:xmax;
                y =polyval(obj.Coeff_fit,x);
                plot(x, y)
                hold on
                
            end
            
        end
        %% Transfomr the point to car coordinate system
        
        function points_Car_coordinate = W_to_Car_Coordinate_system(obj,Car)
            
            [x,y,phi,~] = Car.state_unpack;
            
            tranlate = obj.Nearst_Points - [x,y]; % Setting the car at the origin
            
            points_Car_coordinate = [cos(phi) -sin(phi);sin(phi) cos(phi)] * tranlate';
            
            points_Car_coordinate = points_Car_coordinate';
             
        end
        
        %% This function finds the nearest points to the car
        
        function nearest_points(obj,Car)
            
            [x,y,~,~] = Car.state_unpack;
            
            point_list = zeros(size(obj.WP_World_Coordinate,1),3);
           
                
            for i = 1:size(obj.WP_World_Coordinate,1)
                
                    dist = sqrt((obj.WP_World_Coordinate(i,1) - x)^2+ (obj.WP_World_Coordinate(i,2) - y)^2);
                    
                    point_list(i,1) = obj.WP_World_Coordinate(i,1);
                    point_list(i,2) = obj.WP_World_Coordinate(i,2);
                    point_list(i,3) = dist;
            end
            
            point_list = sortrows(point_list,3);
                
            
           
            obj.Nearst_Points = point_list(1:obj.nump,1:2);
            
        end
        
        %% This function fits a polynomial to the nearest points
        function poly_fit(obj)
            
            obj.Coeff_fit = polyfit(obj.Nearst_Points(:,1),obj.Nearst_Points(:,2),3);
            
        end
        
        %% This function computes the track error which will used later by the PID controller
        function cte = compute_error(obj,Car, coeff)
            
            cte = 0;
            
        end
        
    end
    
    
    
    
end