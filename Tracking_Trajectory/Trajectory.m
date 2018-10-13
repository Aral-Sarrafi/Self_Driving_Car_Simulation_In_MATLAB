classdef Trajectory<handle
   
    properties
        
        WP_World_Coordinate;
        nump = 8;
        Nearst_Points;
        Nearst_Points_C;
        cte;
        show_nearest = true;
        show_fit = true;
        Coeff_fit;
        
    end
    
    methods
        %% Constructor
        function obj = Trajectory(way_points)
            
            obj.WP_World_Coordinate = way_points;
            obj.Nearst_Points = zeros(obj.nump,2);
            obj.Nearst_Points_C = zeros(obj.nump,2);
            obj.cte = 0;
            obj.Coeff_fit = [0 0 0];
            
        end
        
        %% Show waypoints
        function show(obj,Car)
            
            plot(obj.WP_World_Coordinate(:,1), obj.WP_World_Coordinate(:,2),"Linewidth",1.5)
            hold on
            plot(obj.WP_World_Coordinate(:,1), obj.WP_World_Coordinate(:,2),'.','Color',[0.8500, 0.3250, 0.0980],"MarkerSize",8)
            hold on
            
            if (obj.show_nearest)
                plot(obj.Nearst_Points(:,1), obj.Nearst_Points(:,2),'sq','MarkerFaceColor','b')
                hold on
            end
            
            if (obj.show_fit)
                
                xmin = min(obj.Nearst_Points_C(1,:))-100;
                xmax = max(obj.Nearst_Points_C(1,:))+100;
                
                x = xmin:0.1:xmax;
                y =polyval(obj.Coeff_fit,x);
                C_Points = [x' y'];
                
                W_points = Car_to_W_Coordinate_system(C_Points,Car);
                
                plot(W_points(:,1), W_points(:,2),'r-.',"Linewidth",2);
                hold on
                
                
            end
            
        end
        
        %% Transform the point to car coordinate system
        function W_to_Car_Coordinate_system(obj,Car)
            
            [x,y,phi,~] = Car.state_unpack;
            
            tranlate = obj.Nearst_Points - [x,y]; % Setting the car at the origin
            
            obj.Nearst_Points_C = ([cos(phi) sin(phi); -sin(phi) cos(phi)] * tranlate')';
             
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
        function poly_fit(obj,Car)
            
            obj.W_to_Car_Coordinate_system(Car);
            
            obj.Coeff_fit = polyfit(obj.Nearst_Points_C(:,1),obj.Nearst_Points_C(:,2),3);
            
        end
        
        %% This function computes the track error which will used later by the PID controller
        function compute_error(obj)
            
            obj.cte = obj.Coeff_fit(end);
            
        end
        
    end
end

%% Transfomr the point to car coordinate system
        
 function W_points = Car_to_W_Coordinate_system(C_Points,Car)
            
    [x,y,phi,~] = Car.state_unpack;
            
    Rotate = ([cos(phi) -sin(phi); sin(phi) cos(phi)] * C_Points')'; % Setting the car at the origin
           
    W_points = Rotate + [x,y];
  
end