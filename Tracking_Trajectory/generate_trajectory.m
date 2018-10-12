function waypoints = generate_trajectory(resolution)
    num_points = ceil((2*pi)/resolution);
    
    waypoints = zeros(num_points,2);
    theta = 0:resolution:2*pi;
    
    a = 200;
    b = 200;
    
    waypoints(:,1) = a * cos(theta);
    waypoints(:,2) = b * sin(theta);
    

end