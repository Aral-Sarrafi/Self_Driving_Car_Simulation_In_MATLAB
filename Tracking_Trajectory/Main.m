close all
clc
%% Creat your object
initial_states=[0 -225 0 0];
initial_inputs=[0 12];

Lambo=Car(initial_states,initial_inputs);

N = 200;
win = 100;
way_points = generate_trajectory(0.05);

myTrajectory = Trajectory(way_points);

for i = 1: N
    figure(1);
    
    myTrajectory.nearest_points(Lambo);
    myTrajectory.poly_fit(Lambo);
    myTrajectory.compute_error;
    myTrajectory.show(Lambo);
    
    myTrajectory.cte;
    

    Lambo.show;
    Lambo.PID_Controller(myTrajectory.cte);
    Lambo.control_inputs(1);
    xlim([-225 225])
    ylim([-225 225])
    Lambo.update_state;
    
    
%     figure(2);
%     
%     myTrajectory.show(Lambo);
%     
%     [x,y,~,~] = Lambo.state_unpack;
%     
%     Lambo.show;
% 
%     xlim([x - win x + win])
%     ylim([y - win y + win])

   
end