clear all
close all
clc
%% Creat your object
initial_states=[0 0 0 0];
initial_inputs=[pi/10 12];

Lambo=Car(initial_states,initial_inputs);

N = 200;

way_points = generate_trajectory(0.05);

myTrajectory = Trajectory(way_points);

for i = 1: N
    figure(1);
    
    myTrajectory.nearest_points(Lambo);
    myTrajectory.poly_fit;

    myTrajectory.show;

    Lambo.show;
    xlim([-230 230])
    ylim([-230 230])
    Lambo.update_state;
    
    
    
end