clear all
close all
clc
%% Creat your object
initial_states=[0 0 0 0];
initial_inputs=[pi/10 12];

Lambo=Car(initial_states,initial_inputs);

N = 200;


for i = 1: N
    figure(1);

    
    Lambo.show;
    xlim([-200 200])
    ylim([-200 200])
    Lambo.update_state;
    
    
    
end