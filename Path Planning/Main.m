clear all
close all
clc
%% Creat your object
initial_states=[1000 -1000 -pi/2 0];
initial_inputs=[100 0];

Lambo=Car(initial_states,initial_inputs);
Lambo.Goal=[-1500;1500];
Lambo.obstcle_C=[0;0];
Lambo.obstcle_R=400;

Lambo.Obsplot;
hold on
simulation_time=5;

phi_d=pi;
figure(1);

input=[1;1];
X=[];
Y=[];
State=[];
Time=[];
time=0;
while (norm(input)>1)
time=time+Lambo.ts;
Time=[Time;time];


input=Lambo.Controler;
Lambo.update_input(input);    
Lambo.update_state;

figure(1);
Lambo.Carplot;
hold off
xlim([-1800 1800])
ylim([-1800 1800])

[x,y,phi,si]=Lambo.state_unpack;
State=[State;x y phi si];


grid on;




hold on
plot(State(:,1),State(:,2),'Linewidth',2)
hold off

% figure(2);
% plot(Time,State(:,4),'b');
% hold on
% xlim([0 max(Time)])
end