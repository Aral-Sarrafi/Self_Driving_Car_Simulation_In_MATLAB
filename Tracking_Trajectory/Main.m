close all
clc
%% Plot options

x0=0;
y0=0;
gold=(1+sqrt(5))/2;
width=10;
height=4;
FontSize=14;
FontSize_axis=16;
font_weight='bold';

%% Creat your object
initial_states=[0 -225 0 0];
initial_inputs=[0 12];

Lambo=Car(initial_states,initial_inputs);

N = 200; % Number of time steps for the simulation.
win = 120;
way_points = generate_trajectory(0.05);

myTrajectory = Trajectory(way_points);

delay_time = 0;
filename = 'Zoomed.gif';

for i = 1: N
    h = figure(1);
    
    
    myTrajectory.nearest_points(Lambo);
    myTrajectory.poly_fit(Lambo);
    myTrajectory.compute_error;
    myTrajectory.show(Lambo);
    myTrajectory.cte;
    

    Lambo.show;
    Lambo.PID_Controller(myTrajectory.cte);
    Lambo.control_inputs(1);
    %% uncomment to zoom in
    [x,y,~,~] = Lambo.state_unpack;

    xlim([x - win x + win])
    ylim([y - win y + win])
%     xlim([-225 225])
%     ylim([-225 225])
    Lambo.update_state;
    grid on;
    
    %% Figure configurations
    set(gca,'fontsize',FontSize_axis,'FontName','Times','fontweight','bold')
    
    title('\bf{Tracking Trajectory}',...
    'FontUnits','points',...
    'FontWeight',font_weight,...
    'interpreter','latex',...
    'FontSize',FontSize,...
    'FontName','Times')

    xlabel('\bf{x(m)}',...
    'FontUnits','points',...
    'FontWeight',font_weight,...
    'interpreter','latex',...
    'FontSize',FontSize,...
    'FontName','Times')

    ylabel('\bf{y(m)}',...
    'FontUnits','points',...
    'FontWeight',font_weight,...
    'interpreter','latex',...
    'FontSize',FontSize,...
    'FontName','Times')

    %% Make the gif file
    
    frame = getframe(h);
    img = frame2im(frame);
    [AA,map] = rgb2ind(img,256); 
	if i == 1
		imwrite(AA,map,filename,'gif','LoopCount',Inf,'DelayTime',delay_time);
	else
		imwrite(AA,map,filename,'gif','WriteMode','append','DelayTime',delay_time);
	end
end