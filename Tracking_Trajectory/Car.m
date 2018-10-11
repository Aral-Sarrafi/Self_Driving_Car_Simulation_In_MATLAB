classdef Car<handle
    
    properties
        states
        inputs
        track=2057/10;
        wheel_base=2665/10;
        wheel_diameter=66.294;
        wheel_width=25.908;
        ts=0.1;
        Ei=0;
        E_1=0;
        obstcle_C
        obstcle_R
        Goal
       
    end
    
    
    methods
        % Constructor
        function obj=Car(states,inputs)
           obj.states=states;
           obj.inputs=inputs;
           
           obj.obstcle_C=[0;0];
           obj.obstcle_R=100;
          
        end
        
        function Obsplot(obj)
            plot_circle(obj.obstcle_C,obj.obstcle_R)
            
        end


        %Plot My car
        function Carplot(obj)

        
          %Plot the Body of the Car
          pc1=[-obj.wheel_base/2;obj.track/2;1];
          pc2=[obj.wheel_base/2;obj.track/2;1];
          pc3=[obj.wheel_base/2;-obj.track/2;1];
          pc4=[-obj.wheel_base/2;-obj.track/2;1];
          pc=[pc1 pc2 pc3 pc4];
          
          pw1=[-obj.wheel_diameter/2;obj.wheel_width/2;1];
          pw2=[obj.wheel_diameter/2;obj.wheel_width/2;1];
          pw3=[obj.wheel_diameter/2;-obj.wheel_width/2;1];
          pw4=[-obj.wheel_diameter/2;-obj.wheel_width/2;1];
          pwheel=[pw1 pw2 pw3 pw4];
          
          [x,y,~,si]=state_unpack(obj);
          %Plot the center of the car
          plot(x,y,'o','Markersize',15,'Markerface','b')
          hold on
          
          plot_circle(obj.obstcle_C,obj.obstcle_R)
          
          
          R_car_world=carf_to_wf(obj);
          pcw=R_car_world*pc;
          
          

          
          plot([pcw(1,1) pcw(1,2)],[pcw(2,1) pcw(2,2)],'b','Linewidth',2)
          plot([pcw(1,2) pcw(1,3)],[pcw(2,2) pcw(2,3)],'b','Linewidth',2)
          plot([pcw(1,3) pcw(1,4)],[pcw(2,3) pcw(2,4)],'b','Linewidth',2)
          plot([pcw(1,4) pcw(1,1)],[pcw(2,4) pcw(2,1)],'b','Linewidth',2)
         
          set(gca,'units','centimeters')
          
          

          
          
          
          %plot the Back wheels
          %Plot the Front wheels
          
          for i=2:3
          R_wheel_to_car=wheel_frame_to_car(pc(:,i),si);
          
          pwheel=R_car_world*R_wheel_to_car*pwheel;
          
          
          plot([pwheel(1,1) pwheel(1,2)],[pwheel(2,1) pwheel(2,2)],'Color','red','Linewidth',2)
          plot([pwheel(1,2) pwheel(1,3)],[pwheel(2,2) pwheel(2,3)],'Color','red','Linewidth',2)
          plot([pwheel(1,3) pwheel(1,4)],[pwheel(2,3) pwheel(2,4)],'Color','red','Linewidth',2)
          plot([pwheel(1,4) pwheel(1,1)],[pwheel(2,4) pwheel(2,1)],'Color','red','Linewidth',2)
          

         
          pwheel=[pw1 pw2 pw3 pw4];
          
          end
          
          
          for i=1:3:4
          R_wheel_to_car=wheel_frame_to_car(pc(:,i),0);
          
          pwheel=R_car_world*R_wheel_to_car*pwheel;
          
          
          plot([pwheel(1,1) pwheel(1,2)],[pwheel(2,1) pwheel(2,2)],'Color','red','Linewidth',2)
          plot([pwheel(1,2) pwheel(1,3)],[pwheel(2,2) pwheel(2,3)],'Color','red','Linewidth',2)
          plot([pwheel(1,3) pwheel(1,4)],[pwheel(2,3) pwheel(2,4)],'Color','red','Linewidth',2)
          plot([pwheel(1,4) pwheel(1,1)],[pwheel(2,4) pwheel(2,1)],'Color','red','Linewidth',2)

         
          pwheel=[pw1 pw2 pw3 pw4];
          
          end
          
          
          
hold off

        end
        

       
        
        
        function R=carf_to_wf(obj)
            [x,y,phi,~]=state_unpack(obj);
            R=[cos(phi) -sin(phi) x;sin(phi) cos(phi) y;0 0 1];
        end
        

        
        function [x,y,phi,si]=state_unpack(obj)
            x=obj.states(1);
            y=obj.states(2);
            phi=obj.states(3);
            si=obj.states(4);
        end
        
        
        function [v,u]=input_unpack(obj)
            v=obj.inputs(1);
            u=obj.inputs(2);
        end
        
        function obj=update_state(obj)
            [x,y,phi,si]=state_unpack(obj);
            [v,u]=input_unpack(obj);
            
            x_next=x+obj.ts*v*cos(phi+si);
            y_next=y+obj.ts*v*sin(phi+si);
            phi_next=phi+obj.ts*v/(obj.wheel_base/10)*sin(si);
            si_next=si+obj.ts*u;
            obj.states=[x_next y_next phi_next si_next];
        end
        
        function obj=update_input(obj,input)
            obj.inputs=input;
        end
        
        function input=FAC(obj,phi_d)
            kp=.05;
            ki=0;
            kd=0;
            [~,~,phi,~]=state_unpack(obj);
            e=phi_d-phi;
            
            e=atan2(sin(e),cos(e));
            ei=obj.Ei+e;
            ed=e-obj.E_1;
            u=kp*e+ki*ei+ed;            
            input=[100 u]; 
            obj.Ei=ei;
            obj.E_1=e;
        end
        
        function input=GTG(obj)
            kp=.05;
            ki=0;
            kd=0;
            [x,y,phi,~]=state_unpack(obj);
            loc=[x;y];
      
            uGTG=obj.Goal-loc;
            phi_d=atan2(uGTG(2),uGTG(1));
            e=phi_d-phi;
            
            e=atan2(sin(e),cos(e));
            ei=obj.Ei+e;
            ed=e-obj.E_1;
            u=kp*e+ki*ei+ed; 
            if norm(uGTG)>5
              input=[100 u]; 
            else
              input=[0 0];
            end
            obj.Ei=ei;
            obj.E_1=e;
        end
        
        function input=Avoid_obstcle(obj)
            [x,y,~,~]=state_unpack(obj);
            uco=obj.obstcle_C-[x;y];
            theta1=pi;
            rotation1=[cos(theta1) -sin(theta1);sin(theta1) cos(theta1)];
            uAO=rotation1*uco;
            theta2=pi/2;
            rotation2=[cos(theta2) -sin(theta2);sin(theta2) cos(theta2)];
            uTAN=rotation2*uco;
            
            U=uTAN+0.25*uAO;
            
            phi_o=atan2(U(2),U(1));
            input=FAC(obj,phi_o);
        end
        
        function input=Controler(obj)
            [x,y,~,~]=state_unpack(obj);
            safety=obj.obstcle_R;
            if norm(obj.obstcle_C-[x;y])<obj.obstcle_R+safety;
                input=Avoid_obstcle(obj);
            else
                input=GTG(obj);
            end
        end


    end
    
end

        
function R=wheel_frame_to_car(pc,si)
            R=[cos(si) -sin(si) pc(1);sin(si) cos(si) pc(2);0 0 1];
end

function plot_circle(center,R)
X=[];
Y=[];
for t=0:pi/20:2*pi
    x=center(1)+R*sin(t);
    y=center(2)+R*cos(t);
    X=[X;x];
    Y=[Y;y];
end
plot(X,Y,'k','Linewidth',2)
set(gca,'units','centimeters')
end

