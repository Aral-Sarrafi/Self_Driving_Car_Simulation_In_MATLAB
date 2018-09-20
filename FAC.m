classdef FAC<Car
    properties
        kp
        ki
        kd
        u
    end
    
    methods
        
        function obj=FAC();
            obj = obj@Car(obj.states,obj.inputs);
            
            obj.kp = 1;   
        end
        
        function u=execute(obj,phi_d)
            [~,~,phi,~]=state_unpack(obj)
            e=phi_d-phi;
            e=atan2(sin(e),cos(e));
            u=kp*e;
        end
   
    end

end