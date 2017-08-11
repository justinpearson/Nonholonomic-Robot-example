classdef NonholonomicVehicle < handle
    %NONHOLONOMICVEHICLE A model of a Roomba robot
    %   A model of a 2-wheel robot where one sets the wheel speeds.
    %   Use the timestep() method to advance time by the timestep increment
    %   dt. The wheelbase and max-wheel-speeds are for the iRobot Create
    %   robot.
    
    properties
        Vr = 0; % right wheel velocity (m/s)
        Vl = 0; % left  wheel velocity (m/s)
        L = .265; % wheelbase of Roomba = 265mm
        MAX_WHEEL_SPEED = .5; % max speed of wheels = 500mm/s        
        R = inf; % radius of circle Roomba is driving on
        W = 0; % angular velocity of Roomba 
        X = 0; % X-coord of Roomba's position
        Y = 0; % Y-coord of Roomba's position
        TH = 0; % angle Roomba is pointing
        DT = 0; % timestep of simulation
        
    end
    
    methods
        
        function obj = NonholonomicVehicle(dt,x0,y0,th0,vl,vr)
            obj.DT = dt;
            obj.X = x0;
            obj.Y = y0;
            obj.TH = th0;
            obj.setWheelVelocities(vl,vr);
        end
        
        function w = limitSpeed(obj,v)
            w = max(min(v,obj.MAX_WHEEL_SPEED),-obj.MAX_WHEEL_SPEED);
        end
        
        function obj = timestep(obj)
            dt = obj.DT; % timestep, sec
            v = (obj.Vr + obj.Vl)/2; % not sure if this is right
            obj.X = obj.X + v*cos(obj.TH)*dt;
            obj.Y = obj.Y + v*sin(obj.TH)*dt;
            obj.TH = obj.TH + dt*obj.W;
        end
        
        function obj = setWheelVelocities(obj,vl,vr)
            obj.Vl = obj.limitSpeed(vl);
            obj.Vr = obj.limitSpeed(vr);
            obj.W = (vr-vl)/obj.L;
            obj.R = obj.L/2 * (vl+vr)/(vr-vl);
        end
        
        function obj = setAngularVelocityAndRadius(obj,w,r)
            % Warning: w and r will not be achievable if they result in
            % wheel speeds that exceed MAX_WHEEL_SPEED.
            obj.W = w;
            obj.R = r;
            obj.Vr = obj.limitSpeed(w*(r+obj.L/2));
            obj.Vl = obj.limitSpeed(w*(r-obj.L/2));
        end
        
    end
    
end

