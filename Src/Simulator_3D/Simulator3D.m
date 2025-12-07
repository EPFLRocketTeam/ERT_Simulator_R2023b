classdef Simulator3D < handle
    
% -------------------------------------------------------------------------  
% Class properties
% -------------------------------------------------------------------------
   properties(Access = public)
      SimAuxResults; 
   end

   properties(Access = public)
      Rocket
      Environment
      SimOutput  
   end
   
   properties(Access = private)
      firstSimFlag = 1;
      tmpMargin
      tmpAlpha
      tmpCnAlpha
      tmpXcp
      tmpDragCoefficient
      tmpMass
      tmpCenterOfMass
      tmpInertiaLong
      tmpInertiaRot
      tmpDelta
      
      tmpNoseAlpha
      tmpNoseDelta
   end
   
% -------------------------------------------------------------------------  
% Constructor  
% -------------------------------------------------------------------------   
   methods
       
       function obj = Simulator3D(Rocket, Environment, SimOutput)
           if nargin == 0
               % TODO: Put default values or send warning message
           elseif nargin == 3
                obj.Rocket = Rocket;
                obj.Environment = Environment;
                obj.SimOutput = SimOutput;
           else
                error(['ERROR: In Simulator3D constructor, either no arguments '...
                    'or 3 arguments can be given. You gave ' num2str(nargin) '.']);
           end
 
           % Initialise Auxiliary results structure
           obj.SimAuxResults.margin = [];
           obj.SimAuxResults.alpha = [];
           obj.SimAuxResults.cnAlpha = [];
           obj.SimAuxResults.xcp = [];
           obj.SimAuxResults.dragCoefficient = [];
           obj.SimAuxResults.mass = [];
           obj.SimAuxResults.centerOfMass = [];
           obj.SimAuxResults.inertiaLong = [];
           obj.SimAuxResults.inertiaRot = [];
           obj.SimAuxResults.delta = [];
           
           obj.SimAuxResults.noseAlpha = [];
           obj.SimAuxResults.noseDelta = [];
       end
       
   end
     
% -------------------------------------------------------------------------  
% Dynamic Computation methods & Output functions
% -------------------------------------------------------------------------      
   methods(Access = protected)
       
        % --------------------------- 
        % Rail equations 
        % ---------------------------
    
        function S_dot = Dynamics_Rail_1DOF(obj, t, s)

            x = s(1); % position
            v = s(2); % speed

            % Rocket Inertia
            [mass,massRate] = Mass_Non_Lin(t,obj.Rocket); % mass

            % Environment
            g = 9.81;               % Gravity [m/s2] 
            [~, a, ~, rho, kinematicViscosity] = atmosphere(x*sin(obj.Environment.Rail_Angle),obj.Environment); % Atmosphere information (TODO: Include effect of humidity and departure altitude)

            % Force estimation

            % gravity
            G = -g*cos(obj.Environment.Rail_Angle)*mass;

            % Thrust 
            T = Thrust(t,obj.Rocket); % (TODO: Allow for thrust vectoring -> error)

            % drag
            dragCoefficient = drag(obj.Rocket, 0, v,kinematicViscosity, a); % (TODO: make air-viscosity adaptable to temperature)
            D = -0.5*rho*obj.Rocket.maxCrossSectionArea*dragCoefficient*v^2; % (TODO: define drag in wind coordinate system)

            totalForce = G + T*obj.Rocket.motorThrustFactor + D;

            % State derivatives
            
            x_dot = v;
            v_dot = 1/mass*(totalForce);
            if v_dot < 0
                x_dot = 0;
                v_dot = 0;
            end % <--- ajout

            if v_dot < 0
                x_dot = 0;
                v_dot = 0;
            end 

            S_dot = [x_dot; v_dot];
        end
       
        % --------------------------- 
        % 6DOF Flight Equations
        % ---------------------------
        
        function S_dot = Dynamics_6DOF(obj, t, s)

            X = s(1:3);
            V = s(4:6);
            Q = s(7:10);
            W = s(11:13);

            % Check quaternion norm
            Q = normalizeVect(Q);

            % Coordinate systems

            % Rotation matrix from rocket coordinates to Earth coordinates
            rotationMatrix = quat2rotmat(Q);
            angleOfAttack = rot2anglemat(rotationMatrix);

            % Rocket principle frame vectors expressed in earth coordinates
            YA = rotationMatrix*[1,0,0]'; % Yaw axis
            PA = rotationMatrix*[0,1,0]'; % Pitch Axis
            RA = rotationMatrix*[0,0,1]'; % Roll Axis

            % Earth coordinate vectors expressed in earth's frame
            XE = [1, 0, 0]';
            YE = [0, 1, 0]';
            ZE = [0, 0, 1]';

            % Rocket Inertia
            [mass, massRate, centerOfMass, ~, inertiaLong, ~, inertiaRot, ~] = Mass_Properties(t,obj.Rocket,'NonLinear');
            %I = rotationMatrix'*diag([inertiaLong, inertiaLong, inertiaRot])*rotationMatrix; % Inertia TODO: inertiaRot in Mass_Properties
            
            % Inertia using the given I_rocket and the motor
            % Compute I_motor (approximate by a cylinder)
            motorInertia = inertia_fill_cylinder(mass, ...
                obj.Rocket.motor_length, obj.Rocket.motor_dia / 2);
            % Total inertia
            %I = inertial_matrix(obj.Rocket,  centerOfMass, t);
            %disp(I)
            I = obj.Rocket.emptyInertia + motorInertia;
            %disp(I)
            %disp("==============================")
            I = rotationMatrix' * I * rotationMatrix; % Transfert to earth coordinates

            % Temporal derivative of inertial matrix
            inertiaRate = inertia_fill_cylinder(massRate, obj.Rocket.motor_length, ...
                obj.Rocket.motor_dia / 2); % Inertial matrix time derivative
            inertiaRate = rotationMatrix' * inertiaRate * rotationMatrix; % Transfert to earth coordinates

            % Environment
            g = 9.81;               % Gravity [m/s2] 
            [~, a, ~, rho, kinematicViscosity] = atmosphere(X(3)+obj.Environment.Start_Altitude,...
                obj.Environment); % Atmosphere information 

            % Force estimations 

            % Thrust
            % Oriented along roll axis of rocket frame, expressed in earth coordinates. 
            T = Thrust(t,obj.Rocket)*RA; % (TODO: Allow for thrust vectoring -> error)

            % Gravity
            G = -g*mass*ZE;

            % Aerodynamic corrective forces
            % Compute center of mass angleOfAttack of attack
            centerOfMassVelocity = V -...
                     ... % Wind as computed by windmodel
                windModel(t, obj.Environment.Turb_I,obj.Environment.V_inf*obj.Environment.V_dir,...
                obj.Environment.Turb_model,X(3)); 

            centerOfMassSpeed = norm(centerOfMassVelocity);
            centerOfMassAngleOfAttack = atan2(norm(cross(RA, centerOfMassVelocity)), dot(RA, centerOfMassVelocity));

            % Mach number
            Mach = centerOfMassSpeed/a;
            % Normal lift coefficient and center of pressure
            [CNa, xcp,CNa_bar,CP_bar] = normalLift(obj.Rocket, centerOfMassAngleOfAttack, 1.1,...
                Mach, angleOfAttack(3), 1);
            
            
            % Stability margin
            margin = (xcp- centerOfMass);

            % Compute Rocket angleOfAttack of attack
            Wnorm = W/norm(W);
            if(isnan(Wnorm))
                Wnorm  = zeros(3,1);
            end
            relativeVelocity = centerOfMassVelocity + margin*sin(acos(dot(RA,Wnorm)))*(cross(RA, W));
            relativeSpeed = norm(relativeVelocity);
            normalizedVelocity = normalizeVect(relativeVelocity);

            % Angle of attack 
            velocityCrossProduct = cross(RA, normalizedVelocity);
            normalizedCrossProduct = normalizeVect(velocityCrossProduct);
            alpha = atan2(norm(cross(RA, normalizedVelocity)), dot(RA, normalizedVelocity));
            delta = atan2(norm(cross(RA, ZE)), dot(RA, ZE));
            
            % wind coordinate transformation
%             if(abs(alpha)<1e-3)
%                 RW = RA;
%             elseif(abs(alpha-pi)<1e-3)
%                 RW = -RA;
%             else
%                 Cw = quat2rotmat([normalizedCrossProduct*sin(alpha/2); cos(alpha/2)]);
%                 RW = rotationMatrix*Cw*[0;0;1];
%             end

            % normal force
            normalAxis = cross(RA, velocityCrossProduct); % normal axis
            if norm(normalAxis) == 0
                normalForce = [0, 0, 0]'; 
            else
                normalForce = 0.5*rho*obj.Rocket.maxCrossSectionArea*CNa*alpha*relativeSpeed^2*normalAxis/norm(normalAxis);
            end
            % Drag
            % Drag coefficient
            dragCoefficient = drag(obj.Rocket, alpha, relativeSpeed, kinematicViscosity, a)*obj.Rocket.dragCoefficientFactor; 
            if(t>obj.Rocket.Burn_Time)
              dragCoefficient = dragCoefficient + drag_shuriken(obj.Rocket, obj.Rocket.airbrakeAngle, alpha, relativeSpeed, kinematicViscosity); 
            end
            % Drag force
            D = -0.5*rho*obj.Rocket.maxCrossSectionArea*dragCoefficient*relativeSpeed^2*normalizedVelocity ;

            % Total forces
            totalForce = ...
                T*obj.Rocket.motorThrustFactor +...  ;% Thrust
                G +...  ;% gravity
                normalForce +... ;% normal force
                D      ; % drag force

            % Moment estimation

            %Aerodynamic corrective moment
            normalMoment = norm(normalForce)*margin*normalizedCrossProduct ;

            % Aerodynamic damping moment
            pitchAngularVelocity = W - dot(W,RA)*RA; % extract pitch and yaw angular velocity
            pitchDampingCoefficient = pitchDampingMoment(obj.Rocket, rho, CNa_bar, CP_bar, ...
                massRate,  centerOfMass, norm(pitchAngularVelocity) , relativeSpeed); 
            
            dampingMoment = -0.5*rho*pitchDampingCoefficient*obj.Rocket.maxCrossSectionArea*relativeSpeed^2*normalizeVect(pitchAngularVelocity);
            
            totalMoment = ...
                normalMoment...  ; % aerodynamic corrective moment
               + dampingMoment; % aerodynamic damping moment

            % State derivatives

            % Translational dynamics
            X_dot = V;
            V_dot = 1/mass*(totalForce);

            % Rotational dynamics
            Q_dot = quat_evolve(Q, W);
            
            %W_dot = pinv(I)*totalMoment;
            %W_dot = mldivide(I,totalMoment); % (TODO: Add inertia variation with time)
            W_dot = mldivide(I, totalMoment - inertiaRate*angleOfAttack');

            % Return derivative vector
            S_dot = [X_dot;V_dot;Q_dot;W_dot];
            
            % cache auxiliary result data
            obj.tmpMargin = margin/obj.Rocket.maxDiameter;
            obj.tmpAlpha = alpha;
            obj.tmpCnAlpha = CNa;
            obj.tmpXcp = xcp;
            obj.tmpDragCoefficient = dragCoefficient;
            obj.tmpMass = mass;
            obj.tmpCenterOfMass =  centerOfMass;
            obj.tmpInertiaLong = inertiaLong;
            obj.tmpInertiaRot = inertiaRot;
            obj.tmpDelta = delta;
        end
        
        % --------------------------- 
        % 3DOF Parachute descent Equations
        % ---------------------------
        
        function dsdt = Dynamics_Parachute_3DOF(obj, t,s, Rocket, Environment, mass, Main)

            X = s(1:3);
            V = s(4:6);

            % Atmospheric Data
            [~, ~, ~, rho] = atmosphere(X(3)+Environment.Start_Altitude, Environment); % Atmosphere [K,m/s,Pa,kg/m3]

            % Aerodynamic force
            relativeVelocity = -V + ...
                 ... % Wind as computed by windmodel
                windModel(t, Environment.Turb_I,Environment.V_inf*Environment.V_dir,...
                Environment.Turb_model,X(3));

            if Main
                SCD = Rocket.mainParachuteDragArea;
            elseif Main == 0
                SCD = Rocket.drogueParachuteDragArea;
            end
            D = 0.5*rho*SCD*norm(relativeVelocity)*relativeVelocity;

            % Gravity force
            g = 9.81*[0;0;-1];
            G = g*mass;

            dXdt = V;
            dVdt = (D+G)/mass;

            dsdt = [dXdt; dVdt];
        end
        
        % --------------------------- 
        % 3DOF Crash descent Equations
        % ---------------------------
        
        function S_dot = Dynamics_3DOF(obj, t, s, Rocket, Environment)

            X = s(1:3);
            V = s(4:6);

            % Earth coordinate vectors expressed in earth's frame
            XE = [1, 0, 0]';
            YE = [0, 1, 0]';
            ZE = [0, 0, 1]';

            % atmosphere
            [~, a, ~, rho, kinematicViscosity] = atmosphere(X(3)+Environment.Start_Altitude, Environment);

            % mass
            mass = Rocket.emptyMass;

            relativeVelocity = V -...
                 ... % Wind as computed by windmodel
                windModel(t, Environment.Turb_I,Environment.V_inf*Environment.V_dir,...
                Environment.Turb_model,X(3));

            % gravity
            % Gravity
            G = -9.81*mass*ZE;
            % Drag
            % Drag coefficient
            dragCoefficient = drag(Rocket, 0, norm(relativeVelocity), kinematicViscosity, a); % (TODO: make air-viscosity adaptable to temperature)
            % Drag force
            D = -0.5*rho*Rocket.maxCrossSectionArea*dragCoefficient*relativeVelocity*norm(relativeVelocity); 
            % Translational dynamics
            X_dot = V;
            V_dot = 1/mass*(D + G);

            S_dot = [X_dot; V_dot];

        end
        
        % --------------------------- 
        % 3DOF Nosecone Crash descent Equations
        % ---------------------------
        
        function S_dot = Nose_Dynamics_3DOF(obj, t, s, Rocket, Environment)

            X = s(1:3);
            V = s(4:6);

            % Earth coordinate vectors expressed in earth's frame
            XE = [1, 0, 0]';
            YE = [0, 1, 0]';
            ZE = [0, 0, 1]';

            % atmosphere
            [~, a, ~, rho, kinematicViscosity] = atmosphere(X(3)+Environment.Start_Altitude, Environment);

            % mass
            mass = Rocket.emptyMass;

            relativeVelocity = V -...
                 ... % Wind as computed by windmodel
                windModel(t, Environment.Turb_I,Environment.V_inf*Environment.V_dir,...
                Environment.Turb_model,X(3));

            % gravity
            % Gravity
            G = -9.81*mass*ZE;
            % Drag
            % Drag coefficient
            dragCoefficient = Nose_drag(Rocket, 0, norm(relativeVelocity), kinematicViscosity, a); % (TODO: make air-viscosity adaptable to temperature)
            % Drag force
            D = -0.5*rho*Rocket.maxCrossSectionArea*dragCoefficient*relativeVelocity*norm(relativeVelocity); 

            % Translational dynamics
            X_dot = V;
            V_dot = 1/mass*(D + G);

            S_dot = [X_dot; V_dot];

        end
        
        % --------------------------- 
        % 6DOF Nosecone Crash descent Equations
        % ---------------------------
        
        function S_dot = Nose_Dynamics_6DOF(obj, t, s)

            X = s(1:3);
            V = s(4:6);
            Q = s(7:10);
            W = s(11:13);

            % Check quaternion norm
            Q = normalizeVect(Q);

            % Coordinate systems

            % Rotation matrix from rocket coordinates to Earth coordinates
            rotationMatrix = quat2rotmat(Q);
            angleOfAttack = rot2anglemat(rotationMatrix);

            % Rocket principle frame vectors expressed in earth coordinates
            YA = rotationMatrix*[1,0,0]'; % Yaw axis
            PA = rotationMatrix*[0,1,0]'; % Pitch Axis
            RA = rotationMatrix*[0,0,1]'; % Roll Axis

            % Earth coordinate vectors expressed in earth's frame
            XE = [1, 0, 0]';
            YE = [0, 1, 0]';
            ZE = [0, 0, 1]';

            % Rocket Inertia
            [mass,massRate, centerOfMass,~,inertiaLong,~,inertiaRot,~] = Mass_Properties(t,obj.Rocket,'NonLinear');
            %I = rotationMatrix'*diag([inertiaLong, inertiaLong, inertiaRot])*rotationMatrix; % Inertia TODO: inertiaRot in Mass_Properties

            % Inertia using the given I_rocket and the motor
            % Compute I_motor (approximate by a cylinder)
            motorInertia = inertia_fill_cylinder(mass, ...
                obj.Rocket.motor_length, obj.Rocket.motor_dia / 2);
            % Total inertia
            I = obj.Rocket.emptyInertia + motorInertia;
            I = rotationMatrix' * I * rotationMatrix; % Transfert to earth coordinates

            % Temporal derivative of inertial matrix
            inertiaRate = inertia_fill_cylinder(massRate, obj.Rocket.motor_length, ...
                obj.Rocket.motor_dia / 2); % Inertial matrix time derivative
            inertiaRate = rotationMatrix' * inertiaRate * rotationMatrix; % Transfert to earth coordinates

            % Environment
            g = 9.81;               % Gravity [m/s2]
            [~, a, ~, rho, kinematicViscosity] = atmosphere(X(3)+obj.Environment.Start_Altitude,...
                obj.Environment); % Atmosphere information 

            % Force estimations 

            % Thrust
            % Oriented along roll axis of rocket frame, expressed in earth coordinates. 
            T = Thrust(t,obj.Rocket)*RA; % (TODO: Allow for thrust vectoring -> error)

            % Gravity
            G = -g*mass*ZE;

            % Aerodynamic corrective forces
            % Compute center of mass angleOfAttack of attack
            centerOfMassVelocity = V -...
                     ... % Wind as computed by windmodel
                windModel(t, obj.Environment.Turb_I,obj.Environment.V_inf*obj.Environment.V_dir,...
                obj.Environment.Turb_model,X(3)); 

            centerOfMassSpeed = norm(centerOfMassVelocity);
            centerOfMassAngleOfAttack = atan2(norm(cross(RA, centerOfMassVelocity)), dot(RA, centerOfMassVelocity));

            % Mach number
            Mach = centerOfMassSpeed/a;
            % Normal lift coefficient and center of pressure
            [CNa, xcp,CNa_bar,CP_bar] = normalLift(obj.Rocket, centerOfMassAngleOfAttack, 1.1,...
                Mach, angleOfAttack(3), 1);
            % Stability margin
            margin = (xcp- centerOfMass);

            % Compute Rocket angleOfAttack of attack
            Wnorm = W/norm(W);
            if(isnan(Wnorm))
                Wnorm  = zeros(3,1);
            end
            relativeVelocity = centerOfMassVelocity + margin*sin(acos(dot(RA,Wnorm)))*(cross(RA, W));
            relativeSpeed = norm(relativeVelocity);
            normalizedVelocity = normalizeVect(relativeVelocity);

            % Angle of attack 
            velocityCrossProduct = cross(RA, normalizedVelocity);
            normalizedCrossProduct = normalizeVect(velocityCrossProduct);
            alpha = atan2(norm(cross(RA, normalizedVelocity)), dot(RA, normalizedVelocity));
            delta = atan2(norm(cross(RA, ZE)), dot(RA, ZE));

            % wind coordinate transformation
%             if(abs(alpha)<1e-3)
%                 RW = RA;
%             elseif(abs(alpha-pi)<1e-3)
%                 RW = -RA;
%             else
%                 Cw = quat2rotmat([normalizedCrossProduct*sin(alpha/2); cos(alpha/2)]);
%                 RW = rotationMatrix*Cw*[0;0;1];
%             end

            % normal force
            normalAxis = cross(RA, velocityCrossProduct); % normal axis
            if norm(normalAxis) == 0
                normalForce = [0, 0, 0]'; 
            else
                normalForce = 0.5*rho*obj.Rocket.maxCrossSectionArea*CNa*alpha*relativeSpeed^2*normalAxis/norm(normalAxis);
            end

            % Drag
            % Drag coefficient
            dragCoefficient = Nose_drag(obj.Rocket, alpha, relativeSpeed, kinematicViscosity, a)*obj.Rocket.dragCoefficientFactor; 
            if(t>obj.Rocket.Burn_Time)
              dragCoefficient = dragCoefficient + drag_shuriken(obj.Rocket, obj.Rocket.airbrakeAngle, alpha, relativeSpeed, kinematicViscosity); 
            end
            % Drag force
            D = -0.5*rho*obj.Rocket.maxCrossSectionArea*dragCoefficient*relativeSpeed^2*normalizedVelocity;

            % Total forces
            totalForce = ...
                T*obj.Rocket.motorThrustFactor +...  ;% Thrust
                G +...  ;% gravity
                normalForce +... ;% normal force
                D      ; % drag force

            % Moment estimation

            %Aerodynamic corrective moment
            normalMoment = norm(normalForce)*margin*normalizedCrossProduct;

            % Aerodynamic damping moment
            pitchAngularVelocity = W - dot(W,RA)*RA; % extract pitch and yaw angular velocity
            pitchDampingCoefficient = pitchDampingMoment(obj.Rocket, rho, CNa_bar, CP_bar, ...
                massRate,  centerOfMass, norm(pitchAngularVelocity) , relativeSpeed); 
            dampingMoment = -0.5*rho*pitchDampingCoefficient*obj.Rocket.maxCrossSectionArea*relativeSpeed^2*normalizeVect(pitchAngularVelocity);

            totalMoment = ...
                normalMoment...  ; % aerodynamic corrective moment
               + dampingMoment ; % aerodynamic damping moment

            % State derivatives

            % Translational dynamics
            X_dot = V;
            V_dot = 1/mass*(totalForce);

            % Rotational dynamics
            Q_dot = quat_evolve(Q, W);
            %W_dot = mldivide(I,totalMoment); % (TODO: Add inertia variation with time)
            W_dot = mldivide(I, totalMoment - inertiaRate*angleOfAttack');

            % Return derivative vector
            S_dot = [X_dot;V_dot;Q_dot;W_dot];
            
            % cache auxiliary result data
            obj.tmpNoseAlpha = alpha;
            obj.tmpNoseDelta = delta;
        end
        
        % --------------------------- 
        % 3DOF Payload descent Equations
        % ---------------------------
        
        function S_dot = Payload_Dynamics_3DOF(obj, t, s, Rocket, Environment)

            X = s(1:3);
            V = s(4:6);

            % Earth coordinate vectors expressed in earth's frame
            XE = [1, 0, 0]';
            YE = [0, 1, 0]';
            ZE = [0, 0, 1]';

            % atmosphere
            [~, a, ~, rho, kinematicViscosity] = atmosphere(X(3)+Environment.Start_Altitude, Environment);

            % mass
            mass = Rocket.payloadMass;

            relativeVelocity = V -...
                 ... % Wind as computed by windmodel
                windModel(t, Environment.Turb_I,Environment.V_inf*Environment.V_dir,...
                Environment.Turb_model);

            % gravity
            % Gravity
            G = -9.81*mass*ZE;
            % Drag
            % Drag coefficient
            SCD = 2.56e-2; 
            % Drag force
            D = -0.5*rho*SCD*relativeVelocity*norm(relativeVelocity); 

            % Translational dynamics
            X_dot = V;
            V_dot = 1/mass*(D + G);

            S_dot = [X_dot; V_dot];

        end
        
   end     
   
% -------------------------------------------------------------------------  
% Runnable methods
% -------------------------------------------------------------------------           
    methods(Access = public)
        
        % --------------------------- 
        % Rail Simulation
        % ---------------------------
        function [T1, S1] = RailSim(obj)
            
           % Initial Conditions
            X0 = [0,0]'; % positioned at 0 height and 0 velocity

            % time span 
            tspan = [0, 5];

            % options
            Option = odeset('Events', @(t,x) RailEvent(t,x,obj.Environment));

            % integration
            [T1,S1] = ode45(@(t,x) obj.Dynamics_Rail_1DOF(t,x),tspan,X0, Option); 
            
        end
        
        % --------------------------- 
        % Flight Simulation
        % ---------------------------
        function [T2, S2, T2E, S2E, I2E] = FlightSim(obj, tspan, arg2, arg3, arg4, arg5)
            
            if (nargin == 3)
                % Compute initial conditions based on rail output values
                V = arg2;
                
                % Rail vector
                railRotationMatrix = rotmat(obj.Environment.Rail_Azimuth, 3)*...
                    rotmat(obj.Environment.Rail_Angle, 2)*...
                    rotmat(obj.Environment.Rail_Azimuth, 3)';
                railVector = railRotationMatrix*[0;0;1];

                % Initial Conditions
                X0 = railVector*obj.Environment.Rail_Length; % spatial position of centerOfMass
                V0 = railVector*V; % Initial velocity of centerOfMass
                Q0 = rot2quat(railRotationMatrix'); % Initial attitude
                W0 = [0;0;0]; % Initial angular rotation in rocket principle coordinates
                S0 = [X0; V0; Q0; W0];
            elseif (nargin == 6)
                % Set initial conditions based on the exact initial value
                % of the state vector.
                X0 = arg2;
                V0 = arg3;
                Q0 = arg4;
                W0 = arg5;
                S0 = [X0; V0; Q0; W0];
            else
               error('ERROR: In Flight Simulator, function accepts either 3 or 6 arguments.') 
            end

            % options
            Option = odeset('Events', @ApogeeEvent, 'RelTol', 1e-6, 'AbsTol', 1e-6,...
                            'OutputFcn', @(T,S,flag) obj.FlightOutputFunc(T,S,flag),...
                            'Refine', 1);

            % integration
            [T2,S2, T2E, S2E, I2E] = ode45(@(t,s) obj.Dynamics_6DOF(t,s),tspan,S0, Option);
            
        end
        
        
        % --------------------------- 
        % Drogue Parachute Simulation
        % ---------------------------
        function [T3, S3, T3E, S3E, I3E] = DrogueParaSim(obj, T0, X0, V0)
            
            % initial conditions
            S0 = [X0; V0];

            % empty mass
            mass = obj.Rocket.emptyMass - obj.Rocket.payloadMass;

            % time span
            tspan = [T0, 5000];

            % options 
            Option = odeset('Events', @(T,X) MainEvent(T,X,obj.Rocket));

            % integration
            [T3,S3, T3E, S3E, I3E] = ode45(@(t,s) obj.Dynamics_Parachute_3DOF(t,s,obj.Rocket,obj.Environment, mass, 0),tspan,S0, Option);
        
        end
        
        % --------------------------- 
        % Main Parachute Simulation
        % ---------------------------
        function [T4, S4, T4E, S4E, I4E] = MainParaSim(obj, T0, X0, V0)
            
            % initial conditions
            S0 = [X0; V0];

            % empty mass
            mass = obj.Rocket.emptyMass - obj.Rocket.payloadMass;

            % time span
            tspan = [T0, 5000];

            % options 
            Option = odeset('Events',@(T,X) CrashEvent(T,X,obj.Environment));

            % integration
            [T4, S4, T4E, S4E, I4E] = ode45(@(t,s) obj.Dynamics_Parachute_3DOF(t,s,obj.Rocket,obj.Environment, mass, 1),tspan,S0, Option);
            
        end
        
        % --------------------------- 
        % Crash Simulation
        % ---------------------------
        function [T5, S5, T5E, S5E, I5E] = CrashSim(obj, T0, X0, V0)
            
            % Initial Conditions
            S0 = [X0; V0];

            % time span
            tspan = [T0, 100];

            % options
            Option = odeset('Events',@(T,X) CrashEvent(T,X,obj.Environment));

            % integration
            [T5,S5, T5E, S5E, I5E] = ode45(@(t,s) obj.Dynamics_3DOF(t,s,obj.Rocket,obj.Environment),tspan,S0, Option);

        end
        
        % --------------------------- 
        % Nosecone Crash Simulation 3DOF
        % ---------------------------
        function [T6, S6, T6E, S6E, I6E] = Nose_CrashSim_3DOF(obj, T0, X0, V0)
            
            % Initial Conditions
            S0 = [X0; V0];

            % time span
            tspan = [T0, 1000];

            % options
            Option = odeset('Events',@(T,X) CrashEvent(T,X,obj.Environment));

            % integration
            [T6,S6, T6E, S6E, I6E] = ode45(@(t,s) obj.Nose_Dynamics_3DOF(t,s,obj.Rocket,obj.Environment),tspan,S0, Option);

        end
        
        % --------------------------- 
        % Nosecone Crash Simulation 6DOF
        % ---------------------------
        function [T6, S6, T6E, S6E, I6E] = Nose_CrashSim_6DOF(obj, tspan, arg2, arg3, arg4, arg5)
            
            if (nargin == 6)
                % Set initial conditions based on the exact initial value
                % of the state vector.
                X0 = arg2;
                V0 = arg3;
                Q0 = arg4;
                W0 = arg5;
                S0 = [X0; V0; Q0; W0];
            else
               error('ERROR: In Flight Simulator, function accepts either 3 or 6 arguments.') 
            end

            % options
            Option = odeset('Events', @(T,X) CrashEvent(T,X,obj.Environment),...
                            'OutputFcn', @(T,S,flag) obj.CrashOutputFunc(T,S,flag),...
                            'Refine', 1);

            % integration
            [T6,S6, T6E, S6E, I6E] = ode45(@(t,s) obj.Nose_Dynamics_6DOF(t,s),tspan,S0, Option);
            
        end
        
        % --------------------------- 
        % Payload Impact Simulation
        % ---------------------------
        function [T7, S7, T7E, S7E, I7E] = PayloadCrashSim(obj, T0, X0, V0)
            
            % Initial Conditions
            S0 = [X0; V0];

            % time span
            tspan = [T0, 1000];

            % options
            Option = odeset('Events', @(T,X) CrashEvent(T,X,obj.Environment));

            % integration
            [T7,S7, T7E, S7E, I7E] = ode45(@(t,s) obj.Payload_Dynamics_3DOF(t,s,obj.Rocket,obj.Environment),tspan,S0, Option);

        end
    end
    
% -------------------------------------------------------------------------  
% Private methods
% -------------------------------------------------------------------------  
methods(Access = private)
    function status = FlightOutputFunc(obj, T,S,flag)

        % keep simulation running
        status = 0;

        if isempty(flag) || (strcmp(flag, 'init') && obj.firstSimFlag)

            obj.firstSimFlag = 0;
            
            if obj.SimOutput.Margin
                obj.SimAuxResults.margin = [obj.SimAuxResults.margin, obj.tmpMargin];
            end 
            if obj.SimOutput.Alpha
                obj.SimAuxResults.alpha = [obj.SimAuxResults.alpha, obj.tmpAlpha];
            end 
            if obj.SimOutput.Cn_alpha
                obj.SimAuxResults.cnAlpha = [obj.SimAuxResults.cnAlpha, obj.tmpCnAlpha];
            end 
            if obj.SimOutput.Xcp
                obj.SimAuxResults.xcp = [obj.SimAuxResults.xcp, obj.tmpXcp];
            end 
            if obj.SimOutput.Cd
                obj.SimAuxResults.dragCoefficient = [obj.SimAuxResults.dragCoefficient, obj.tmpDragCoefficient];
            end 
            if obj.SimOutput.Mass
                obj.SimAuxResults.mass = [obj.SimAuxResults.mass, obj.tmpMass];
            end 
            if obj.SimOutput.CM
                obj.SimAuxResults.centerOfMass = [obj.SimAuxResults.centerOfMass, obj.tmpCenterOfMass];
            end 
            if obj.SimOutput.Il
                obj.SimAuxResults.inertiaLong = [obj.SimAuxResults.inertiaLong, obj.tmpInertiaLong];
            end 
            if obj.SimOutput.Ir
                obj.SimAuxResults.inertiaRot = [obj.SimAuxResults.inertiaRot, obj.tmpInertiaRot];
            end
            if obj.SimOutput.Delta
                obj.SimAuxResults.delta = [obj.SimAuxResults.delta, obj.tmpDelta];
            end
            
            if obj.SimOutput.Nose_Alpha
                obj.SimAuxResults.noseAlpha = [obj.SimAuxResults.noseAlpha, obj.tmpNoseAlpha];
            end
            if obj.SimOutput.Nose_Delta
                obj.SimAuxResults.noseDelta = [obj.SimAuxResults.noseDelta, obj.tmpNoseDelta];
            end
            
        end
        
    end
    
    function status = CrashOutputFunc(obj, T,S,flag)

        % keep simulation running
        status = 0;

        if isempty(flag) || (strcmp(flag, 'init') && obj.firstSimFlag)

            obj.firstSimFlag = 0;
            if obj.SimOutput.Nose_Alpha
                obj.SimAuxResults.noseAlpha = [obj.SimAuxResults.noseAlpha, obj.tmpNoseAlpha];
            end
            if obj.SimOutput.Nose_Delta
                obj.SimAuxResults.noseDelta = [obj.SimAuxResults.noseDelta, obj.tmpNoseDelta];
            end
            
        end
        
    end
end
end