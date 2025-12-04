classdef Simulator3DDragOpenRocket < handle
    
% -------------------------------------------------------------------------  
% Class properties
% -------------------------------------------------------------------------
   properties(Access = public)
      simAuxResults; 
   end

   properties(Access = public)
      rocket
      environment
      drag
      interpType
      simOutput  
   end
   
   properties(Access = private)
      firstSimFlag = 1;
      tmpMargin
      tmpAlpha
      tmpCnAlpha
      tmpXcp
      tmpCd
      tmpMass
      tmpCm
      tmpIl
      tmpIr
      tmpDelta
      
      tmpNoseAlpha
      tmpNoseDelta
   end
   
% -------------------------------------------------------------------------  
% Constructor  
% -------------------------------------------------------------------------   
   methods
       
       function obj = Simulator3DDragOpenRocket(rocket, environment, drag, interpType, simOutput)
           if nargin == 0
               % TODO: Put default values or send warning message
           elseif nargin == 5
               obj.rocket = rocket;
               obj.environment = environment;
               
               %Extract drag coefficients computed by OR
               dragOR = readtable(drag,'Format','%s%s%s%s');

               indexStart = find(contains(dragOR{:,1},'# Event LIFTOFF'));
               indexEnd = find(contains(dragOR{:,1},'# Event APOGEE'));

               dragOR = dragOR{indexStart:indexEnd,:};

               removeIndex = find(contains(dragOR(:,1),"Event"));
               dragOR(removeIndex,:) = [];
               dragOR = str2double(dragOR);
               obj.drag = dragOR;
               
               obj.interpType = interpType;
               obj.simOutput = simOutput;
           else
               error(['ERROR: In Simulator3DDragOpenRocket constructor, either no arguments '...
                   'or 5 arguments can be given. You gave ' num2str(nargin) '.']);
           end

           % Initialise Auxiliary results structure
           obj.simAuxResults.margin = [];
           obj.simAuxResults.alpha = [];
           obj.simAuxResults.cnAlpha = [];
           obj.simAuxResults.xcp = [];
           obj.simAuxResults.cd = [];
           obj.simAuxResults.mass = [];
           obj.simAuxResults.cm = [];
           obj.simAuxResults.il = [];
           obj.simAuxResults.ir = [];
           obj.simAuxResults.delta = [];
           
           obj.simAuxResults.noseAlpha = [];
           obj.simAuxResults.noseDelta = [];
       end
       
   end
     
% -------------------------------------------------------------------------  
% Dynamic Computation methods & Output functions
% -------------------------------------------------------------------------      
   methods(Access = protected)
       
        % --------------------------- 
        % Rail equations 
        % ---------------------------
    
        function sDot = Dynamics_Rail_1DOF(obj, t, s)

            x = s(1); % position
            v = s(2); % speed

            % Rocket Inertia
            [mass,massRate] = Mass_Non_Lin(t,obj.rocket); % mass

            % Environment
            g = 9.81;               % Gravity [m/s2] 
            [~, a, ~, rho, kinematicViscosity] = atmosphere(x*sin(obj.environment.railAngle),obj.environment); % Atmosphere information (TODO: Include effect of humidity and departure altitude)

            % Force estimation

            % gravity
            gravityForce = -g*cos(obj.environment.railAngle)*mass;

            % Thrust 
            thrust = Thrust(t,obj.rocket); % (TODO: Allow for thrust vectoring -> error)

            % drag
            dragCoefficient = drag(obj.drag, obj.interpType, t, x, v); % (TODO: make air-viscosity adaptable to temperature)
            dragForce = -0.5*rho*obj.rocket.Sm*dragCoefficient*v^2; % (TODO: define drag in wind coordinate system)

            totalForce = gravityForce + thrust*obj.rocket.motor_fac + dragForce;

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

            sDot = [x_dot; v_dot];
        end
       
        % --------------------------- 
        % 6DOF Flight Equations
        % ---------------------------
        
        function sDot = Dynamics_6DOF(obj, t, s)

            X = s(1:3);
            V = s(4:6);
            Q = s(7:10);
            W = s(11:13);

            % Check quaternion norm
            Q = normalizeVect(Q);

            % Coordinate systems

            % Rotation matrix from rocket coordinates to Earth coordinates
            rotationMatrix = quat2rotmat(Q);
            angle = rot2anglemat(rotationMatrix);

            % Rocket principle frame vectors expressed in earth coordinates
            YA = rotationMatrix*[1,0,0]'; % Yaw axis
            PA = rotationMatrix*[0,1,0]'; % Pitch Axis
            RA = rotationMatrix*[0,0,1]'; % Roll Axis

            % Earth coordinate vectors expressed in earth's frame
            XE = [1, 0, 0]';
            YE = [0, 1, 0]';
            ZE = [0, 0, 1]';

            % Rocket Inertia
            [mass,massRate,cm,~,iLong,~,iRot,~] = Mass_Properties(t,obj.rocket,'NonLinear');
            %I = rotationMatrix'*diag([iLong, iLong, iRot])*rotationMatrix; % Inertia TODO: iRot in Mass_Properties
            
            % Inertia using the given I_rocket and the motor
            % Compute I_motor (approximate by a cylinder)
            motorInertia = inertia_fill_cylinder(mass, ...
                obj.rocket.motor_length, obj.rocket.motor_dia / 2);
            % Total inertia
            %I = inertial_matrix(obj.rocket, cm, t);
            %disp(I)
            I = obj.rocket.rocket_inertia + motorInertia;
            %disp(I)
            %disp("==============================")
            I = rotationMatrix' * I * rotationMatrix; % Transfert to earth coordinates

            % Temporal derivative of inertial matrix
            inertiaRate = inertia_fill_cylinder(massRate, obj.rocket.motor_length, ...
                obj.rocket.motor_dia / 2); % Inertial matrix time derivative
            inertiaRate = rotationMatrix' * inertiaRate * rotationMatrix; % Transfert to earth coordinates

            % Environment
            g = 9.81;               % Gravity [m/s2] 
            [~, a, ~, rho, nu] = atmosphere(X(3)+obj.environment.startAltitude,...
                obj.environment); % Atmosphere information 

            % Force estimations 

            % Thrust
            % Oriented along roll axis of rocket frame, expressed in earth coordinates. 
            thrust = Thrust(t,obj.rocket)*RA; % (TODO: Allow for thrust vectoring -> error)

            % Gravity
            gravityForce = -g*mass*ZE;

            % Aerodynamic corrective forces
            % Compute center of mass angle of attack
            Vcm = V -...
                     ... % Wind as computed by windmodel
                windModel(t, obj.environment.turbI,obj.environment.V_inf*obj.environment.V_dir,...
                obj.environment.turbModel,X(3)); 

            VcmMag = norm(Vcm);
            alphaCm = atan2(norm(cross(RA, Vcm)), dot(RA, Vcm));

            % Mach number
            Mach = VcmMag/a;
            % Normal lift coefficient and center of pressure
            [CNa, xcp,CNa_bar,CP_bar] = normalLift(obj.rocket, alphaCm, 1.1,...
                Mach, angle(3), 1);
            
            
            % Stability margin
            margin = (xcp-cm);

            % Compute Rocket angle of attack
            Wnorm = W/norm(W);
            if(isnan(Wnorm))
                Wnorm  = zeros(3,1);
            end
            Vrel = Vcm + margin*sin(acos(dot(RA,Wnorm)))*(cross(RA, W));
            Vmag = norm(Vrel);
            Vnorm = normalizeVect(Vrel);

            % Angle of attack 
            Vcross = cross(RA, Vnorm);
            VcrossNorm = normalizeVect(Vcross);
            alpha = atan2(norm(cross(RA, Vnorm)), dot(RA, Vnorm));
            delta = atan2(norm(cross(RA, ZE)), dot(RA, ZE));
            
            % wind coordinate transformation
%             if(abs(alpha)<1e-3)
%                 RW = RA;
%             elseif(abs(alpha-pi)<1e-3)
%                 RW = -RA;
%             else
%                 Cw = quat2rotmat([VcrossNorm*sin(alpha/2); cos(alpha/2)]);
%                 RW = rotationMatrix*Cw*[0;0;1];
%             end

            % normal force
            NA = cross(RA, Vcross); % normal axis
            if norm(NA) == 0
                normalForce = [0, 0, 0]'; 
            else
                normalForce = 0.5*rho*obj.rocket.Sm*CNa*alpha*Vmag^2*NA/norm(NA);
            end
            % Drag
            % Drag coefficient
            dragCoefficient = drag(obj.drag, obj.interpType, t, X(3), V(3))*obj.rocket.CD_fac; 
            if(t>obj.rocket.burnTime)
              dragCoefficient = dragCoefficient + drag_shuriken(obj.rocket, obj.rocket.ab_phi, alpha, Vmag, nu); 
            end
            % Drag force
            dragForce = -0.5*rho*obj.rocket.Sm*dragCoefficient*Vmag^2*Vnorm ;

            % Total forces
            totalForce = ...
                thrust*obj.rocket.motor_fac +...  ;% Thrust
                gravityForce +...  ;% gravity
                normalForce +... ;% normal force
                dragForce      ; % drag force

            % Moment estimation

            %Aerodynamic corrective moment
            normalMoment = norm(normalForce)*margin*VcrossNorm ;

            % Aerodynamic damping moment
            W_pitch = W - dot(W,RA)*RA; % extract pitch and yaw angular velocity
            CDM = pitchDampingMoment(obj.rocket, rho, CNa_bar, CP_bar, ...
                massRate, cm, norm(W_pitch) , Vmag); 
            
            dampingMoment = -0.5*rho*CDM*obj.rocket.Sm*Vmag^2*normalizeVect(W_pitch);
            
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
            W_dot = mldivide(I, totalMoment - inertiaRate*angle');

            % Return derivative vector
            sDot = [X_dot;V_dot;Q_dot;W_dot];
            
            % cache auxiliary result data
            obj.tmpMargin = margin/obj.rocket.dm;
            obj.tmpAlpha = alpha;
            obj.tmpCnAlpha = CNa;
            obj.tmpXcp = xcp;
            obj.tmpCd = dragCoefficient;
            obj.tmpMass = mass;
            obj.tmpCm = cm;
            obj.tmpIl = iLong;
            obj.tmpIr = iRot;
            obj.tmpDelta = delta;
        end
        
        % --------------------------- 
        % 3DOF Parachute descent Equations
        % ---------------------------
        
        function dsdt = Dynamics_Parachute_3DOF(obj, t,s, rocket, environment, mass, Main)

            X = s(1:3);
            V = s(4:6);

            % Atmospheric Data
            [~, ~, ~, rho] = atmosphere(X(3)+environment.startAltitude, environment); % Atmosphere [K,m/s,Pa,kg/m3]

            % Aerodynamic force
            Vrel = -V + ...
                 ... % Wind as computed by windmodel
                windModel(t, environment.turbI,environment.V_inf*environment.V_dir,...
                environment.turbModel,X(3));

            if Main
                SCD = rocket.para_main_SCD;
            elseif Main == 0
                SCD = rocket.para_drogue_SCD;
            end
            dragForce = 0.5*rho*SCD*norm(Vrel)*Vrel;

            % Gravity force
            g = 9.81*[0;0;-1];
            gravityForce = g*mass;

            dXdt = V;
            dVdt = (dragForce+gravityForce)/mass;

            dsdt = [dXdt; dVdt];
        end
        
        % --------------------------- 
        % 3DOF Crash descent Equations
        % ---------------------------
        
        function sDot = Dynamics_3DOF(obj, t, s, rocket, environment)

            X = s(1:3);
            V = s(4:6);

            % Earth coordinate vectors expressed in earth's frame
            XE = [1, 0, 0]';
            YE = [0, 1, 0]';
            ZE = [0, 0, 1]';

            % atmosphere
            [~, a, ~, rho, nu] = atmosphere(X(3)+environment.startAltitude, environment);

            % mass
            mass = rocket.rocket_m;

            V_rel = V -...
                 ... % Wind as computed by windmodel
                windModel(t, environment.turbI,environment.V_inf*environment.V_dir,...
                environment.turbModel,X(3));

            % gravity
            % Gravity
            gravityForce = -9.81*mass*ZE;
            % Drag
            % Drag coefficient
            dragCoefficient = drag(obj.drag, obj.interpType, t, X(3), V(3)); % (TODO: make air-viscosity adaptable to temperature)
            % Drag force
            dragForce = -0.5*rho*rocket.Sm*dragCoefficient*V_rel*norm(V_rel); 
            % Translational dynamics
            X_dot = V;
            V_dot = 1/mass*(dragForce + gravityForce);

            sDot = [X_dot; V_dot];

        end
        
        % --------------------------- 
        % 3DOF Nosecone Crash descent Equations
        % ---------------------------
        
        function sDot = Nose_Dynamics_3DOF(obj, t, s, rocket, environment)

            X = s(1:3);
            V = s(4:6);

            % Earth coordinate vectors expressed in earth's frame
            XE = [1, 0, 0]';
            YE = [0, 1, 0]';
            ZE = [0, 0, 1]';

            % atmosphere
            [~, a, ~, rho, nu] = atmosphere(X(3)+environment.startAltitude, environment);

            % mass
            mass = rocket.rocket_m;

            V_rel = V -...
                 ... % Wind as computed by windmodel
                windModel(t, environment.turbI,environment.V_inf*environment.V_dir,...
                environment.turbModel,X(3));

            % gravity
            % Gravity
            gravityForce = -9.81*mass*ZE;
            % Drag
            % Drag coefficient
            dragCoefficient = Nose_drag(rocket, 0, norm(V_rel), nu, a); % (TODO: make air-viscosity adaptable to temperature)
            % Drag force
            dragForce = -0.5*rho*rocket.Sm*dragCoefficient*V_rel*norm(V_rel); 

            % Translational dynamics
            X_dot = V;
            V_dot = 1/mass*(dragForce + gravityForce);

            sDot = [X_dot; V_dot];

        end
        
        % --------------------------- 
        % 6DOF Nosecone Crash descent Equations
        % ---------------------------
        
        function sDot = Nose_Dynamics_6DOF(obj, t, s)

            X = s(1:3);
            V = s(4:6);
            Q = s(7:10);
            W = s(11:13);

            % Check quaternion norm
            Q = normalizeVect(Q);

            % Coordinate systems

            % Rotation matrix from rocket coordinates to Earth coordinates
            rotationMatrix = quat2rotmat(Q);
            angle = rot2anglemat(rotationMatrix);

            % Rocket principle frame vectors expressed in earth coordinates
            YA = rotationMatrix*[1,0,0]'; % Yaw axis
            PA = rotationMatrix*[0,1,0]'; % Pitch Axis
            RA = rotationMatrix*[0,0,1]'; % Roll Axis

            % Earth coordinate vectors expressed in earth's frame
            XE = [1, 0, 0]';
            YE = [0, 1, 0]';
            ZE = [0, 0, 1]';

            % Rocket Inertia
            [mass,massRate,cm,~,iLong,~,iRot,~] = Mass_Properties(t,obj.rocket,'NonLinear');
            %I = rotationMatrix'*diag([iLong, iLong, iRot])*rotationMatrix; % Inertia TODO: iRot in Mass_Properties

            % Inertia using the given I_rocket and the motor
            % Compute I_motor (approximate by a cylinder)
            motorInertia = inertia_fill_cylinder(mass, ...
                obj.rocket.motor_length, obj.rocket.motor_dia / 2);
            % Total inertia
            I = obj.rocket.rocket_inertia + motorInertia;
            I = rotationMatrix' * I * rotationMatrix; % Transfert to earth coordinates

            % Temporal derivative of inertial matrix
            inertiaRate = inertia_fill_cylinder(massRate, obj.rocket.motor_length, ...
                obj.rocket.motor_dia / 2); % Inertial matrix time derivative
            inertiaRate = rotationMatrix' * inertiaRate * rotationMatrix; % Transfert to earth coordinates

            % Environment
            g = 9.81;               % Gravity [m/s2]
            [~, a, ~, rho, nu] = atmosphere(X(3)+obj.environment.startAltitude,...
                obj.environment); % Atmosphere information 

            % Force estimations 

            % Thrust
            % Oriented along roll axis of rocket frame, expressed in earth coordinates. 
            thrust = Thrust(t,obj.rocket)*RA; % (TODO: Allow for thrust vectoring -> error)

            % Gravity
            gravityForce = -g*mass*ZE;

            % Aerodynamic corrective forces
            % Compute center of mass angle of attack
            Vcm = V -...
                     ... % Wind as computed by windmodel
                windModel(t, obj.environment.turbI,obj.environment.V_inf*obj.environment.V_dir,...
                obj.environment.turbModel,X(3)); 

            VcmMag = norm(Vcm);
            alphaCm = atan2(norm(cross(RA, Vcm)), dot(RA, Vcm));

            % Mach number
            Mach = VcmMag/a;
            % Normal lift coefficient and center of pressure
            [CNa, xcp,CNa_bar,CP_bar] = normalLift(obj.rocket, alphaCm, 1.1,...
                Mach, angle(3), 1);
            % Stability margin
            margin = (xcp-cm);

            % Compute Rocket angle of attack
            Wnorm = W/norm(W);
            if(isnan(Wnorm))
                Wnorm  = zeros(3,1);
            end
            Vrel = Vcm + margin*sin(acos(dot(RA,Wnorm)))*(cross(RA, W));
            Vmag = norm(Vrel);
            Vnorm = normalizeVect(Vrel);

            % Angle of attack 
            Vcross = cross(RA, Vnorm);
            VcrossNorm = normalizeVect(Vcross);
            alpha = atan2(norm(cross(RA, Vnorm)), dot(RA, Vnorm));
            delta = atan2(norm(cross(RA, ZE)), dot(RA, ZE));

            % wind coordinate transformation
%             if(abs(alpha)<1e-3)
%                 RW = RA;
%             elseif(abs(alpha-pi)<1e-3)
%                 RW = -RA;
%             else
%                 Cw = quat2rotmat([VcrossNorm*sin(alpha/2); cos(alpha/2)]);
%                 RW = rotationMatrix*Cw*[0;0;1];
%             end

            % normal force
            NA = cross(RA, Vcross); % normal axis
            if norm(NA) == 0
                normalForce = [0, 0, 0]'; 
            else
                normalForce = 0.5*rho*obj.rocket.Sm*CNa*alpha*Vmag^2*NA/norm(NA);
            end

            % Drag
            % Drag coefficient
            dragCoefficient = Nose_drag(obj.rocket, alpha, Vmag, nu, a)*obj.rocket.CD_fac; 
            if(t>obj.rocket.burnTime)
              dragCoefficient = dragCoefficient + drag_shuriken(obj.rocket, obj.rocket.ab_phi, alpha, Vmag, nu); 
            end
            % Drag force
            dragForce = -0.5*rho*obj.rocket.Sm*dragCoefficient*Vmag^2*Vnorm;

            % Total forces
            totalForce = ...
                thrust*obj.rocket.motor_fac +...  ;% Thrust
                gravityForce +...  ;% gravity
                normalForce +... ;% normal force
                dragForce      ; % drag force

            % Moment estimation

            %Aerodynamic corrective moment
            normalMoment = norm(normalForce)*margin*VcrossNorm;

            % Aerodynamic damping moment
            W_pitch = W - dot(W,RA)*RA; % extract pitch and yaw angular velocity
            CDM = pitchDampingMoment(obj.rocket, rho, CNa_bar, CP_bar, ...
                massRate, cm, norm(W_pitch) , Vmag); 
            dampingMoment = -0.5*rho*CDM*obj.rocket.Sm*Vmag^2*normalizeVect(W_pitch);

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
            W_dot = mldivide(I, totalMoment - inertiaRate*angle');

            % Return derivative vector
            sDot = [X_dot;V_dot;Q_dot;W_dot];
            
            % cache auxiliary result data
            obj.tmpNoseAlpha = alpha;
            obj.tmpNoseDelta = delta;
        end
        
        % --------------------------- 
        % 3DOF Payload descent Equations
        % ---------------------------
        
        function sDot = Payload_Dynamics_3DOF(obj, t, s, rocket, environment)

            X = s(1:3);
            V = s(4:6);

            % Earth coordinate vectors expressed in earth's frame
            XE = [1, 0, 0]';
            YE = [0, 1, 0]';
            ZE = [0, 0, 1]';

            % atmosphere
            [~, a, ~, rho, nu] = atmosphere(X(3)+environment.startAltitude, environment);

            % mass
            mass = rocket.pl_mass;

            V_rel = V -...
                 ... % Wind as computed by windmodel
                windModel(t, environment.turbI,environment.V_inf*environment.V_dir,...
                environment.turbModel);

            % gravity
            % Gravity
            gravityForce = -9.81*mass*ZE;
            % Drag
            % Drag coefficient
            SCD = 2.56e-2; 
            % Drag force
            dragForce = -0.5*rho*SCD*V_rel*norm(V_rel); 

            % Translational dynamics
            X_dot = V;
            V_dot = 1/mass*(dragForce + gravityForce);

            sDot = [X_dot; V_dot];

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
            Option = odeset('Events', @(t,x) RailEvent(t,x,obj.environment));

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
                railRotationMatrix = rotmat(obj.environment.railAzimuth, 3)*...
                    rotmat(obj.environment.railAngle, 2)*...
                    rotmat(obj.environment.railAzimuth, 3)';
                railVector = railRotationMatrix*[0;0;1];

                % Initial Conditions
                X0 = railVector*obj.environment.railLength; % spatial position of cm
                V0 = railVector*V; % Initial velocity of cm
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
            mass = obj.rocket.rocket_m - obj.rocket.pl_mass;

            % time span
            tspan = [T0, 5000];

            % options 
            Option = odeset('Events', @(T,X) MainEvent(T,X,obj.rocket));

            % integration
            [T3,S3, T3E, S3E, I3E] = ode45(@(t,s) obj.Dynamics_Parachute_3DOF(t,s,obj.rocket,obj.environment, mass, 0),tspan,S0, Option);
        
        end
        
        % --------------------------- 
        % Main Parachute Simulation
        % ---------------------------
        function [T4, S4, T4E, S4E, I4E] = MainParaSim(obj, T0, X0, V0)
            
            % initial conditions
            S0 = [X0; V0];

            % empty mass
            mass = obj.rocket.rocket_m - obj.rocket.pl_mass;

            % time span
            tspan = [T0, 5000];

            % options 
            Option = odeset('Events',@(T,X) CrashEvent(T,X,obj.environment));

            % integration
            [T4, S4, T4E, S4E, I4E] = ode45(@(t,s) obj.Dynamics_Parachute_3DOF(t,s,obj.rocket,obj.environment, mass, 1),tspan,S0, Option);
            
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
            Option = odeset('Events',@(T,X) CrashEvent(T,X,obj.environment));

            % integration
            [T5,S5, T5E, S5E, I5E] = ode45(@(t,s) obj.Dynamics_3DOF(t,s,obj.rocket,obj.environment),tspan,S0, Option);

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
            Option = odeset('Events',@(T,X) CrashEvent(T,X,obj.environment));

            % integration
            [T6,S6, T6E, S6E, I6E] = ode45(@(t,s) obj.Nose_Dynamics_3DOF(t,s,obj.rocket,obj.environment),tspan,S0, Option);

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
            Option = odeset('Events', @(T,X) CrashEvent(T,X,obj.environment),...
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
            Option = odeset('Events', @(T,X) CrashEvent(T,X,obj.environment));

            % integration
            [T7,S7, T7E, S7E, I7E] = ode45(@(t,s) obj.Payload_Dynamics_3DOF(t,s,obj.rocket,obj.environment),tspan,S0, Option);

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
            
            if obj.simOutput.Margin
                obj.simAuxResults.margin = [obj.simAuxResults.margin, obj.tmpMargin];
            end 
            if obj.simOutput.Alpha
                obj.simAuxResults.alpha = [obj.simAuxResults.alpha, obj.tmpAlpha];
            end 
            if obj.simOutput.Cn_alpha
                obj.simAuxResults.cnAlpha = [obj.simAuxResults.cnAlpha, obj.tmpCnAlpha];
            end 
            if obj.simOutput.Xcp
                obj.simAuxResults.xcp = [obj.simAuxResults.xcp, obj.tmpXcp];
            end 
            if obj.simOutput.Cd
                obj.simAuxResults.cd = [obj.simAuxResults.cd, obj.tmpCd];
            end 
            if obj.simOutput.Mass
                obj.simAuxResults.mass = [obj.simAuxResults.mass, obj.tmpMass];
            end 
            if obj.simOutput.CM
                obj.simAuxResults.cm = [obj.simAuxResults.cm, obj.tmpCm];
            end 
            if obj.simOutput.Il
                obj.simAuxResults.il = [obj.simAuxResults.il, obj.tmpIl];
            end 
            if obj.simOutput.Ir
                obj.simAuxResults.ir = [obj.simAuxResults.ir, obj.tmpIr];
            end
            if obj.simOutput.Delta
                obj.simAuxResults.delta = [obj.simAuxResults.delta, obj.tmpDelta];
            end
            
            if obj.simOutput.Nose_Alpha
                obj.simAuxResults.noseAlpha = [obj.simAuxResults.noseAlpha, obj.tmpNoseAlpha];
            end
            if obj.simOutput.Nose_Delta
                obj.simAuxResults.noseDelta = [obj.simAuxResults.noseDelta, obj.tmpNoseDelta];
            end
            
        end
        
    end
    
    function status = CrashOutputFunc(obj, T,S,flag)

        % keep simulation running
        status = 0;

        if isempty(flag) || (strcmp(flag, 'init') && obj.firstSimFlag)

            obj.firstSimFlag = 0;
            if obj.simOutput.Nose_Alpha
                obj.simAuxResults.noseAlpha = [obj.simAuxResults.noseAlpha, obj.tmpNoseAlpha];
            end
            if obj.simOutput.Nose_Delta
                obj.simAuxResults.noseDelta = [obj.simAuxResults.noseDelta, obj.tmpNoseDelta];
            end
            
        end
        
    end
end
end