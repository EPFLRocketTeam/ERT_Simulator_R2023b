classdef Simulator3D < handle
    
% -------------------------------------------------------------------------  
% Class properties
% -------------------------------------------------------------------------
   properties(Access = public)
      simAuxResults; 
   end

   properties(Access = public)
      rocket
      environment
      simOutput  
   end
   
   properties(Access = private)
      firstSimFlag = 1;
      tmpStabilityMargin
      tmpAngleOfAttack
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
       
       function obj = Simulator3D(rocket, environment, simOutput)
           if nargin == 0
               % TODO: Put default values or send warning message
           elseif nargin == 3
                obj.rocket = rocket;
                obj.environment = environment;
                obj.simOutput = simOutput;
           else
                error(['ERROR: In Simulator3D constructor, either no arguments '...
                    'or 3 arguments can be given. You gave ' num2str(nargin) '.']);
           end
 
           % Initialise Auxiliary results structure
           obj.simAuxResults.stabilityMargin = [];
           obj.simAuxResults.angleOfAttack = [];
           obj.simAuxResults.cnAlpha = [];
           obj.simAuxResults.centerOfPressure = [];
           obj.simAuxResults.dragCoefficient = [];
           obj.simAuxResults.mass = [];
           obj.simAuxResults.centerOfMass = [];
           obj.simAuxResults.inertiaLong = [];
           obj.simAuxResults.inertiaRot = [];
           obj.simAuxResults.flightPathAngle = [];
           
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
    
        function dS = Dynamics_Rail_1DOF(obj, t, s)

            x = s(1); % position
            v = s(2); % speed

            % rocket Inertia
            [mass, massRate] = Mass_Non_Lin(t,obj.rocket); % mass

            % Environment
            g = 9.81;  % Gravity [m/s2] 
            [~, speedOfSound, ~, density, kinematicViscosity] = atmosphere(x*sin(obj.environment.railAngle), obj.environment); % Atmosphere information

            % Force estimation

            % gravity
            gravityForce = -g*cos(obj.environment.railAngle)*mass;

            % Thrust 
            thrust = Thrust(t,obj.rocket); % (TODO: Allow for thrust vectoring -> error)

            % drag
            dragCoefficient = drag(obj.rocket, 0, v,kinematicViscosity, speedOfSound); % (TODO: make air-viscosity adaptable to temperature)
            dragForce = -0.5*density*obj.rocket.maxCrossSectionArea*dragCoefficient*v^2; % (TODO: define drag in wind coordinate system)

            totalForce = gravityForce + thrust*obj.rocket.motorThrustFactor + dragForce;

            % State derivatives
            
            positionDot = v;
            velocityDot = 1/mass*(totalForce);
            if velocityDot < 0
                positionDot = 0;
                velocityDot = 0;
            end % <--- ajout

            if velocityDot < 0
                positionDot = 0;
                velocityDot = 0;
            end 

            dS = [positionDot; velocityDot];
        end
       
        % --------------------------- 
        % 6DOF Flight Equations
        % ---------------------------
        function dS = Dynamics_6DOF(obj, t, s)

            position = s(1:3);
            velocity = s(4:6);
            quaternion = s(7:10);
            angularVelocity = s(11:13);

            % Check quaternion norm
            quaternion = normalizeVect(quaternion);

            % Coordinate systems

            % Rotation matrix from rocket coordinates to Earth coordinates
            rotationMatrix = quat2rotmat(quaternion);
            eulerAngles = rot2anglemat(rotationMatrix);

            % rocket principle frame vectors expressed in earth coordinates
            yawAxis = rotationMatrix*[1,0,0]'; % Yaw axis
            pitchAxis = rotationMatrix*[0,1,0]'; % Pitch Axis
            rollAxis = rotationMatrix*[0,0,1]'; % Roll Axis

            % Earth coordinate vectors expressed in earth's frame
            xEarth = [1, 0, 0]';
            yEarth = [0, 1, 0]';
            zEarth = [0, 0, 1]';

            % rocket Inertia
            [mass,massRate,centerOfMass,~,inertiaLong,~,inertiaRot,~] = Mass_Properties(t,obj.rocket,'NonLinear');
            %inertiaMatrix = rotationMatrix'*diag([inertiaLong, inertiaLong, inertiaRot])*rotationMatrix; % Inertia TODO: inertiaRot in Mass_Properties
            
            % Inertia using the given I_rocket and the motor
            % Compute I_motor (approximate by speedOfSound cylinder)
            motorInertia = inertia_fill_cylinder(mass, ...
                obj.rocket.motor_length, obj.rocket.motor_dia / 2);
            % Total inertia
            %inertiaMatrix = inertial_matrix(obj.rocket, centerOfMass, t);
            %disp(inertiaMatrix)
            inertiaMatrix = obj.rocket.emptyInertia + motorInertia;
            %disp(inertiaMatrix)
            %disp("==============================")
            inertiaMatrix = rotationMatrix' * inertiaMatrix * rotationMatrix; % Transfert to earth coordinates

            % Temporal derivative of inertial matrix
            dIdt = inertia_fill_cylinder(massRate, obj.rocket.motor_length, ...
                obj.rocket.motor_dia / 2); % Inertial matrix time derivative
            dIdt = rotationMatrix' * dIdt * rotationMatrix; % Transfert to earth coordinates

            % Environment
            g = 9.81;               % Gravity [m/s2] 
            [~, speedOfSound, ~, density, kinematicViscosity] = atmosphere(position(3)+obj.environment.startAltitude,...
                obj.environment); % Atmosphere information 

            % Force estimations 

            % Thrust
            % Oriented along roll axis of rocket frame, expressed in earth coordinates. 
            thrust = Thrust(t,obj.rocket)*rollAxis; % (TODO: Allow for thrust vectoring -> error)

            % Gravity
            gravityForce = -g*mass*zEarth;

            % Aerodynamic corrective forces
            % Compute center of mass eulerAngles of attack
            centerOfMassVelocity = velocity -...
                     ... % Wind as computed by windmodel
                windModel(t, obj.environment.Turb_I,obj.environment.V_inf*obj.environment.V_dir,...
                obj.environment.Turb_model,position(3)); 

            centerOfMassSpeed = norm(centerOfMassVelocity);
            centerOfMassAngleOfAttack = atan2(norm(cross(rollAxis, centerOfMassVelocity)), dot(rollAxis, centerOfMassVelocity));

            % matchNumber number
            matchNumber = centerOfMassSpeed/speedOfSound;
            % Normal lift coefficient and center of pressure
            [normalForceCoefficient, centerOfPressure,normalForceCoefficientBar,centerOfPressureBar] = normalLift(obj.rocket, centerOfMassAngleOfAttack, 1.1,...
                matchNumber, eulerAngles(3), 1);
            
            
            % Stability margin
            margin = (centerOfPressure-centerOfMass);

            % Compute rocket eulerAngles of attack
            normalizedAngularVelocity = angularVelocity/norm(angularVelocity);
            if(isnan(normalizedAngularVelocity))
                normalizedAngularVelocity  = zeros(3,1);
            end
            relativeVelocity = centerOfMassVelocity + margin*sin(acos(dot(rollAxis,normalizedAngularVelocity)))*(cross(rollAxis, angularVelocity));
            relativeSpeed = norm(relativeVelocity);
            normalizedVelocity = normalizeVect(relativeVelocity);

            % Angle of attack 
            velocityCrossProduct = cross(rollAxis, normalizedVelocity);
            normalizedCrossProduct = normalizeVect(velocityCrossProduct);
            angleOfAttack = atan2(norm(cross(rollAxis, normalizedVelocity)), dot(rollAxis, normalizedVelocity));
            flightPathAngle = atan2(norm(cross(rollAxis, zEarth)), dot(rollAxis, zEarth));
            
            % wind coordinate transformation
%             if(abs(angleOfAttack)<1e-3)
%                 RW = rollAxis;
%             elseif(abs(angleOfAttack-pi)<1e-3)
%                 RW = -rollAxis;
%             else
%                 Cw = quat2rotmat([normalizedCrossProduct*sin(angleOfAttack/2); cos(angleOfAttack/2)]);
%                 RW = rotationMatrix*Cw*[0;0;1];
%             end

            % normal force
            normalAxis = cross(rollAxis, velocityCrossProduct); % normal axis
            if norm(normalAxis) == 0
                normalForce = [0, 0, 0]'; 
            else
                normalForce = 0.5*density*obj.rocket.maxCrossSectionArea*normalForceCoefficient*angleOfAttack*relativeSpeed^2*normalAxis/norm(normalAxis);
            end
            % Drag
            % Drag coefficient
            dragCoefficient = drag(obj.rocket, angleOfAttack, relativeSpeed, kinematicViscosity, speedOfSound)*obj.rocket.dragCoefficientFactor; 
            if(t>obj.rocket.Burn_Time)
              dragCoefficient = dragCoefficient + drag_shuriken(obj.rocket, obj.rocket.airbrakeAngle, angleOfAttack, relativeSpeed, kinematicViscosity); 
            end
            % Drag force
            dragForce = -0.5*density*obj.rocket.maxCrossSectionArea*dragCoefficient*relativeSpeed^2*normalizedVelocity ;

            % Total forces
            totalForce = ...
                thrust*obj.rocket.motorThrustFactor +...  ;% Thrust
                gravityForce +...  ;% gravity
                normalForce +... ;% normal force
                dragForce      ; % drag force

            % Moment estimation

            %Aerodynamic corrective moment
            normalMoment = norm(normalForce)*margin*normalizedCrossProduct ;

            % Aerodynamic damping moment
            pitchAngularVelocity = angularVelocity - dot(angularVelocity,rollAxis)*rollAxis; % extract pitch and yaw angular velocity
            pitchDampingCoefficient = pitchDampingMoment(obj.rocket, density, normalForceCoefficientBar, centerOfPressureBar, ...
                massRate, centerOfMass, norm(pitchAngularVelocity) , relativeSpeed); 
            
            dampingMoment = -0.5*density*pitchDampingCoefficient*obj.rocket.maxCrossSectionArea*relativeSpeed^2*normalizeVect(pitchAngularVelocity);
            
            totalMoment = ...
                normalMoment...  ; % aerodynamic corrective moment
               + dampingMoment; % aerodynamic damping moment

            % State derivatives

            % Translational dynamics
            dx = velocity;
            dv = 1/mass*(totalForce);

            % Rotational dynamics
            dQuaternion = quat_evolve(quaternion, angularVelocity);
            
            %dAngularVelocity = pinv(inertiaMatrix)*totalMoment;
            %dAngularVelocity = mldivide(inertiaMatrix,totalMoment); % (TODO: Add inertia variation with time)
            dAngularVelocity = mldivide(inertiaMatrix, totalMoment - dIdt*eulerAngles');

            % Return derivative vector
            dS = [dx;dv;dQuaternion;dAngularVelocity];
            
            % cache auxiliary result data
            obj.tmpStabilityMargin = margin/obj.rocket.maxDiameter;
            obj.tmpAngleOfAttack = angleOfAttack;
            obj.tmpCnAlpha = normalForceCoefficient;
            obj.tmpXcp = centerOfPressure;
            obj.tmpDragCoefficient = dragCoefficient;
            obj.tmpMass = mass;
            obj.tmpCenterOfMass = centerOfMass;
            obj.tmpInertiaLong = inertiaLong;
            obj.tmpInertiaRot = inertiaRot;
            obj.tmpDelta = flightPathAngle;
        end

        function dS = Dynamics_6DOF2(obj, t, s)

            position = s(1:3);
            velocity = s(4:6);
            quaternion = s(7:10);
            angularVelocity = s(11:13);

            % Check quaternion norm
            quaternion = normalizeVect(quaternion);

            % Coordinate systems

            % Rotation matrix from rocket coordinates to Earth coordinates
            rotationMatrix = quat2rotmat(quaternion);
            angleOfAttack = rot2anglemat(rotationMatrix);

            % rocket principle frame vectors expressed in earth coordinates
            yawAxis = rotationMatrix*[1,0,0]'; % Yaw axis
            pitchAxis = rotationMatrix*[0,1,0]'; % Pitch Axis
            rollAxis = rotationMatrix*[0,0,1]'; % Roll Axis

            % Earth coordinate vectors expressed in earth's frame
            xEarth = [1, 0, 0]';
            yEarth = [0, 1, 0]';
            zEarth = [0, 0, 1]';

            % rocket Inertia
            [mass, massRate, centerOfMass, ~, inertiaLong, ~, inertiaRot, ~] = Mass_Properties(t,obj.rocket,'NonLinear');
            %inertiaMatrix = rotationMatrix'*diag([inertiaLong, inertiaLong, inertiaRot])*rotationMatrix; % Inertia TODO: inertiaRot in Mass_Properties
            
            % Inertia using the given I_rocket and the motor
            % Compute I_motor (approximate by speedOfSound cylinder)
            motorInertia = inertia_fill_cylinder(mass, ...
                obj.rocket.motor_length, obj.rocket.motor_dia / 2);
            
             % Total inertia
            inertiaMatrix = obj.rocket.emptyInertia + motorInertia;
            inertiaMatrix = rotationMatrix' * inertiaMatrix * rotationMatrix; % Transfer to earth coordinates

            % Temporal derivative of inertial matrix
            inertiaRot = inertia_fill_cylinder(massRate, obj.rocket.motor_length, ...
                obj.rocket.motor_dia / 2); % Inertial matrix time derivative
            inertiaRot = rotationMatrix' * inertiaRot * rotationMatrix; % Transfert to earth coordinates

            % Environment
            g = 9.81; % Gravity [m/s2] 
            [~, speedOfSound, ~, density, kinematicViscosity] = atmosphere(position(3)+obj.environment.startAltitude,...
                obj.environment); % Atmosphere information 

            % Force estimations 

            % Thrust
            % Oriented along roll axis of rocket frame, expressed in earth coordinates. 
            thrust = Thrust(t,obj.rocket)*rollAxis; % (TODO: Allow for thrust vectoring -> error)

            % Gravity
            gravityForce = -g*mass*zEarth;

            % Aerodynamic corrective forces
            % Compute center of mass angleOfAttack of attack
            centerOfMassVelocity = velocity -...
                     ... % Wind as computed by windmodel
                windModel(t, obj.environment.Turb_I,obj.environment.V_inf*obj.environment.V_dir,...
                obj.environment.Turb_model,position(3)); 

            centerOfMassSpeed = norm(centerOfMassVelocity);
            centerOfMassAngleOfAttack = atan2(norm(cross(rollAxis, centerOfMassVelocity)), dot(rollAxis, centerOfMassVelocity));

            % matchNumber number
            matchNumber = centerOfMassSpeed/speedOfSound;

            % Normal lift coefficient and center of pressure
            [normalForceCoefficient, centerOfPressure, normalForceCoefficientBar, ...
                centerOfPressureBar] = normalLift(obj.rocket, centerOfMassAngleOfAttack, 1.1,...
                matchNumber, angleOfAttack(3), 1);
            
            
            % Stability margin
            stabilityMargin = (centerOfPressure- centerOfMass);

            % Compute rocket angleOfAttack of attack
            normalizedAngularVelocity = angularVelocity/norm(angularVelocity);
            if(isnan(normalizedAngularVelocity))
                normalizedAngularVelocity  = zeros(3,1);
            end
            relativeVelocity = centerOfMassVelocity + stabilityMargin*sin(acos(dot(rollAxis,normalizedAngularVelocity)))*(cross(rollAxis, angularVelocity));
            relativeSpeed = norm(relativeVelocity);
            normalizedVelocity = normalizeVect(relativeVelocity);

            % Angle of attack 
            velocityCrossProduct = cross(rollAxis, normalizedVelocity);
            normalizedCrossProduct = normalizeVect(velocityCrossProduct);
            angleOfAttack = atan2(norm(cross(rollAxis, normalizedVelocity)), dot(rollAxis, normalizedVelocity));
            flightPathAngle = atan2(norm(cross(rollAxis, zEarth)), dot(rollAxis, zEarth));
            
            % wind coordinate transformation
%             if(abs(angleOfAttack)<1e-3)
%                 RW = rollAxis;
%             elseif(abs(angleOfAttack-pi)<1e-3)
%                 RW = -rollAxis;
%             else
%                 Cw = quat2rotmat([normalizedCrossProduct*sin(angleOfAttack/2); cos(angleOfAttack/2)]);
%                 RW = rotationMatrix*Cw*[0;0;1];
%             end

            % normal force
            normalAxis = cross(rollAxis, velocityCrossProduct); % normal axis
            if norm(normalAxis) == 0
                normalForce = [0, 0, 0]'; 
            else
                normalForce = 0.5*density*obj.rocket.maxCrossSectionArea*normalForceCoefficient*angleOfAttack*relativeSpeed^2*normalAxis/norm(normalAxis);
            end
            % Drag
            % Drag coefficient
            dragCoefficient = drag(obj.rocket, angleOfAttack, relativeSpeed, kinematicViscosity, speedOfSound)*obj.rocket.dragCoefficientFactor; 
            if(t>obj.rocket.Burn_Time)
              dragCoefficient = dragCoefficient + drag_shuriken(obj.rocket, obj.rocket.airbrakeAngle, angleOfAttack, relativeSpeed, kinematicViscosity); 
            end
            % Drag force
            dragForce = -0.5*density*obj.rocket.maxCrossSectionArea*dragCoefficient*relativeSpeed^2*normalizedVelocity ;

            % Total forces
            totalForce = ...
                thrust*obj.rocket.motorThrustFactor +...  ;% Thrust
                gravityForce +...  ;% gravity
                normalForce +... ;% normal force
                dragForce      ; % drag force

            % Moment estimation

            %Aerodynamic corrective moment
            normalMoment = norm(normalForce)*stabilityMargin*normalizedCrossProduct ;

            % Aerodynamic damping moment
            pitchAngularVelocity = angularVelocity - dot(angularVelocity,rollAxis)*rollAxis; % extract pitch and yaw angular velocity
            pitchDampingCoefficient = pitchDampingMoment(obj.rocket, density,  normalForceCoefficientBar, centerOfPressureBar, ...
                massRate,  centerOfMass, norm(pitchAngularVelocity) , relativeSpeed); 
            
            dampingMoment = -0.5*density*pitchDampingCoefficient*obj.rocket.maxCrossSectionArea*relativeSpeed^2*normalizeVect(pitchAngularVelocity);
            
            totalMoment = ...
                normalMoment...  ; % aerodynamic corrective moment
               + dampingMoment; % aerodynamic damping moment

            % State derivatives

            % Translational dynamics
            positionDot = velocity;
            velocityDot = 1/mass*(totalForce);

            % Rotational dynamics
            quaternionDot = quat_evolve(quaternion, angularVelocity);
            
            %angularVelocityDot = pinv(inertiaMatrix)*totalMoment;
            %angularVelocityDot = mldivide(inertiaMatrix,totalMoment); % (TODO: Add inertia variation with time)
            angularVelocityDot = mldivide(inertiaMatrix, totalMoment - inertiaRot*angleOfAttack');

            % Return derivative vector
            dS = [positionDot;velocityDot;quaternionDot;angularVelocityDot];
            
            % cache auxiliary result data
            obj.tmpStabilityMargin = stabilityMargin/obj.rocket.maxDiameter;
            obj.tmpAngleOfAttack = angleOfAttack;
            obj.tmpCnAlpha = normalForceCoefficient;
            obj.tmpXcp = centerOfPressure;
            obj.tmpDragCoefficient = dragCoefficient;
            obj.tmpMass = mass;
            obj.tmpCenterOfMass =  centerOfMass;
            obj.tmpInertiaLong = inertiaLong;
            obj.tmpInertiaRot = inertiaRot;
            obj.tmpDelta = flightPathAngle;
        end
        
        % --------------------------- 
        % 3DOF Parachute descent Equations
        % ---------------------------
        
        function dsdt = Dynamics_Parachute_3DOF(obj, t,s, rocket, environment, mass, main)

            position = s(1:3);
            velocity = s(4:6);

            % Atmospheric Data
            [~, ~, ~, density] = atmosphere(position(3)+environment.startAltitude, environment); % Atmosphere [K,m/s,Pa,kg/m3]

            % Aerodynamic force
            relativeVelocity = -velocity + ...
                 ... % Wind as computed by windmodel
                windModel(t, environment.Turb_I,environment.V_inf*environment.V_dir,...
                environment.Turb_model,position(3));

            if main
                scd = rocket.mainParachuteDragArea;
            elseif main == 0
                scd = rocket.drogueParachuteDragArea;
            end
            dragForce = 0.5*density*scd*norm(relativeVelocity)*relativeVelocity;

            % Gravity force
            g = 9.81*[0;0;-1];
            gravityForce = g*mass;

            positionDot = velocity;
            velocityDot = (dragForce+gravityForce)/mass;

            dsdt = [positionDot; velocityDot];
        end
        
        % --------------------------- 
        % 3DOF Crash descent Equations
        % ---------------------------
        
        function dS = Dynamics_3DOF(obj, t, s, rocket, environment)

            position = s(1:3);
            velocity = s(4:6);

            % Earth coordinate vectors expressed in earth's frame
            xEarth = [1, 0, 0]';
            yEarth = [0, 1, 0]';
            zEarth = [0, 0, 1]';

            % atmosphere
            [~, speedOfSound, ~, density, kinematicViscosity] = atmosphere(position(3)+environment.startAltitude, environment);

            % mass
            mass = rocket.emptyMass;

            relativeVelocity = velocity -...
                 ... % Wind as computed by windmodel
                windModel(t, environment.Turb_I,environment.V_inf*environment.V_dir,...
                environment.Turb_model,position(3));

            % gravity
            % Gravity
            gravityForce = -9.81*mass*zEarth;
            % Drag
            % Drag coefficient
            dragCoefficient = drag(rocket, 0, norm(relativeVelocity), kinematicViscosity, speedOfSound); % (TODO: make air-viscosity adaptable to temperature)
            % Drag force
            dragForce = -0.5*density*rocket.maxCrossSectionArea*dragCoefficient*relativeVelocity*norm(relativeVelocity); 
            % Translational dynamics
            positionDot = velocity;
            velocityDot = 1/mass*(dragForce + gravityForce);

            dS = [positionDot; velocityDot];

        end
        
        % --------------------------- 
        % 3DOF Nosecone Crash descent Equations
        % ---------------------------
    
        function dS = Nose_Dynamics_3DOF(obj, t, s, rocket, environment)

            position = s(1:3);
            velocity = s(4:6);

            % Earth coordinate vectors expressed in earth's frame
            xEarth = [1, 0, 0]';
            yEarth = [0, 1, 0]';
            zEarth = [0, 0, 1]';

            % atmosphere
            [~, speedOfSound, ~, density, kinematicViscosity] = atmosphere(position(3)+environment.startAltitude, environment);

            % mass
            mass = rocket.emptyMass;

            relativeVelocity = velocity -...
                 ... % Wind as computed by windmodel
                windModel(t, environment.Turb_I,environment.V_inf*environment.V_dir,...
                environment.Turb_model,position(3));

            % gravity
            % Gravity
            gravityForce = -9.81*mass*zEarth;
            % Drag
            % Drag coefficient
            dragCoefficient = Nose_drag(rocket, 0, norm(relativeVelocity), kinematicViscosity, speedOfSound); % (TODO: make air-viscosity adaptable to temperature)
            % Drag force
            dragForce = -0.5*density*rocket.maxCrossSectionArea*dragCoefficient*relativeVelocity*norm(relativeVelocity); 

            % Translational dynamics
            positionDot = velocity;
            velocityDot = 1/mass*(dragForce + gravityForce);

            dS = [positionDot; velocityDot];

        end
        
        % --------------------------- 
        % 6DOF Nosecone Crash descent Equations
        % ---------------------------
        
        function dS = Nose_Dynamics_6DOF(obj, t, s)

            position = s(1:3);
            velocity = s(4:6);
            quaternion = s(7:10);
            angularVelocity = s(11:13);

            % Check quaternion norm
            quaternion = normalizeVect(quaternion);

            % Coordinate systems

            % Rotation matrix from rocket coordinates to Earth coordinates
            rotationMatrix = quat2rotmat(quaternion);
            angleOfAttack = rot2anglemat(rotationMatrix);

            % rocket principle frame vectors expressed in earth coordinates
            yawAxis = rotationMatrix*[1,0,0]'; % Yaw axis
            pitchAxis = rotationMatrix*[0,1,0]'; % Pitch Axis
            rollAxis = rotationMatrix*[0,0,1]'; % Roll Axis

            % Earth coordinate vectors expressed in earth's frame
            xEarth = [1, 0, 0]';
            yEarth = [0, 1, 0]';
            zEarth = [0, 0, 1]';

            % rocket Inertia
            [mass,massRate, centerOfMass,~,inertiaLong,~,inertiaRot,~] = Mass_Properties(t,obj.rocket,'NonLinear');
            %inertiaMatrix = rotationMatrix'*diag([inertiaLong, inertiaLong, inertiaRot])*rotationMatrix; % Inertia TODO: inertiaRot in Mass_Properties

            % Inertia using the given I_rocket and the motor
            % Compute I_motor (approximate by speedOfSound cylinder)
            motorInertia = inertia_fill_cylinder(mass, ...
                obj.rocket.motor_length, obj.rocket.motor_dia / 2);
            % Total inertia
            inertiaMatrix = obj.rocket.emptyInertia + motorInertia;
            inertiaMatrix = rotationMatrix' * inertiaMatrix * rotationMatrix; % Transfert to earth coordinates

            % Temporal derivative of inertial matrix
            inertiaRot = inertia_fill_cylinder(massRate, obj.rocket.motor_length, ...
                obj.rocket.motor_dia / 2); % Inertial matrix time derivative
            inertiaRot = rotationMatrix' * inertiaRot * rotationMatrix; % Transfert to earth coordinates

            % environment
            g = 9.81;               % Gravity [m/s2]
            [~, speedOfSound, ~, density, kinematicViscosity] = atmosphere(position(3)+obj.environment.startAltitude,...
                obj.environment); % Atmosphere information 

            % Force estimations 

            % Thrust
            % Oriented along roll axis of rocket frame, expressed in earth coordinates. 
            thrust = Thrust(t,obj.rocket)*rollAxis; % (TODO: Allow for thrust vectoring -> error)

            % Gravity
            gravityForce = -g*mass*zEarth;

            % Aerodynamic corrective forces
            % Compute center of mass angleOfAttack of attack
            centerOfMassVelocity = velocity -...
                     ... % Wind as computed by windmodel
                windModel(t, obj.environment.Turb_I,obj.environment.V_inf*obj.environment.V_dir,...
                obj.environment.Turb_model,position(3)); 

            centerOfMassSpeed = norm(centerOfMassVelocity);
            centerOfMassAngleOfAttack = atan2(norm(cross(rollAxis, centerOfMassVelocity)), dot(rollAxis, centerOfMassVelocity));

            % Match number
            matchNumber = centerOfMassSpeed/speedOfSound;
            % Normal lift coefficient and center of pressure
            [normalForceCoefficient, centerOfPressure, normalForceCoefficientBar,centerOfPressureBar] = normalLift(obj.rocket, centerOfMassAngleOfAttack, 1.1,...
                matchNumber, angleOfAttack(3), 1);
            % Stability stabilityMargin
            stabilityMargin = (centerOfPressure- centerOfMass);

            % Compute rocket angleOfAttack of attack
            normalizedAngularVelocity = angularVelocity/norm(angularVelocity);
            if(isnan(normalizedAngularVelocity))
                normalizedAngularVelocity  = zeros(3,1);
            end
            relativeVelocity = centerOfMassVelocity + stabilityMargin*sin(acos(dot(rollAxis,normalizedAngularVelocity)))*(cross(rollAxis, angularVelocity));
            relativeSpeed = norm(relativeVelocity);
            normalizedVelocity = normalizeVect(relativeVelocity);

            % Angle of attack 
            velocityCrossProduct = cross(rollAxis, normalizedVelocity);
            normalizedCrossProduct = normalizeVect(velocityCrossProduct);
            angleOfAttack = atan2(norm(cross(rollAxis, normalizedVelocity)), dot(rollAxis, normalizedVelocity));
            flightPathAngle = atan2(norm(cross(rollAxis, zEarth)), dot(rollAxis, zEarth));

            % wind coordinate transformation
%             if(abs(angleOfAttack)<1e-3)
%                 RW = rollAxis;
%             elseif(abs(angleOfAttack-pi)<1e-3)
%                 RW = -rollAxis;
%             else
%                 Cw = quat2rotmat([normalizedCrossProduct*sin(angleOfAttack/2); cos(angleOfAttack/2)]);
%                 RW = rotationMatrix*Cw*[0;0;1];
%             end

            % normal force
            normalAxis = cross(rollAxis, velocityCrossProduct); % normal axis
            if norm(normalAxis) == 0
                normalForce = [0, 0, 0]'; 
            else
                normalForce = 0.5*density*obj.rocket.maxCrossSectionArea*normalForceCoefficient*angleOfAttack*relativeSpeed^2*normalAxis/norm(normalAxis);
            end

            % Drag
            % Drag coefficient
            dragCoefficient = Nose_drag(obj.rocket, angleOfAttack, relativeSpeed, kinematicViscosity, speedOfSound)*obj.rocket.dragCoefficientFactor; 
            if(t>obj.rocket.Burn_Time)
              dragCoefficient = dragCoefficient + drag_shuriken(obj.rocket, obj.rocket.airbrakeAngle, angleOfAttack, relativeSpeed, kinematicViscosity); 
            end
            % Drag force
            dragForce = -0.5*density*obj.rocket.maxCrossSectionArea*dragCoefficient*relativeSpeed^2*normalizedVelocity;

            % Total forces
            totalForce = ...
                thrust*obj.rocket.motorThrustFactor +...  ;% Thrust
                gravityForce +...  ;% gravity
                normalForce +... ;% normal force
                dragForce      ; % drag force

            % Moment estimation

            %Aerodynamic corrective moment
            normalMoment = norm(normalForce)*stabilityMargin*normalizedCrossProduct;

            % Aerodynamic damping moment
            pitchAngularVelocity = angularVelocity - dot(angularVelocity,rollAxis)*rollAxis; % extract pitch and yaw angular velocity
            pitchDampingCoefficient = pitchDampingMoment(obj.rocket, density,  normalForceCoefficientBar, centerOfPressureBar, ...
                massRate,  centerOfMass, norm(pitchAngularVelocity) , relativeSpeed); 
            dampingMoment = -0.5*density*pitchDampingCoefficient*obj.rocket.maxCrossSectionArea*relativeSpeed^2*normalizeVect(pitchAngularVelocity);

            totalMoment = ...
                normalMoment...  ; % aerodynamic corrective moment
               + dampingMoment ; % aerodynamic damping moment

            % State derivatives

            % Translational dynamics
            positionDot = velocity;
            velocityDot = 1/mass*(totalForce);

            % Rotational dynamics
            quaternionDot = quat_evolve(quaternion, angularVelocity);
            %angularVelocityDot = mldivide(inertiaMatrix,totalMoment); % (TODO: Add inertia variation with time)
            angularVelocityDot = mldivide(inertiaMatrix, totalMoment - inertiaRot*angleOfAttack');

            % Return derivative vector
            dS = [positionDot;velocityDot;quaternionDot;angularVelocityDot];
            
            % cache auxiliary result data
            obj.tmpNoseAlpha = angleOfAttack;
            obj.tmpNoseDelta = flightPathAngle;
        end
        
        % --------------------------- 
        % 3DOF Payload descent Equations
        % ---------------------------
        
        function dS = Payload_Dynamics_3DOF(obj, t, s, rocket, environment)

            position = s(1:3);
            velocity = s(4:6);

            % Earth coordinate vectors expressed in earth's frame
            xEarth = [1, 0, 0]';
            yEarth = [0, 1, 0]';
            zEarth = [0, 0, 1]';

            % atmosphere
            [~, speedOfSound, ~, density, kinematicViscosity] = atmosphere(position(3)+environment.startAltitude, environment);

            % mass
            mass = rocket.payloadMass;

            relativeVelocity = velocity -...
                 ... % Wind as computed by windmodel
                windModel(t, environment.Turb_I,environment.V_inf*environment.V_dir,...
                environment.Turb_model);

            % gravity
            % Gravity
            gravityForce = -9.81*mass*zEarth;
            % Drag
            % Drag coefficient
            scd = 2.56e-2; 
            % Drag force
            dragForce = -0.5*density*scd*relativeVelocity*norm(relativeVelocity); 

            % Translational dynamics
            positionDot = velocity;
            velocityDot = 1/mass*(dragForce + gravityForce);

            dS = [positionDot; velocityDot];

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
            timeSpan = [0, 5];

            % options
            options = odeset('Events', @(t,x) RailEvent(t,x,obj.environment));

            % integration
            [T1, S1] = ode45(@(t,x) obj.Dynamics_Rail_1DOF(t, x), timeSpan, X0, options); 
            
        end
        
        % --------------------------- 
        % Flight Simulation
        % ---------------------------
        function [T2, S2, T2E, S2E, I2E] = FlightSim(obj, tspan, arg2, arg3, arg4, arg5)
            
            if (nargin == 3)
                % Compute initial conditions based on rail output values
                velocity = arg2;
                
                % Rail vector
                C_rail = rotmat(obj.environment.railAzimuth, 3)*...
                    rotmat(obj.environment.railAngle, 2)*...
                    rotmat(obj.environment.railAzimuth, 3)';
                RV = C_rail*[0;0;1];

                % Initial Conditions
                X0 = RV*obj.environment.railLength; % spatial position of cm
                V0 = RV*velocity; % Initial velocity of cm
                Q0 = rot2quat(C_rail'); % Initial attitude
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
            options = odeset('Events', @ApogeeEvent, 'RelTol', 1e-6, 'AbsTol', 1e-6,...
                            'OutputFcn', @(thrust,S,flag) obj.FlightOutputFunc(thrust,S,flag),...
                            'Refine', 1);

            % integration
            [T2,S2, T2E, S2E, I2E] = ode45(@(t,s) obj.Dynamics_6DOF(t,s),tspan,S0, options);
            
        end
                
        % --------------------------- 
        % Drogue Parachute Simulation
        % ---------------------------
        function [drogueTime, drogueState, drogueTimeEvents, drogueStateEvents, drogueEventIndices] = DrogueParaSim(obj, T0, X0, V0)
            
            % initial conditions
            S0 = [X0; V0];

            % empty mass
            mass = obj.rocket.emptyMass - obj.rocket.payloadMass;

            % time span
            timeSpan = [T0, 5000];

            % options 
            options = odeset('Events', @(thrust,position) MainEvent(thrust,position,obj.rocket));

            % integration
            [drogueTime,drogueState, drogueTimeEvents, drogueStateEvents, drogueEventIndices] = ode45(@(t,s) obj.Dynamics_Parachute_3DOF(t,s,obj.rocket,obj.environment, mass, 0),timeSpan,S0, options);
        
        end
        
        % --------------------------- 
        % main Parachute Simulation
        % ---------------------------
        function [mainChuteTime, mainChuteState, mainChuteTimeEvents, S4E, mainChuteEventsIndices] = MainParaSim(obj, T0, X0, V0)
            
            % initial conditions
            S0 = [X0; V0];

            % empty mass
            mass = obj.rocket.emptyMass - obj.rocket.payloadMass;

            % time span
            timeSpan = [T0, 5000];

            % options 
            options = odeset('Events',@(thrust,position) CrashEvent(thrust,position,obj.environment));

            % integration
            [mainChuteTime, mainChuteState, mainChuteTimeEvents, S4E, mainChuteEventsIndices] = ode45(@(t,s) obj.Dynamics_Parachute_3DOF(t,s,obj.rocket,obj.environment, mass, 1),timeSpan,S0, options);
            
        end
        
        % --------------------------- 
        % Crash Simulation
        % ---------------------------
        function [crashTime, crashState, crashTimeEvents, crashStateEvents, crashEventIndices] = CrashSim(obj, T0, X0, V0)
            
            % Initial Conditions
            S0 = [X0; V0];

            % time span
            timeSpan = [T0, 100];

            % options
            options = odeset('Events',@(thrust,position) CrashEvent(thrust,position,obj.environment));

            % integration
            [crashTime,crashState, crashTimeEvents, crashStateEvents, crashEventIndices] = ode45(@(t,s) obj.Dynamics_3DOF(t,s,obj.rocket,obj.environment),timeSpan,S0, options);

        end
        
        % --------------------------- 
        % Nosecone Crash Simulation 3DOF
        % ---------------------------
        function [T6, S6, T6E, S6E, I6E] = Nose_CrashSim_3DOF(obj, T0, X0, V0)
            
            % Initial Conditions
            S0 = [X0; V0];

            % time span
            timeSpan = [T0, 1000];

            % options
            options = odeset('Events',@(thrust,position) CrashEvent(thrust,position,obj.environment));

            % integration
            [T6,S6, T6E, S6E, I6E] = ode45(@(t,s) obj.Nose_Dynamics_3DOF(t,s,obj.rocket,obj.environment),timeSpan,S0, options);

        end
        
        % --------------------------- 
        % Nosecone Crash Simulation 6DOF
        % ---------------------------
        function [T6, S6, T6E, S6E, I6E] = Nose_CrashSim_6DOF(obj, timeSpan, arg2, arg3, arg4, arg5)
            
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
            options = odeset('Events', @(thrust,position) CrashEvent(thrust,position,obj.environment),...
                            'OutputFcn', @(thrust,S,flag) obj.CrashOutputFunc(thrust,S,flag),...
                            'Refine', 1);

            % integration
            [T6,S6, T6E, S6E, I6E] = ode45(@(t,s) obj.Nose_Dynamics_6DOF(t,s),timeSpan,S0, options);
            
        end
        
        % --------------------------- 
        % Payload Impact Simulation
        % ---------------------------
        function [T7, S7, T7E, S7E, I7E] = PayloadCrashSim(obj, T0, X0, V0)
            
            % Initial Conditions
            S0 = [X0; V0];

            % time span
            timeSpan = [T0, 1000];

            % options
            options = odeset('Events', @(thrust,position) CrashEvent(thrust,position,obj.environment));

            % integration
            [T7,S7, T7E, S7E, I7E] = ode45(@(t,s) obj.Payload_Dynamics_3DOF(t,s,obj.rocket,obj.environment),timeSpan,S0, options);

        end
    end
    
% -------------------------------------------------------------------------  
% Private methods
% -------------------------------------------------------------------------  
methods(Access = private)
    function status = FlightOutputFunc(obj, thrust, S, flag)

        % keep simulation running
        status = 0;

        if isempty(flag) || (strcmp(flag, 'init') && obj.firstSimFlag)

            obj.firstSimFlag = 0;
            
            if obj.simOutput.Margin
                obj.simAuxResults.stabilityMargin = [obj.simAuxResults.stabilityMargin, obj.tmpStabilityMargin];
            end 
            if obj.simOutput.Alpha
                obj.simAuxResults.angleOfAttack = [obj.simAuxResults.angleOfAttack, obj.tmpAngleOfAttack];
            end 
            if obj.simOutput.Cn_alpha
                obj.simAuxResults.cnAlpha = [obj.simAuxResults.cnAlpha, obj.tmpCnAlpha];
            end 
            if obj.simOutput.Xcp
                obj.simAuxResults.centerOfPressure = [obj.simAuxResults.centerOfPressure, obj.tmpXcp];
            end 
            if obj.simOutput.Cd
                obj.simAuxResults.dragCoefficient = [obj.simAuxResults.dragCoefficient, obj.tmpDragCoefficient];
            end 
            if obj.simOutput.Mass
                obj.simAuxResults.mass = [obj.simAuxResults.mass, obj.tmpMass];
            end 
            if obj.simOutput.CM
                obj.simAuxResults.centerOfMass = [obj.simAuxResults.centerOfMass, obj.tmpCenterOfMass];
            end 
            if obj.simOutput.Il
                obj.simAuxResults.inertiaLong = [obj.simAuxResults.inertiaLong, obj.tmpInertiaLong];
            end 
            if obj.simOutput.Ir
                obj.simAuxResults.inertiaRot = [obj.simAuxResults.inertiaRot, obj.tmpInertiaRot];
            end
            if obj.simOutput.Delta
                obj.simAuxResults.flightPathAngle = [obj.simAuxResults.flightPathAngle, obj.tmpDelta];
            end
            
            if obj.simOutput.Nose_Alpha
                obj.simAuxResults.noseAlpha = [obj.simAuxResults.noseAlpha, obj.tmpNoseAlpha];
            end
            if obj.simOutput.Nose_Delta
                obj.simAuxResults.noseDelta = [obj.simAuxResults.noseDelta, obj.tmpNoseDelta];
            end
            
        end
        
    end
    
    function status = CrashOutputFunc(obj, thrust,S,flag)

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