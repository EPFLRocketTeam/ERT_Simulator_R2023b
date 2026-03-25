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
      tmpNormalForceCoefficientSlope
      tmpCenterOfPressure
      tmpDragCoefficient
      tmpMass
      tmpCenterOfMass
      tmpInertiaLong
      tmpInertiaRot
      tmpFlightPathAngle
      
      tmpNoseAngleOfAttack
      tmpNoseFlightPathAngle
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
           obj.simAuxResults.normalForceCoefficientSlope = [];
           obj.simAuxResults.centerOfPressure = [];
           obj.simAuxResults.dragCoefficient = [];
           obj.simAuxResults.mass = [];
           obj.simAuxResults.centerOfMass = [];
           obj.simAuxResults.inertiaLong = [];
           obj.simAuxResults.inertiaRot = [];
           obj.simAuxResults.flightPathAngle = [];
           
           obj.simAuxResults.noseAngleOfAttack = [];
           obj.simAuxResults.noseFlightPathAngle = [];
       end
       
   end
     
% -------------------------------------------------------------------------  
% Dynamic Computation methods & Output functions
% -------------------------------------------------------------------------      
   methods(Access = protected)
       
        % --------------------------- 
        % Rail equations 
        % ---------------------------
    
        function stateDerivative = Dynamics_Rail_1DOF(obj, time, state)

            x = state(1); % position
            v = state(2); % speed

            % rocket Inertia
            [mass, massRate] = Mass_Non_Lin(time,obj.rocket); % mass

            % Environment
            g = 9.81;  % Gravity [m/s2] 
            [~, speedOfSound, ~, density, kinematicViscosity] = atmosphere(x*sin(obj.environment.railAngle), obj.environment); % Atmosphere information

            % Force estimation

            % gravity
            gravityForce = -g*cos(obj.environment.railAngle)*mass;

            % Thrust 
            thrust = Thrust(time,obj.rocket); % (TODO: Allow for thrust vectoring -> error)

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
            end 

            if velocityDot < 0
                positionDot = 0;
                velocityDot = 0;
            end 

            stateDerivative = [positionDot; velocityDot];
        end
       
        % --------------------------- 
        % 6DOF Flight Equations
        % ---------------------------
        function stateDerivative = Dynamics_6DOF(obj, time, state)

            position = state(1:3);
            velocity = state(4:6);
            quaternion = state(7:10);
            angularVelocity = state(11:13);

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

            % Earth coordinate vectors expressed in earth'state frame
            xEarth = [1, 0, 0]';
            yEarth = [0, 1, 0]';
            zEarth = [0, 0, 1]';

            % rocket Inertia
            [mass,massRate,centerOfMass,~,inertiaLong,~,inertiaRot,~] = Mass_Properties(time,obj.rocket,'NonLinear');
            %inertiaMatrix = rotationMatrix'*diag([inertiaLong, inertiaLong, inertiaRot])*rotationMatrix; % Inertia TODO: inertiaRot in Mass_Properties
            
            % Inertia using the given I_rocket and the motor
            % Compute I_motor (approximate by speedOfSound cylinder)
            motorInertia = inertia_fill_cylinder(mass, ...
                obj.rocket.motor_length, obj.rocket.motor_dia / 2);
            % Total inertia
            %inertiaMatrix = inertial_matrix(obj.rocket, centerOfMass, time);
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
            thrust = Thrust(time,obj.rocket)*rollAxis; % (TODO: Allow for thrust vectoring -> error)

            % Gravity
            gravityForce = -g*mass*zEarth;

            % Aerodynamic corrective forces
            % Compute center of mass eulerAngles of attack
            centerOfMassVelocity = velocity -...
                     ... % Wind as computed by windmodel
                windModel(time, obj.environment.Turb_I,obj.environment.V_inf*obj.environment.V_dir,...
                obj.environment.Turb_model,position(3)); 

            centerOfMassSpeed = norm(centerOfMassVelocity);
            centerOfMassAngleOfAttack = atan2(norm(cross(rollAxis, centerOfMassVelocity)), dot(rollAxis, centerOfMassVelocity));

            % machNumber number
            machNumber = centerOfMassSpeed/speedOfSound;
            % Normal lift coefficient and center of pressure
            [normalForceCoefficient, centerOfPressure,normalForceCoefficientBar,centerOfPressureBar] = normalLift(obj.rocket, centerOfMassAngleOfAttack, 1.1,...
                machNumber, eulerAngles(3), 1);
            
            
            % Stability stabilityMargin
            stabilityMargin = (centerOfPressure-centerOfMass);

            % Compute rocket eulerAngles of attack
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
            if(time>obj.rocket.Burn_Time)
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
            stateDerivative = [dx;dv;dQuaternion;dAngularVelocity];
            
            % cache auxiliary result data
            obj.tmpStabilityMargin = stabilityMargin/obj.rocket.maxDiameter;
            obj.tmpAngleOfAttack = angleOfAttack;
            obj.tmpNormalForceCoefficientSlope = normalForceCoefficient;
            obj.tmpCenterOfPressure = centerOfPressure;
            obj.tmpDragCoefficient = dragCoefficient;
            obj.tmpMass = mass;
            obj.tmpCenterOfMass = centerOfMass;
            obj.tmpInertiaLong = inertiaLong;
            obj.tmpInertiaRot = inertiaRot;
            obj.tmpFlightPathAngle = flightPathAngle;
        end

        function stateDerivative = Dynamics_6DOF2(obj, time, state)

            position = state(1:3);
            velocity = state(4:6);
            quaternion = state(7:10);
            angularVelocity = state(11:13);

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

            % Earth coordinate vectors expressed in earth'state frame
            xEarth = [1, 0, 0]';
            yEarth = [0, 1, 0]';
            zEarth = [0, 0, 1]';

            % rocket Inertia
            [mass, massRate, centerOfMass, ~, inertiaLong, ~, inertiaRot, ~] = Mass_Properties(time,obj.rocket,'NonLinear');
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
            thrust = Thrust(time,obj.rocket)*rollAxis; % (TODO: Allow for thrust vectoring -> error)

            % Gravity
            gravityForce = -g*mass*zEarth;

            % Aerodynamic corrective forces
            % Compute center of mass angleOfAttack of attack
            centerOfMassVelocity = velocity -...
                     ... % Wind as computed by windmodel
                windModel(time, obj.environment.Turb_I,obj.environment.V_inf*obj.environment.V_dir,...
                obj.environment.Turb_model,position(3)); 

            centerOfMassSpeed = norm(centerOfMassVelocity);
            centerOfMassAngleOfAttack = atan2(norm(cross(rollAxis, centerOfMassVelocity)), dot(rollAxis, centerOfMassVelocity));

            % machNumber number
            machNumber = centerOfMassSpeed/speedOfSound;

            % Normal lift coefficient and center of pressure
            [normalForceCoefficient, centerOfPressure, normalForceCoefficientBar, ...
                centerOfPressureBar] = normalLift(obj.rocket, centerOfMassAngleOfAttack, 1.1,...
                machNumber, angleOfAttack(3), 1);
            
            
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
            dragCoefficient = drag(obj.rocket, angleOfAttack, relativeSpeed, kinematicViscosity, speedOfSound)*obj.rocket.dragCoefficientFactor; 
            if(time>obj.rocket.Burn_Time)
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
            stateDerivative = [positionDot;velocityDot;quaternionDot;angularVelocityDot];
            
            % cache auxiliary result data
            obj.tmpStabilityMargin = stabilityMargin/obj.rocket.maxDiameter;
            obj.tmpAngleOfAttack = angleOfAttack;
            obj.tmpNormalForceCoefficientSlope = normalForceCoefficient;
            obj.tmpCenterOfPressure = centerOfPressure;
            obj.tmpDragCoefficient = dragCoefficient;
            obj.tmpMass = mass;
            obj.tmpCenterOfMass =  centerOfMass;
            obj.tmpInertiaLong = inertiaLong;
            obj.tmpInertiaRot = inertiaRot;
            obj.tmpFlightPathAngle = flightPathAngle;
        end
        
        % --------------------------- 
        % 3DOF Parachute descent Equations
        % ---------------------------
        
        function dsdt = Dynamics_Parachute_3DOF(obj, time, state, rocket, environment, mass, main)

            position = state(1:3);
            velocity = state(4:6);

            % Atmospheric Data
            [~, ~, ~, density] = atmosphere(position(3)+environment.startAltitude, environment); % Atmosphere [K,m/state,Pa,kg/m3]

            % Aerodynamic force
            relativeVelocity = -velocity + ...
                 ... % Wind as computed by windmodel
                windModel(time, environment.Turb_I,environment.V_inf*environment.V_dir,...
                environment.Turb_model,position(3));

            if main
                dragArea = rocket.mainParachuteDragArea;
            elseif main == 0
                dragArea = rocket.drogueParachuteDragArea;
            end
            dragForce = 0.5*density*dragArea*norm(relativeVelocity)*relativeVelocity;

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
        
        function stateDerivative = Dynamics_3DOF(obj, time, state, rocket, environment)

            position = state(1:3);
            velocity = state(4:6);

            % Earth coordinate vectors expressed in earth'state frame
            xEarth = [1, 0, 0]';
            yEarth = [0, 1, 0]';
            zEarth = [0, 0, 1]';

            % atmosphere
            [~, speedOfSound, ~, density, kinematicViscosity] = atmosphere(position(3)+environment.startAltitude, environment);

            % mass
            mass = rocket.emptyMass;

            relativeVelocity = velocity -...
                 ... % Wind as computed by windmodel
                windModel(time, environment.Turb_I,environment.V_inf*environment.V_dir,...
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

            stateDerivative = [positionDot; velocityDot];

        end
        
        % --------------------------- 
        % 3DOF Nosecone Crash descent Equations
        % ---------------------------
    
        function stateDerivative = Nose_Dynamics_3DOF(obj, time, state, rocket, environment)

            position = state(1:3);
            velocity = state(4:6);

            % Earth coordinate vectors expressed in earth'state frame
            xEarth = [1, 0, 0]';
            yEarth = [0, 1, 0]';
            zEarth = [0, 0, 1]';

            % atmosphere
            [~, speedOfSound, ~, density, kinematicViscosity] = atmosphere(position(3)+environment.startAltitude, environment);

            % mass
            mass = rocket.emptyMass;

            relativeVelocity = velocity -...
                 ... % Wind as computed by windmodel
                windModel(time, environment.Turb_I,environment.V_inf*environment.V_dir,...
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

            stateDerivative = [positionDot; velocityDot];

        end
        
        % --------------------------- 
        % 6DOF Nosecone Crash descent Equations
        % ---------------------------
        
        function stateDerivative = Nose_Dynamics_6DOF(obj, time, state)

            position = state(1:3);
            velocity = state(4:6);
            quaternion = state(7:10);
            angularVelocity = state(11:13);

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

            % Earth coordinate vectors expressed in earth'state frame
            xEarth = [1, 0, 0]';
            yEarth = [0, 1, 0]';
            zEarth = [0, 0, 1]';

            % rocket Inertia
            [mass,massRate, centerOfMass,~,inertiaLong,~,inertiaRot,~] = Mass_Properties(time,obj.rocket,'NonLinear');
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
            thrust = Thrust(time,obj.rocket)*rollAxis; % (TODO: Allow for thrust vectoring -> error)

            % Gravity
            gravityForce = -g*mass*zEarth;

            % Aerodynamic corrective forces
            % Compute center of mass angleOfAttack of attack
            centerOfMassVelocity = velocity -...
                     ... % Wind as computed by windmodel
                windModel(time, obj.environment.Turb_I,obj.environment.V_inf*obj.environment.V_dir,...
                obj.environment.Turb_model,position(3)); 

            centerOfMassSpeed = norm(centerOfMassVelocity);
            centerOfMassAngleOfAttack = atan2(norm(cross(rollAxis, centerOfMassVelocity)), dot(rollAxis, centerOfMassVelocity));

            % Match number
            machNumber = centerOfMassSpeed/speedOfSound;
            % Normal lift coefficient and center of pressure
            [normalForceCoefficient, centerOfPressure, normalForceCoefficientBar,centerOfPressureBar] = normalLift(obj.rocket, centerOfMassAngleOfAttack, 1.1,...
                machNumber, angleOfAttack(3), 1);
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
            if(time>obj.rocket.Burn_Time)
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
            stateDerivative = [positionDot;velocityDot;quaternionDot;angularVelocityDot];
            
            % cache auxiliary result data
            obj.tmpNoseAngleOfAttack = angleOfAttack;
            obj.tmpNoseFlightPathAngle = flightPathAngle;
        end
        
        % --------------------------- 
        % 3DOF Payload descent Equations
        % ---------------------------
        
        function stateDerivative = Payload_Dynamics_3DOF(obj, time, state, rocket, environment)

            position = state(1:3);
            velocity = state(4:6);

            % Earth coordinate vectors expressed in earth'state frame
            xEarth = [1, 0, 0]';
            yEarth = [0, 1, 0]';
            zEarth = [0, 0, 1]';

            % atmosphere
            [~, speedOfSound, ~, density, kinematicViscosity] = atmosphere(position(3)+environment.startAltitude, environment);

            % mass
            mass = rocket.payloadMass;

            relativeVelocity = velocity -...
                 ... % Wind as computed by windmodel
                windModel(time, environment.Turb_I,environment.V_inf*environment.V_dir,...
                environment.Turb_model);

            % gravity
            % Gravity
            gravityForce = -9.81*mass*zEarth;
            % Drag
            % Drag coefficient
            payloadDragArea = 2.56e-2; 
            % Drag force
            dragForce = -0.5*density*payloadDragArea*relativeVelocity*norm(relativeVelocity); 

            % Translational dynamics
            positionDot = velocity;
            velocityDot = 1/mass*(dragForce + gravityForce);

            stateDerivative = [positionDot; velocityDot];

        end
        
   end     
   
% -------------------------------------------------------------------------  
% Runnable methods
% -------------------------------------------------------------------------           
    methods(Access = public)
        
        % --------------------------- 
        % Rail Simulation
        % ---------------------------
        function [railTime, railState] = RailSim(obj)
            
           % Initial Conditions
            initialPosition = [0,0]'; % positioned at 0 height and 0 velocity

            % time span  
            timeSpan = [0, 5];

            % options
            options = odeset('Events', @(time,x) RailEvent(time,x,obj.environment));

            % integration
            [railTime, railState] = ode45(@(time,x) obj.Dynamics_Rail_1DOF(time, x), timeSpan, initialPosition, options); 
            
        end
        
        % --------------------------- 
        % Flight Simulation
        % ---------------------------
        function [flightTime, flightState, flightTimeEvents, flightStateEvents, flightEventIndices] = FlightSim(obj, tspan, arg2, arg3, arg4, arg5)
            
            if (nargin == 3)
                % Compute initial conditions based on rail output values
                velocity = arg2;
                
                % Rail vector
                railRotation = rotmat(obj.environment.railAzimuth, 3)*...
                    rotmat(obj.environment.railAngle, 2)*...
                    rotmat(obj.environment.railAzimuth, 3)';
                railVector = railRotation*[0;0;1];

                % Initial Conditions
                initialPosition = railVector*obj.environment.railLength; % spatial position of cm
                initialVelocity = railVector*velocity; % Initial velocity of cm
                initialQuaternion = rot2quat(railRotation'); % Initial attitude
                initialAngularVelocity = [0;0;0]; % Initial angular rotation in rocket principle coordinates
                initialState = [initialPosition; initialVelocity; initialQuaternion; initialAngularVelocity];
            elseif (nargin == 6)
                % Set initial conditions based on the exact initial value
                % of the state vector.
                initialPosition = arg2;
                initialVelocity = arg3;
                initialQuaternion = arg4;
                initialAngularVelocity = arg5;
                initialState = [initialPosition; initialVelocity; initialQuaternion; initialAngularVelocity];
            else
               error('ERROR: In Flight Simulator, function accepts either 3 or 6 arguments.') 
            end

            % options
            options = odeset('Events', @ApogeeEvent, 'RelTol', 1e-6, 'AbsTol', 1e-6,...
                            'OutputFcn', @(thrust,state,flag) obj.FlightOutputFunc(thrust,state,flag),...
                            'Refine', 1);

            % integration
            [flightTime, flightState, flightTimeEvents, flightStateEvents, flightEventIndices] = ode45(@(time,state) obj.Dynamics_6DOF(time,state),tspan,initialState, options);
            
        end
                
        % --------------------------- 
        % Drogue Parachute Simulation
        % ---------------------------
        function [drogueTime, drogueState, drogueTimeEvents, drogueStateEvents, drogueEventIndices] = DrogueParaSim(obj, initialTime, initialPosition, initialVelocity)
            
            % initial conditions
            initialState = [initialPosition; initialVelocity];

            % empty mass
            mass = obj.rocket.emptyMass - obj.rocket.payloadMass;

            % time span
            timeSpan = [initialTime, 5000];

            % options 
            options = odeset('Events', @(thrust,position) MainEvent(thrust,position,obj.rocket));

            % integration
            [drogueTime,drogueState, drogueTimeEvents, drogueStateEvents, drogueEventIndices] = ode45(@(time,state) obj.Dynamics_Parachute_3DOF(time,state,obj.rocket,obj.environment, mass, 0),timeSpan,initialState, options);
        
        end
        
        % --------------------------- 
        % main Parachute Simulation
        % ---------------------------
        function [mainChuteTime, mainChuteState, mainChuteTimeEvents, S4E, mainChuteEventsIndices] = MainParaSim(obj, initialTime, initialPosition, initialVelocity)
            
            % initial conditions
            initialState = [initialPosition; initialVelocity];

            % empty mass
            mass = obj.rocket.emptyMass - obj.rocket.payloadMass;

            % time span
            timeSpan = [initialTime, 5000];

            % options 
            options = odeset('Events',@(thrust,position) CrashEvent(thrust,position,obj.environment));

            % integration
            [mainChuteTime, mainChuteState, mainChuteTimeEvents, S4E, mainChuteEventsIndices] = ode45(@(time,state) obj.Dynamics_Parachute_3DOF(time,state,obj.rocket,obj.environment, mass, 1),timeSpan,initialState, options);
            
        end
        
        % --------------------------- 
        % Crash Simulation
        % ---------------------------
        function [crashTime, crashState, crashTimeEvents, crashStateEvents, crashEventIndices] = CrashSim(obj, initialTime, initialPosition, initialVelocity)
            
            % Initial Conditions
            initialState = [initialPosition; initialVelocity];

            % time span
            timeSpan = [initialTime, 100];

            % options
            options = odeset('Events',@(thrust,position) CrashEvent(thrust,position,obj.environment));

            % integration
            [crashTime,crashState, crashTimeEvents, crashStateEvents, crashEventIndices] = ode45(@(time,state) obj.Dynamics_3DOF(time,state,obj.rocket,obj.environment),timeSpan,initialState, options);

        end
        
        % --------------------------- 
        % Nosecone Crash Simulation 3DOF
        % ---------------------------
        function [noseconeCrashTime, noseconeCrashState, noseconeCrashTimeEvents, noseconeCrashStateEvents, noseconeCrashEventIndices] = Nose_CrashSim_3DOF(obj, initialTime, initialPosition, initialVelocity)
            
            % Initial Conditions
            initialState = [initialPosition; initialVelocity];

            % time span
            timeSpan = [initialTime, 1000];

            % options
            options = odeset('Events',@(thrust,position) CrashEvent(thrust,position,obj.environment));

            % integration
            [noseconeCrashTime, noseconeCrashState, noseconeCrashTimeEvents, noseconeCrashStateEvents, noseconeCrashEventIndices] = ode45(@(time,state) obj.Nose_Dynamics_3DOF(time,state,obj.rocket,obj.environment),timeSpan,initialState, options);

        end
        
        % --------------------------- 
        % Nosecone Crash Simulation 6DOF
        % ---------------------------
        function [noseconeCrashTime, noseconeCrashState, noseconeCrashTimeEvents, noseconeCrashStateEvents, noseconeCrashEventIndices] = Nose_CrashSim_6DOF(obj, timeSpan, arg2, arg3, arg4, arg5)
            
            if (nargin == 6)
                % Set initial conditions based on the exact initial value
                % of the state vector.
                initialPosition = arg2;
                initialVelocity = arg3;
                initialQuaternion = arg4;
                initialAngularVelocity = arg5;
                initialState = [initialPosition; initialVelocity; initialQuaternion; initialAngularVelocity];
            else
               error('ERROR: In Flight Simulator, function accepts either 3 or 6 arguments.') 
            end

            % options
            options = odeset('Events', @(thrust,position) CrashEvent(thrust,position,obj.environment),...
                            'OutputFcn', @(thrust,state,flag) obj.CrashOutputFunc(thrust,state,flag),...
                            'Refine', 1);

            % integration
            [noseconeCrashTime, noseconeCrashState, noseconeCrashTimeEvents, noseconeCrashStateEvents, noseconeCrashEventIndices] = ode45(@(time,state) obj.Nose_Dynamics_6DOF(time,state),timeSpan,initialState, options);
            
        end
        
        % --------------------------- 
        % Payload Impact Simulation
        % ---------------------------
        function [payloadCrashTime, payloadCrashState, payloadCrashTimeEvents, payloadCrashStateEvents, payloadCrashEventIndices] = PayloadCrashSim(obj, initialTime, initialPosition, initialVelocity)
            
            % Initial Conditions
            initialState = [initialPosition; initialVelocity];

            % time span
            timeSpan = [initialTime, 1000];

            % options
            options = odeset('Events', @(thrust,position) CrashEvent(thrust,position,obj.environment));

            % integration
            [payloadCrashTime, payloadCrashState, payloadCrashTimeEvents, payloadCrashStateEvents, payloadCrashEventIndices] = ode45(@(time,state) obj.Payload_Dynamics_3DOF(time,state,obj.rocket,obj.environment),timeSpan,initialState, options);

        end
    end
    
% -------------------------------------------------------------------------  
% Private methods
% -------------------------------------------------------------------------  
methods(Access = private)
    function status = FlightOutputFunc(obj, thrust, state, flag)

        % keep simulation running
        status = 0;

        if isempty(flag) || (strcmp(flag, 'init') && obj.firstSimFlag)

            obj.firstSimFlag = 0;
            
            if obj.simOutput.stabilityMargin
                obj.simAuxResults.stabilityMargin = [obj.simAuxResults.stabilityMargin, obj.tmpStabilityMargin];
            end 
            if obj.simOutput.angleOfAttack
                obj.simAuxResults.angleOfAttack = [obj.simAuxResults.angleOfAttack, obj.tmpAngleOfAttack];
            end 
            if obj.simOutput.normalForceCoefficientSlope
                obj.simAuxResults.normalForceCoefficientSlope = [obj.simAuxResults.normalForceCoefficientSlope, obj.tmpNormalForceCoefficientSlope];
            end 
            if obj.simOutput.centerOfPressure
                obj.simAuxResults.centerOfPressure = [obj.simAuxResults.centerOfPressure, obj.tmpCenterOfPressure];
            end 
            if obj.simOutput.dragCoefficient
                obj.simAuxResults.dragCoefficient = [obj.simAuxResults.dragCoefficient, obj.tmpDragCoefficient];
            end 
            if obj.simOutput.mass
                obj.simAuxResults.mass = [obj.simAuxResults.mass, obj.tmpMass];
            end 
            if obj.simOutput.centerOfMass
                obj.simAuxResults.centerOfMass = [obj.simAuxResults.centerOfMass, obj.tmpCenterOfMass];
            end 
            if obj.simOutput.inertiaLong
                obj.simAuxResults.inertiaLong = [obj.simAuxResults.inertiaLong, obj.tmpInertiaLong];
            end 
            if obj.simOutput.inertiaRot
                obj.simAuxResults.inertiaRot = [obj.simAuxResults.inertiaRot, obj.tmpInertiaRot];
            end
            if obj.simOutput.flightPathAngle
                obj.simAuxResults.flightPathAngle = [obj.simAuxResults.flightPathAngle, obj.tmpFlightPathAngle];
            end
            
            if obj.simOutput.noseAngleOfAttack
                obj.simAuxResults.noseAngleOfAttack = [obj.simAuxResults.noseAngleOfAttack, obj.tmpNoseAngleOfAttack];
            end
            if obj.simOutput.noseFlightPathAngle
                obj.simAuxResults.noseFlightPathAngle = [obj.simAuxResults.noseFlightPathAngle, obj.tmpNoseFlightPathAngle];
            end
            
        end
        
    end
    
    function status = CrashOutputFunc(obj, time, state, flag)

        % keep simulation running
        status = 0;

        if isempty(flag) || (strcmp(flag, 'init') && obj.firstSimFlag)

            obj.firstSimFlag = 0;
            if obj.simOutput.noseAngleOfAttack
                obj.simAuxResults.noseAngleOfAttack = [obj.simAuxResults.noseAngleOfAttack, obj.tmpNoseAngleOfAttack];
            end
            if obj.simOutput.noseFlightPathAngle
                obj.simAuxResults.noseFlightPathAngle = [obj.simAuxResults.noseFlightPathAngle, obj.tmpNoseFlightPathAngle];
            end
            
        end
        
    end
end
end