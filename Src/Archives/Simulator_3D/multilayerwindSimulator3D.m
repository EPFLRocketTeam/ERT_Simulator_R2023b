classdef multilayerwindSimulator3D < handle
    
% -------------------------------------------------------------------------  
% Class properties
% -------------------------------------------------------------------------
   properties(Access = public)
      simAuxResults; 
   end

   properties(Access = public)
      Rocket
      Environment
      SimOutput  
   end
   
   properties(Access = private)
      firstSimFlag = 1;
      tmp_Margin
      tmp_Alpha
      tmp_Cn_alpha
      tmpCenterOfPressure
      tmpDragCoefficient
      tmpMass
      tmpCenterOfMass
      tmpInertiaLong
      tmpInertiaRot
      tmp_Delta
      
      tmpNoseAlpha
      tmpNoseDelta
   end
   
% -------------------------------------------------------------------------  
% Constructor  
% -------------------------------------------------------------------------   
   methods
       
       function obj = multilayerwindSimulator3D(Rocket, Environment, SimOutput)
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
           obj.simAuxResults.Margin = [];
           obj.simAuxResults.Alpha = [];
           obj.simAuxResults.Cn_alpha = [];
           obj.simAuxResults.Xcp = [];
           obj.simAuxResults.Cd = [];
           obj.simAuxResults.Mass = [];
           obj.simAuxResults.CM = [];
           obj.simAuxResults.Il = [];
           obj.simAuxResults.Ir = [];
           obj.simAuxResults.Delta = [];
           
           obj.simAuxResults.Nose_Alpha = [];
           obj.simAuxResults.Nose_Delta = [];
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
            [Mass,dMdt] = Mass_Non_Lin(t,obj.Rocket); % mass

            % Environment
            g = 9.81;               % Gravity [m/s2] 
            [~, a, ~, density, Nu] = stdAtmos(x*sin(obj.Environment.railAngle),obj.Environment); % Atmosphere information (TODO: Include effect of humidity and departure altitude)

            % Force estimation

            % gravity
            G = -g*cos(obj.Environment.railAngle)*Mass;

            % Thrust 
            T = Thrust(t,obj.Rocket); % (TODO: Allow for thrust vectoring -> error)

            % drag
            CD = drag(obj.Rocket, 0, v,Nu, a); % (TODO: make air-viscosity adaptable to temperature)
            D = -0.5*density*obj.Rocket.maxCrossSectionArea*CD*v^2; % (TODO: define drag in wind coordinate system)

            F_tot = G + T*obj.Rocket.motorThrustFactor + D;

            % State derivatives

            x_dot = v;
            v_dot = 1/Mass*(F_tot - v*dMdt);

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
            C = quat2rotmat(Q);
            angle = rot2anglemat(C);

            % Rocket principle frame vectors expressed in earth coordinates
            YA = C*[1,0,0]'; % Yaw axis
            PA = C*[0,1,0]'; % Pitch Axis
            RA = C*[0,0,1]'; % Roll Axis

            % Earth coordinate vectors expressed in earth's frame
            XE = [1, 0, 0]';
            YE = [0, 1, 0]';
            ZE = [0, 0, 1]';

            % Rocket Inertia
            [M,dMdt,Cm,~,I_L,~,I_R,~] = Mass_Properties(t,obj.Rocket,'NonLinear');
            I = C'*diag([I_L, I_L, I_R])*C; % Inertia TODO: I_R in Mass_Properties

            % Environment
            g = 9.81;               % Gravity [m/s2] 
            [~, a, ~, density, nu] = stdAtmos(X(3)+obj.Environment.startAltitude,...
                obj.Environment); % Atmosphere information 

            % Force estimations 

            % Thrust
            % Oriented along roll axis of rocket frame, expressed in earth coordinates. 
            T = Thrust(t,obj.Rocket)*RA; % (TODO: Allow for thrust vectoring -> error)

            % Gravity
            G = -g*M*ZE;

            % Aerodynamic corrective forces
            % Compute center of mass angle of attack
            alt = min(400, max(1,round(X(3)/10)));
            V_inf = obj.Environment.Vspeed(alt)*[obj.Environment.Vdirx(alt);obj.Environment.Vdiry(alt);obj.Environment.Vdirz(alt)];
            Vcm = V -...
                     ... % Wind as computed by windmodel
                V_inf;

            Vcm_mag = norm(Vcm);
            alpha_cm = atan2(norm(cross(RA, Vcm)), dot(RA, Vcm));

            % Mach number
            Mach = Vcm_mag/a;
            % Normal lift coefficient and center of pressure
            [CNa, Xcp,CNa_bar,CP_bar] = normalLift(obj.Rocket, alpha_cm, 1.1,...
                Mach, angle(3), 1);
            % Stability margin
            margin = (Xcp-Cm);

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
            Vcross_norm = normalizeVect(Vcross);
            alpha = atan2(norm(cross(RA, Vnorm)), dot(RA, Vnorm));
            delta = atan2(norm(cross(RA, ZE)), dot(RA, ZE));

            % wind coordinate transformation
%             if(abs(alpha)<1e-3)
%                 RW = RA;
%             elseif(abs(alpha-pi)<1e-3)
%                 RW = -RA;
%             else
%                 Cw = quat2rotmat([Vcross_norm*sin(alpha/2); cos(alpha/2)]);
%                 RW = C*Cw*[0;0;1];
%             end

            % normal force
            NA = cross(RA, Vcross); % normal axis
            if norm(NA) == 0
                N = [0, 0, 0]'; 
            else
                N = 0.5*density*obj.Rocket.maxCrossSectionArea*CNa*alpha*Vmag^2*NA/norm(NA);
            end

            % Drag
            % Drag coefficient
            CD = drag(obj.Rocket, alpha, Vmag, nu, a)*obj.Rocket.dragCoefficientFactor; 
            if(t>obj.Rocket.Burn_Time)
              CD = CD + drag_shuriken(obj.Rocket, obj.Rocket.airbrakeAngle, alpha, Vmag, nu); 
            end
            % Drag force
            D = -0.5*density*obj.Rocket.maxCrossSectionArea*CD*Vmag^2*Vnorm; 

            % Total forces
            F_tot = ...
                T*obj.Rocket.motorThrustFactor +...  ;% Thrust
                G +...  ;% gravity
                N +... ;% normal force
                D      ; % drag force

            % Moment estimation

            %Aerodynamic corrective moment
            MN = norm(N)*margin*Vcross_norm; 

            % Aerodynamic damping moment
            W_pitch = W - dot(W,RA)*RA; % extract pitch and yaw angular velocity
            CDM = pitchDampingMoment(obj.Rocket, density, CNa_bar, CP_bar, ...
                dMdt, Cm, norm(W_pitch) , Vmag); 
            MD = -0.5*density*CDM*obj.Rocket.maxCrossSectionArea*Vmag^2*normalizeVect(W_pitch);

            M_tot = ...
                MN...  ; % aerodynamic corrective moment
               + MD ; % aerodynamic damping moment

            % State derivatives

            % Translational dynamics
            X_dot = V;
            V_dot = 1/M*(F_tot - V*dMdt);

            % Rotational dynamics
            Q_dot = quat_evolve(Q, W);
            W_dot = pinv(I)*(M_tot); % (TODO: Add inertia variation with time)

            % Return derivative vector
            S_dot = [X_dot;V_dot;Q_dot;W_dot];
            
            % cache auxiliary result data
            obj.tmp_Margin = margin/obj.Rocket.maxDiameter;
            obj.tmp_Alpha = alpha;
            obj.tmp_Cn_alpha = CNa;
            obj.tmpCenterOfPressure = Xcp;
            obj.tmpDragCoefficient = CD;
            obj.tmpMass = M;
            obj.tmpCenterOfMass = Cm;
            obj.tmpInertiaLong = I_L;
            obj.tmpInertiaRot = I_R;
            obj.tmp_Delta = delta;
        end
        
        % --------------------------- 
        % 3DOF Parachute descent Equations
        % ---------------------------
        
        function dsdt = Dynamics_Parachute_3DOF(obj, t,s, Rocket, Environment, M, Main)

            X = s(1:3);
            V = s(4:6);

            % Atmospheric Data
            [~, ~, ~, density] = stdAtmos(X(3)+Environment.startAltitude, Environment); % Atmosphere [K,m/s,Pa,kg/m3]

            % Aerodynamic force
            alt = min(400, max(1,round(X(3)/10)));
            V_inf = Environment.Vspeed(alt)*[Environment.Vdirx(alt);Environment.Vdiry(alt);Environment.Vdirz(alt)];
            Vrel = -V + ...
                 ... % Wind as computed by windmodel
                V_inf;

            if Main
                SCD = Rocket.mainParachuteDragArea;
            elseif Main == 0
                SCD = Rocket.drogueParachuteDragArea;
            end
            D = 0.5*density*SCD*norm(Vrel)*Vrel;

            % Gravity force
            g = 9.81*[0;0;-1];
            G = g*M;

            dXdt = V;
            dVdt = (D+G)/M;

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
            [~, a, ~, density, nu] = stdAtmos(X(3)+Environment.startAltitude, Environment);

            % mass
            M = Rocket.emptyMass;
            alt = min(400, max(1,round(X(3)/10)));
            V_inf = Environment.Vspeed(alt)*[Environment.Vdirx(alt);Environment.Vdiry(alt);Environment.Vdirz(alt)];
            V_rel = V -...
                 ... % Wind as computed by windmodel
                V_inf;

            % gravity
            % Gravity
            G = -9.81*M*ZE;
            % Drag
            % Drag coefficient
            CD = drag(Rocket, 0, norm(V_rel), nu, a); % (TODO: make air-viscosity adaptable to temperature)
            % Drag force
            D = -0.5*density*Rocket.maxCrossSectionArea*CD*V_rel*norm(V_rel); 

            % Translational dynamics
            X_dot = V;
            V_dot = 1/M*(D + G);

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
            C = quat2rotmat(Q);
            angle = rot2anglemat(C);

            % Rocket principle frame vectors expressed in earth coordinates
            YA = C*[1,0,0]'; % Yaw axis
            PA = C*[0,1,0]'; % Pitch Axis
            RA = C*[0,0,1]'; % Roll Axis

            % Earth coordinate vectors expressed in earth's frame
            XE = [1, 0, 0]';
            YE = [0, 1, 0]';
            ZE = [0, 0, 1]';

            % Rocket Inertia
            [M,dMdt,Cm,~,I_L,~,I_R,~] = Mass_Properties(t,obj.Rocket,'NonLinear');
            I = C'*diag([I_L, I_L, I_R])*C; % Inertia TODO: I_R in Mass_Properties

            % Environment
            g = 9.81;               % Gravity [m/s2]
            [~, a, ~, density, nu] = stdAtmos(X(3)+obj.Environment.startAltitude,...
                obj.Environment); % Atmosphere information 

            % Force estimations 

            % Thrust
            % Oriented along roll axis of rocket frame, expressed in earth coordinates. 
            T = Thrust(t,obj.Rocket)*RA; % (TODO: Allow for thrust vectoring -> error)

            % Gravity
            G = -g*M*ZE;

            % Aerodynamic corrective forces
            % Compute center of mass angle of attack
            alt = min(4000, max(1,round(X(3))));
            V_inf = obj.Environment.Vspeed(alt)*obj.Environment.Vdir(alt);
            Vcm = V -...
                     ... % Wind as computed by windmodel
            V_inf;

            Vcm_mag = norm(Vcm);
            alpha_cm = atan2(norm(cross(RA, Vcm)), dot(RA, Vcm));

            % Mach number
            Mach = Vcm_mag/a;
            % Normal lift coefficient and center of pressure
            [CNa, Xcp,CNa_bar,CP_bar] = normalLift(obj.Rocket, alpha_cm, 1.1,...
                Mach, angle(3), 1);
            % Stability margin
            margin = (Xcp-Cm);

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
            Vcross_norm = normalizeVect(Vcross);
            alpha = atan2(norm(cross(RA, Vnorm)), dot(RA, Vnorm));
            delta = atan2(norm(cross(RA, ZE)), dot(RA, ZE));

            % wind coordinate transformation
%             if(abs(alpha)<1e-3)
%                 RW = RA;
%             elseif(abs(alpha-pi)<1e-3)
%                 RW = -RA;
%             else
%                 Cw = quat2rotmat([Vcross_norm*sin(alpha/2); cos(alpha/2)]);
%                 RW = C*Cw*[0;0;1];
%             end

            % normal force
            NA = cross(RA, Vcross); % normal axis
            if norm(NA) == 0
                N = [0, 0, 0]'; 
            else
                N = 0.5*density*obj.Rocket.maxCrossSectionArea*CNa*alpha*Vmag^2*NA/norm(NA);
            end

            % Drag
            % Drag coefficient
            CD = Nose_drag(obj.Rocket, alpha, Vmag, nu, a)*obj.Rocket.dragCoefficientFactor; 
            if(t>obj.Rocket.Burn_Time)
              CD = CD + drag_shuriken(obj.Rocket, obj.Rocket.airbrakeAngle, alpha, Vmag, nu); 
            end
            % Drag force
            D = -0.5*density*obj.Rocket.maxCrossSectionArea*CD*Vmag^2*Vnorm;

            % Total forces
            F_tot = ...
                T*obj.Rocket.motorThrustFactor +...  ;% Thrust
                G +...  ;% gravity
                N +... ;% normal force
                D      ; % drag force

            % Moment estimation

            %Aerodynamic corrective moment
            MN = norm(N)*margin*Vcross_norm;

            % Aerodynamic damping moment
            W_pitch = W - dot(W,RA)*RA; % extract pitch and yaw angular velocity
            CDM = pitchDampingMoment(obj.Rocket, density, CNa_bar, CP_bar, ...
                dMdt, Cm, norm(W_pitch) , Vmag); 
            MD = -0.5*density*CDM*obj.Rocket.maxCrossSectionArea*Vmag^2*normalizeVect(W_pitch);

            M_tot = ...
                MN...  ; % aerodynamic corrective moment
               + MD ; % aerodynamic damping moment

            % State derivatives

            % Translational dynamics
            X_dot = V;
            V_dot = 1/M*(F_tot - V*dMdt);

            % Rotational dynamics
            Q_dot = quat_evolve(Q, W);
            W_dot = pinv(I)*(M_tot); % (TODO: Add inertia variation with time)

            % Return derivative vector
            S_dot = [X_dot;V_dot;Q_dot;W_dot];
            
            % cache auxiliary result data
            obj.tmpNoseAlpha = alpha;
            obj.tmpNoseDelta = delta;
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
            X0 = [0,0]'; % positioned at 0 height and 0 velocity

            % time span 
            tspan = [0, 5];

            % options
            Option = odeset('Events', @(t,x) RailEvent(t,x,obj.Environment));

            % integration
            [railTime,railState] = ode45(@(t,x) obj.Dynamics_Rail_1DOF(t,x),tspan,X0, Option); 
            
        end
        
        % --------------------------- 
        % Flight Simulation
        % ---------------------------
        function [flightTime, flightState, T2E, S2E, I2E] = FlightSim(obj, tspan, arg2, arg3, arg4, arg5)
            
            if (nargin == 3)
                % Compute initial conditions based on rail output values
                V = arg2;
                
                % Rail vector
                C_rail = rotmat(obj.Environment.railAzimuth, 3)*...
                    rotmat(obj.Environment.railAngle, 2)*...
                    rotmat(obj.Environment.railAzimuth, 3)';
                RV = C_rail*[0;0;1];

                % Initial Conditions
                X0 = RV*obj.Environment.railLength; % spatial position of cm
                V0 = RV*V; % Initial velocity of cm
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
            Option = odeset('Events', @ApogeeEvent, 'RelTol', 1e-6, 'AbsTol', 1e-6,...
                            'OutputFcn', @(T,S,flag) obj.FlightOutputFunc(T,S,flag),...
                            'Refine', 1);

            % integration
            [flightTime,flightState, T2E, S2E, I2E] = ode45(@(t,s) obj.Dynamics_6DOF(t,s),tspan,S0, Option);
%             figure
%             plot3(flightState(:,1),flightState(:,2),flightState(:,3),'*r');
%             grid on
            
        end
        
        
        % --------------------------- 
        % Drogue Parachute Simulation
        % ---------------------------
        function [drogueTime, drogueState, drogueTimeEvents, drogueStateEvents, drogueEventIndices] = DrogueParaSim(obj, T0, X0, V0)
            
            % initial conditions
            S0 = [X0; V0];

            % empty mass
            M = obj.Rocket.emptyMass - obj.Rocket.payloadMass;

            % time span
            tspan = [T0, 500];
            %T0
            

            % options 
            Option = odeset('Events', @(T,X) MainEvent(T,X,obj.Rocket));

            % integration
            [drogueTime,drogueState, drogueTimeEvents, drogueStateEvents, drogueEventIndices] = ode45(@(t,s) obj.Dynamics_Parachute_3DOF(t,s,obj.Rocket,obj.Environment, M, 0),tspan,S0, Option);
%             figure
%             plot3(drogueState(:,1),drogueState(:,2),drogueState(:,3),'*r');
%             grid on
%             hold on
%             plot3(X0(1),X0(2),X0(3),'b+');
%             hold off
        end
        
        % --------------------------- 
        % Main Parachute Simulation
        % ---------------------------
        function [mainChuteTime, mainChuteState, mainChuteTimeEvents, S4E, mainChuteEventsIndices] = MainParaSim(obj, T0, X0, V0)
            
            % initial conditions
            S0 = [X0; V0];
            
            % empty mass
            M = obj.Rocket.emptyMass - obj.Rocket.payloadMass;

            % time span
            tspan = [T0, 500];
            

            % options 
            Option = odeset('Events',@(T,X) CrashEvent(T,X,obj.Environment));
           
            % integration
            [mainChuteTime, mainChuteState, mainChuteTimeEvents, S4E, mainChuteEventsIndices] = ode45(@(t,s) obj.Dynamics_Parachute_3DOF(t,s,obj.Rocket,obj.Environment, M, 1),tspan,S0, Option);
            
        end
            % --------------------------- 
        % Crash Simulation
        % ---------------------------
        function [crashTime, crashState, crashTimeEvents, crashStateEvents, crashEventIndices] = CrashSim(obj, T0, X0, V0)
            
            % Initial Conditions
            S0 = [X0; V0];

            % time span
            tspan = [T0, 100];

            % options
            Option = odeset('Events',@(T,X) CrashEvent(T,X,obj.Environment));

            % integration
            [crashTime,crashState, crashTimeEvents, crashStateEvents, crashEventIndices] = ode45(@(t,s) obj.Dynamics_3DOF(t,s,obj.Rocket,obj.Environment),tspan,S0, Option);

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
                obj.simAuxResults.Margin = [obj.simAuxResults.Margin, obj.tmp_Margin];
            end 
            if obj.SimOutput.Alpha
                obj.simAuxResults.Alpha = [obj.simAuxResults.Alpha, obj.tmp_Alpha];
            end 
            if obj.SimOutput.Cn_alpha
                obj.simAuxResults.Cn_alpha = [obj.simAuxResults.Cn_alpha, obj.tmp_Cn_alpha];
            end 
            if obj.SimOutput.Xcp
                obj.simAuxResults.Xcp = [obj.simAuxResults.Xcp, obj.tmpCenterOfPressure];
            end 
            if obj.SimOutput.Cd
                obj.simAuxResults.Cd = [obj.simAuxResults.Cd, obj.tmpDragCoefficient];
            end 
            if obj.SimOutput.Mass
                obj.simAuxResults.Mass = [obj.simAuxResults.Mass, obj.tmpMass];
            end 
            if obj.SimOutput.CM
                obj.simAuxResults.CM = [obj.simAuxResults.CM, obj.tmpCenterOfMass];
            end 
            if obj.SimOutput.Il
                obj.simAuxResults.Il = [obj.simAuxResults.Il, obj.tmpInertiaLong];
            end 
            if obj.SimOutput.Ir
                obj.simAuxResults.Ir = [obj.simAuxResults.Ir, obj.tmpInertiaRot];
            end
            if obj.SimOutput.Delta
                obj.simAuxResults.Delta = [obj.simAuxResults.Delta, obj.tmp_Delta];
            end
            
            if obj.SimOutput.Nose_Alpha
                obj.simAuxResults.Nose_Alpha = [obj.simAuxResults.Nose_Alpha, obj.tmpNoseAlpha];
            end
            if obj.SimOutput.Nose_Delta
                obj.simAuxResults.Nose_Delta = [obj.simAuxResults.Nose_Delta, obj.tmpNoseDelta];
            end
            
        end
        
    end
    
    function status = CrashOutputFunc(obj, T,S,flag)

        % keep simulation running
        status = 0;

        if isempty(flag) || (strcmp(flag, 'init') && obj.firstSimFlag)

            obj.firstSimFlag = 0;
            if obj.SimOutput.Nose_Alpha
                obj.simAuxResults.Nose_Alpha = [obj.simAuxResults.Nose_Alpha, obj.tmpNoseAlpha];
            end
            if obj.SimOutput.Nose_Delta
                obj.simAuxResults.Nose_Delta = [obj.simAuxResults.Nose_Delta, obj.tmpNoseDelta];
            end
            
        end
        
    end
end
end