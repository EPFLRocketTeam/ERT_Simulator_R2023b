classdef Simulator3D_DragOR < handle
    
% -------------------------------------------------------------------------  
% Class properties
% -------------------------------------------------------------------------
   properties(Access = public)
      SimAuxResults; 
   end

   properties(Access = public)
      Rocket
      Environment
      Drag
      interp_type
      SimOutput  
   end
   
   properties(Access = private)
      firstSimFlag = 1;
      tmp_Margin
      tmp_Alpha
      tmp_Cn_alpha
      tmp_Xcp
      tmp_Cd
      tmp_Mass
      tmp_CM
      tmp_Il
      tmp_Ir
      tmp_Delta
      
      tmp_Nose_Alpha
      tmp_Nose_Delta
   end
   
% -------------------------------------------------------------------------  
% Constructor  
% -------------------------------------------------------------------------   
   methods
       
       function obj = Simulator3D(Rocket, Environment, Drag, interp_type, SimOutput)
           if nargin == 0
               % TODO: Put default values or send warning message
           elseif nargin == 5
               obj.Rocket = Rocket;
               obj.Environment = Environment;
               
               %Extract drag coefficients computed by OR
               dragOR = readtable(Drag,'Format','%s%s%s%s');

               Index_start = find(contains(dragOR{:,1},'# Event LIFTOFF'));
               Index_end = find(contains(dragOR{:,1},'# Event APOGEE'));

               dragOR = dragOR{Index_start:Index_end,:};

               removeIndex = find(contains(dragOR(:,1),"Event"));
               dragOR(removeIndex,:) = [];
               dragOR = str2double(dragOR);
               obj.Drag = dragOR;
               
               obj.interp_type = interp_type;
               obj.SimOutput = SimOutput;
           else
               error(['ERROR: In Simulator3D constructor, either no arguments '...
                   'or 5 arguments can be given. You gave ' num2str(nargin) '.']);
           end

           % Initialise Auxiliary results structure
           obj.SimAuxResults.Margin = [];
           obj.SimAuxResults.Alpha = [];
           obj.SimAuxResults.Cn_alpha = [];
           obj.SimAuxResults.Xcp = [];
           obj.SimAuxResults.Cd = [];
           obj.SimAuxResults.Mass = [];
           obj.SimAuxResults.CM = [];
           obj.SimAuxResults.Il = [];
           obj.SimAuxResults.Ir = [];
           obj.SimAuxResults.Delta = [];
           
           obj.SimAuxResults.Nose_Alpha = [];
           obj.SimAuxResults.Nose_Delta = [];
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
            [~, a, ~, rho, Nu] = atmosphere(x*sin(obj.Environment.Rail_Angle),obj.Environment); % Atmosphere information (TODO: Include effect of humidity and departure altitude)

            % Force estimation

            % gravity
            G = -g*cos(obj.Environment.Rail_Angle)*Mass;

            % Thrust 
            T = Thrust(t,obj.Rocket); % (TODO: Allow for thrust vectoring -> error)

            % drag
            CD = drag(obj.Drag, obj.interp_type, t, x, v); % (TODO: make air-viscosity adaptable to temperature)
            D = -0.5*rho*obj.Rocket.Sm*CD*v^2; % (TODO: define drag in wind coordinate system)

            F_tot = G + T*obj.Rocket.motor_fac + D;

            % State derivatives
            
            x_dot = v;
            v_dot = 1/Mass*(F_tot);
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
            %I = C'*diag([I_L, I_L, I_R])*C; % Inertia TODO: I_R in Mass_Properties
            
            % Inertia using the given I_rocket and the motor
            % Compute I_motor (approximate by a cylinder)
            motor_inertia = inertia_fill_cylinder(M, ...
                obj.Rocket.motor_length, obj.Rocket.motor_dia / 2);
            % Total inertia
            %I = inertial_matrix(obj.Rocket, Cm, t);
            %disp(I)
            I = obj.Rocket.rocket_inertia + motor_inertia;
            %disp(I)
            %disp("==============================")
            I = C' * I * C; % Transfert to earth coordinates

            % Temporal derivative of inertial matrix
            dIdt = inertia_fill_cylinder(dMdt, obj.Rocket.motor_length, ...
                obj.Rocket.motor_dia / 2); % Inertial matrix time derivative
            dIdt = C' * dIdt * C; % Transfert to earth coordinates

            % Environment
            g = 9.81;               % Gravity [m/s2] 
            [~, a, ~, rho, nu] = atmosphere(X(3)+obj.Environment.Start_Altitude,...
                obj.Environment); % Atmosphere information 

            % Force estimations 

            % Thrust
            % Oriented along roll axis of rocket frame, expressed in earth coordinates. 
            T = Thrust(t,obj.Rocket)*RA; % (TODO: Allow for thrust vectoring -> error)

            % Gravity
            G = -g*M*ZE;

            % Aerodynamic corrective forces
            % Compute center of mass angle of attack
            Vcm = V -...
                     ... % Wind as computed by windmodel
                windModel(t, obj.Environment.Turb_I,obj.Environment.V_inf*obj.Environment.V_dir,...
                obj.Environment.Turb_model,X(3)); 

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
                N = 0.5*rho*obj.Rocket.Sm*CNa*alpha*Vmag^2*NA/norm(NA);
            end
            % Drag
            % Drag coefficient
            CD = drag(obj.Drag, obj.interp_type, t, X(3), V(3))*obj.Rocket.CD_fac; 
            if(t>obj.Rocket.Burn_Time)
              CD = CD + drag_shuriken(obj.Rocket, obj.Rocket.ab_phi, alpha, Vmag, nu); 
            end
            % Drag force
            D = -0.5*rho*obj.Rocket.Sm*CD*Vmag^2*Vnorm ;

            % Total forces
            F_tot = ...
                T*obj.Rocket.motor_fac +...  ;% Thrust
                G +...  ;% gravity
                N +... ;% normal force
                D      ; % drag force

            % Moment estimation

            %Aerodynamic corrective moment
            MN = norm(N)*margin*Vcross_norm ;

            % Aerodynamic damping moment
            W_pitch = W - dot(W,RA)*RA; % extract pitch and yaw angular velocity
            CDM = pitchDampingMoment(obj.Rocket, rho, CNa_bar, CP_bar, ...
                dMdt, Cm, norm(W_pitch) , Vmag); 
            
            MD = -0.5*rho*CDM*obj.Rocket.Sm*Vmag^2*normalizeVect(W_pitch);
            
            M_tot = ...
                MN...  ; % aerodynamic corrective moment
               + MD; % aerodynamic damping moment

            % State derivatives

            % Translational dynamics
            X_dot = V;
            V_dot = 1/M*(F_tot);

            % Rotational dynamics
            Q_dot = quat_evolve(Q, W);
            
            %W_dot = pinv(I)*M_tot;
            %W_dot = mldivide(I,M_tot); % (TODO: Add inertia variation with time)
            W_dot = mldivide(I, M_tot - dIdt*angle');

            % Return derivative vector
            S_dot = [X_dot;V_dot;Q_dot;W_dot];
            
            % cache auxiliary result data
            obj.tmp_Margin = margin/obj.Rocket.dm;
            obj.tmp_Alpha = alpha;
            obj.tmp_Cn_alpha = CNa;
            obj.tmp_Xcp = Xcp;
            obj.tmp_Cd = CD;
            obj.tmp_Mass = M;
            obj.tmp_CM = Cm;
            obj.tmp_Il = I_L;
            obj.tmp_Ir = I_R;
            obj.tmp_Delta = delta;
        end
        
        % --------------------------- 
        % 3DOF Parachute descent Equations
        % ---------------------------
        
        function dsdt = Dynamics_Parachute_3DOF(obj, t,s, Rocket, Environment, M, Main)

            X = s(1:3);
            V = s(4:6);

            % Atmospheric Data
            [~, ~, ~, rho] = atmosphere(X(3)+Environment.Start_Altitude, Environment); % Atmosphere [K,m/s,Pa,kg/m3]

            % Aerodynamic force
            Vrel = -V + ...
                 ... % Wind as computed by windmodel
                windModel(t, Environment.Turb_I,Environment.V_inf*Environment.V_dir,...
                Environment.Turb_model,X(3));

            if Main
                SCD = Rocket.para_main_SCD;
            elseif Main == 0
                SCD = Rocket.para_drogue_SCD;
            end
            D = 0.5*rho*SCD*norm(Vrel)*Vrel;

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
            [~, a, ~, rho, nu] = atmosphere(X(3)+Environment.Start_Altitude, Environment);

            % mass
            M = Rocket.rocket_m;

            V_rel = V -...
                 ... % Wind as computed by windmodel
                windModel(t, Environment.Turb_I,Environment.V_inf*Environment.V_dir,...
                Environment.Turb_model,X(3));

            % gravity
            % Gravity
            G = -9.81*M*ZE;
            % Drag
            % Drag coefficient
            CD = drag(obj.Drag, obj.interp_type, t, X(3), V(3)); % (TODO: make air-viscosity adaptable to temperature)
            % Drag force
            D = -0.5*rho*Rocket.Sm*CD*V_rel*norm(V_rel); 
            % Translational dynamics
            X_dot = V;
            V_dot = 1/M*(D + G);

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
            [~, a, ~, rho, nu] = atmosphere(X(3)+Environment.Start_Altitude, Environment);

            % mass
            M = Rocket.rocket_m;

            V_rel = V -...
                 ... % Wind as computed by windmodel
                windModel(t, Environment.Turb_I,Environment.V_inf*Environment.V_dir,...
                Environment.Turb_model,X(3));

            % gravity
            % Gravity
            G = -9.81*M*ZE;
            % Drag
            % Drag coefficient
            CD = Nose_drag(Rocket, 0, norm(V_rel), nu, a); % (TODO: make air-viscosity adaptable to temperature)
            % Drag force
            D = -0.5*rho*Rocket.Sm*CD*V_rel*norm(V_rel); 

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
            %I = C'*diag([I_L, I_L, I_R])*C; % Inertia TODO: I_R in Mass_Properties

            % Inertia using the given I_rocket and the motor
            % Compute I_motor (approximate by a cylinder)
            motor_inertia = inertia_fill_cylinder(M, ...
                obj.Rocket.motor_length, obj.Rocket.motor_dia / 2);
            % Total inertia
            I = obj.Rocket.rocket_inertia + motor_inertia;
            I = C' * I * C; % Transfert to earth coordinates

            % Temporal derivative of inertial matrix
            dIdt = inertia_fill_cylinder(dMdt, obj.Rocket.motor_length, ...
                obj.Rocket.motor_dia / 2); % Inertial matrix time derivative
            dIdt = C' * dIdt * C; % Transfert to earth coordinates

            % Environment
            g = 9.81;               % Gravity [m/s2]
            [~, a, ~, rho, nu] = atmosphere(X(3)+obj.Environment.Start_Altitude,...
                obj.Environment); % Atmosphere information 

            % Force estimations 

            % Thrust
            % Oriented along roll axis of rocket frame, expressed in earth coordinates. 
            T = Thrust(t,obj.Rocket)*RA; % (TODO: Allow for thrust vectoring -> error)

            % Gravity
            G = -g*M*ZE;

            % Aerodynamic corrective forces
            % Compute center of mass angle of attack
            Vcm = V -...
                     ... % Wind as computed by windmodel
                windModel(t, obj.Environment.Turb_I,obj.Environment.V_inf*obj.Environment.V_dir,...
                obj.Environment.Turb_model,X(3)); 

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
                N = 0.5*rho*obj.Rocket.Sm*CNa*alpha*Vmag^2*NA/norm(NA);
            end

            % Drag
            % Drag coefficient
            CD = Nose_drag(obj.Rocket, alpha, Vmag, nu, a)*obj.Rocket.CD_fac; 
            if(t>obj.Rocket.Burn_Time)
              CD = CD + drag_shuriken(obj.Rocket, obj.Rocket.ab_phi, alpha, Vmag, nu); 
            end
            % Drag force
            D = -0.5*rho*obj.Rocket.Sm*CD*Vmag^2*Vnorm;

            % Total forces
            F_tot = ...
                T*obj.Rocket.motor_fac +...  ;% Thrust
                G +...  ;% gravity
                N +... ;% normal force
                D      ; % drag force

            % Moment estimation

            %Aerodynamic corrective moment
            MN = norm(N)*margin*Vcross_norm;

            % Aerodynamic damping moment
            W_pitch = W - dot(W,RA)*RA; % extract pitch and yaw angular velocity
            CDM = pitchDampingMoment(obj.Rocket, rho, CNa_bar, CP_bar, ...
                dMdt, Cm, norm(W_pitch) , Vmag); 
            MD = -0.5*rho*CDM*obj.Rocket.Sm*Vmag^2*normalizeVect(W_pitch);

            M_tot = ...
                MN...  ; % aerodynamic corrective moment
               + MD ; % aerodynamic damping moment

            % State derivatives

            % Translational dynamics
            X_dot = V;
            V_dot = 1/M*(F_tot);

            % Rotational dynamics
            Q_dot = quat_evolve(Q, W);
            %W_dot = mldivide(I,M_tot); % (TODO: Add inertia variation with time)
            W_dot = mldivide(I, M_tot - dIdt*angle');

            % Return derivative vector
            S_dot = [X_dot;V_dot;Q_dot;W_dot];
            
            % cache auxiliary result data
            obj.tmp_Nose_Alpha = alpha;
            obj.tmp_Nose_Delta = delta;
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
            [~, a, ~, rho, nu] = atmosphere(X(3)+Environment.Start_Altitude, Environment);

            % mass
            M = Rocket.pl_mass;

            V_rel = V -...
                 ... % Wind as computed by windmodel
                windModel(t, Environment.Turb_I,Environment.V_inf*Environment.V_dir,...
                Environment.Turb_model);

            % gravity
            % Gravity
            G = -9.81*M*ZE;
            % Drag
            % Drag coefficient
            SCD = 2.56e-2; 
            % Drag force
            D = -0.5*rho*SCD*V_rel*norm(V_rel); 

            % Translational dynamics
            X_dot = V;
            V_dot = 1/M*(D + G);

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
                C_rail = rotmat(obj.Environment.Rail_Azimuth, 3)*...
                    rotmat(obj.Environment.Rail_Angle, 2)*...
                    rotmat(obj.Environment.Rail_Azimuth, 3)';
                RV = C_rail*[0;0;1];

                % Initial Conditions
                X0 = RV*obj.Environment.Rail_Length; % spatial position of cm
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
            [T2,S2, T2E, S2E, I2E] = ode45(@(t,s) obj.Dynamics_6DOF(t,s),tspan,S0, Option);
            
        end
        
        
        % --------------------------- 
        % Drogue Parachute Simulation
        % ---------------------------
        function [T3, S3, T3E, S3E, I3E] = DrogueParaSim(obj, T0, X0, V0)
            
            % initial conditions
            S0 = [X0; V0];

            % empty mass
            M = obj.Rocket.rocket_m - obj.Rocket.pl_mass;

            % time span
            tspan = [T0, 5000];

            % options 
            Option = odeset('Events', @(T,X) MainEvent(T,X,obj.Rocket));

            % integration
            [T3,S3, T3E, S3E, I3E] = ode45(@(t,s) obj.Dynamics_Parachute_3DOF(t,s,obj.Rocket,obj.Environment, M, 0),tspan,S0, Option);
        
        end
        
        % --------------------------- 
        % Main Parachute Simulation
        % ---------------------------
        function [T4, S4, T4E, S4E, I4E] = MainParaSim(obj, T0, X0, V0)
            
            % initial conditions
            S0 = [X0; V0];

            % empty mass
            M = obj.Rocket.rocket_m - obj.Rocket.pl_mass;

            % time span
            tspan = [T0, 5000];

            % options 
            Option = odeset('Events',@(T,X) CrashEvent(T,X,obj.Environment));

            % integration
            [T4, S4, T4E, S4E, I4E] = ode45(@(t,s) obj.Dynamics_Parachute_3DOF(t,s,obj.Rocket,obj.Environment, M, 1),tspan,S0, Option);
            
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
                obj.SimAuxResults.Margin = [obj.SimAuxResults.Margin, obj.tmp_Margin];
            end 
            if obj.SimOutput.Alpha
                obj.SimAuxResults.Alpha = [obj.SimAuxResults.Alpha, obj.tmp_Alpha];
            end 
            if obj.SimOutput.Cn_alpha
                obj.SimAuxResults.Cn_alpha = [obj.SimAuxResults.Cn_alpha, obj.tmp_Cn_alpha];
            end 
            if obj.SimOutput.Xcp
                obj.SimAuxResults.Xcp = [obj.SimAuxResults.Xcp, obj.tmp_Xcp];
            end 
            if obj.SimOutput.Cd
                obj.SimAuxResults.Cd = [obj.SimAuxResults.Cd, obj.tmp_Cd];
            end 
            if obj.SimOutput.Mass
                obj.SimAuxResults.Mass = [obj.SimAuxResults.Mass, obj.tmp_Mass];
            end 
            if obj.SimOutput.CM
                obj.SimAuxResults.CM = [obj.SimAuxResults.CM, obj.tmp_CM];
            end 
            if obj.SimOutput.Il
                obj.SimAuxResults.Il = [obj.SimAuxResults.Il, obj.tmp_Il];
            end 
            if obj.SimOutput.Ir
                obj.SimAuxResults.Ir = [obj.SimAuxResults.Ir, obj.tmp_Ir];
            end
            if obj.SimOutput.Delta
                obj.SimAuxResults.Delta = [obj.SimAuxResults.Delta, obj.tmp_Delta];
            end
            
            if obj.SimOutput.Nose_Alpha
                obj.SimAuxResults.Nose_Alpha = [obj.SimAuxResults.Nose_Alpha, obj.tmp_Nose_Alpha];
            end
            if obj.SimOutput.Nose_Delta
                obj.SimAuxResults.Nose_Delta = [obj.SimAuxResults.Nose_Delta, obj.tmp_Nose_Delta];
            end
            
        end
        
    end
    
    function status = CrashOutputFunc(obj, T,S,flag)

        % keep simulation running
        status = 0;

        if isempty(flag) || (strcmp(flag, 'init') && obj.firstSimFlag)

            obj.firstSimFlag = 0;
            if obj.SimOutput.Nose_Alpha
                obj.SimAuxResults.Nose_Alpha = [obj.SimAuxResults.Nose_Alpha, obj.tmp_Nose_Alpha];
            end
            if obj.SimOutput.Nose_Delta
                obj.SimAuxResults.Nose_Delta = [obj.SimAuxResults.Nose_Delta, obj.tmp_Nose_Delta];
            end
            
        end
        
    end
end
end