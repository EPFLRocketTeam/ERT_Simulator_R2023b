function Rocket = motor2RocketReader(motorFilePath, Rocket)
% MOTOR2ROCKETREADER reads the information contained in the RASP formatted
% motor text file named 'motorFilePath' into the 'Rocket' structure.
if( Rocket.isHybrid == 0)
[time, thrust, Info] = motorReader(motorFilePath);

% motor info
Rocket.motorDiameter = Info{2}/1000;
Rocket.motorLength = Info{3}/1000;
Rocket.motorDelay = Info{4}{1};
Rocket.propelMass = Info{5};
Rocket.motorMass = Info{6};
Rocket.casingMass = Rocket.motorMass-Rocket.propelMass;

% thrust lookup table
if time(1)>0
   time = [0, time];
   thrust = [0, thrust];
elseif time(1)<0
   error('ERROR: in motor2RocketReader, thrust curve only allows positive time values'); 
end
Rocket.thrustTime = time;
Rocket.thrustForce = thrust;

% Burn time
Rocket.burnTime = time(end);

% mass variation coefficient
A_T = trapz(time,thrust);
Rocket.Thrust2dMass_Ratio = Rocket.propelMass/A_T;
else
    
[time, ThrustP, InfoP] = motorReader(motorFilePath);
[timeF,ThrustF,InfoF] = motorReader(Rocket.motorId);


% prop bloc info
Rocket.motorDiameterPropel = InfoP{2}/1000;
Rocket.motorLengthPropel = InfoP{3}/1000;
Rocket.motorDelayPropel = InfoP{4}{1};
Rocket.massPropel = InfoP{5};
Rocket.motorMassPropel = InfoP{6};
Rocket.casingMassPropel = Rocket.motorMassPropel-Rocket.massPropel;

% fuel info
Rocket.motorDiameterFuel = InfoF{2}/1000;
Rocket.motorLengthFuel = InfoF{3}/1000;
Rocket.motorDelayFuel = InfoF{4}{1};
Rocket.massFuel = InfoF{5};
Rocket.motorMassFuel = InfoF{6};
Rocket.casingMassFuel = Rocket.motorMassFuel-Rocket.massFuel;

%Global info
Rocket.motorDiameter = max(Rocket.motorDiameterPropel, Rocket.motorDiameterFuel);
Rocket.motorLength = Rocket.motorLengthPropel + Rocket.motorLengthFuel + Rocket.interMotorDistance ;
Rocket.motorDelay = InfoP{4}{1};
Rocket.propelMass = Rocket.massPropel + Rocket.massFuel ;
Rocket.motorMass = Rocket.motorMassPropel + Rocket.motorMassFuel; 
Rocket.casingMass = Rocket.casingMassPropel + Rocket.casingMassFuel ;

        
    
    
% thrust lookup table
if time(1)>0
   time = [0, time];
   ThrustP = [0, ThrustP];
elseif time(1)<0
   error('ERROR: in motor2RocketReader, thrust curve only allows positive time values'); 
end
Rocket.thrustTime = time;
Rocket.thrustForce = ThrustP;

% Burn time
Rocket.burnTime = time(end);

% mass variation coefficient
A_T = trapz(time,ThrustP);
Rocket.Thrust2dMass_Ratio = Rocket.propelMass/(A_T);    
end
end