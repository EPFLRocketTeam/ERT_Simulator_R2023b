Rs = linspace(0, 1, 100);

KFB = 0.8065*Rs.^2+1.1553.*Rs; % Interference of body on fin lift
KBF = 0.1935*Rs.^2+0.8174.*Rs+1; % Interference of fins on body lift % WARNING ONLY VALID FOR GIVEN VALUES

figure
hold on
plot(Rs, KFB)
plot(Rs, KBF)
grid on
box on

dalpha_dCL = 0.32;
1/dalpha_dCL