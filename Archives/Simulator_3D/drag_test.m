addpath(genpath('../Declarations'),...
        genpath('../Functions'),...
        genpath('../Snippets'),...
        genpath('../Simulator_1D'));
% Rocket Definition
Rocket = rocketReader('WH_test.txt');
Environment = environnementReader('Environment/Environnement_Definition_Wasserfallen.txt');

U=[0.3:0.02:3]*340;
res=[];
for i=1:size(U,2)
    res(end+1)=drag(Rocket, 0, U(i), 1.789e-5, 340);
end
figure
plot(U/340,res)