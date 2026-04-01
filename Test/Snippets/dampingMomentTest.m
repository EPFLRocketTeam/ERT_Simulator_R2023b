%  Tests the total damping coefficient c2 = cr2 + ca2 where:
%    cr2 = dMdt * (L - cm)^2                          (thrust damping)
%    ca2 = rho * v * sm/2 * sum(cAlpha*(cp-cm)^2)     (aerodynamic damping)
%
%  Run with: results = runtests('test_DampingMoment')
 
function tests = dampingMomentTest
    tests = functiontests(localfunctions);
end
 
%--------------------------------------------------------------------------
% Fixtures
%--------------------------------------------------------------------------
function rocket = buildRocket()
    rocket.Sm        = 0.02;   % Reference area [m^2]
    rocket.L         = 3.0;    % Total rocket length [m]
    rocket.Burn_Time = 5.0;
end
 
function env = buildEnvironnement()
    env.P0 = 101325;
    env.T0 = 288.15;
    env.L  = 0.0065;
    env.R  = 287.05;
    env.g  = 9.81;
end
 
%--------------------------------------------------------------------------
% Test 1 – c2 is strictly positive under nominal conditions
%--------------------------------------------------------------------------
function test_NominalPositiveC2(testCase)
    rocket   = buildRocket();
    env      = buildEnvironnement();
    cAlpha   = [3.0; 4.0];   % Per-fin lift slopes
    cp       = [2.5; 2.6];   % Centre-of-pressure locations [m]
    velocity = 250;
 
    c2 = dampingMoment(0, rocket, cAlpha, cp, velocity, env, 0);
 
    testCase.verifyGreaterThan(c2, 0, ...
        'Total damping c2 must be positive under nominal conditions');
end
 
%--------------------------------------------------------------------------
% Test 2 – Output is a real scalar
%--------------------------------------------------------------------------
function test_OutputIsRealScalar(testCase)
    rocket = buildRocket();
    env    = buildEnvironnement();
    cAlpha = [5.0];
    cp     = [2.2];
 
    c2 = dampingMoment(0, rocket, cAlpha, cp, 200, env, 0);
 
    testCase.verifySize(c2, [1 1], 'c2 must be a scalar');
    testCase.verifyTrue(isreal(c2), 'c2 must be real-valued');
end
 
%--------------------------------------------------------------------------
% Test 3 – Aerodynamic term scales linearly with velocity
%   ca2 = rho*v*sm/2 * sum(cAlpha*(cp-cm)^2)
%   If thrust damping cr2 is negligible (dMdt ≈ 0), c2 ∝ v.
%   We verify: c2(2v) / c2(v) ≈ 2  (at t after burnout, dMdt=0)
%--------------------------------------------------------------------------
function test_AeroDampingLinearInVelocity(testCase)
    rocket           = buildRocket();
    rocket.Burn_Time = 0;   % Immediately past burnout → dMdt = 0
    env              = buildEnvironnement();
    cAlpha           = [6.0];
    cp               = [2.3];
    tPostburn        = rocket.Burn_Time + 1;   % ensure post-burnout
 
    c2V1 = dampingMoment(tPostburn, rocket, cAlpha, cp, 100, env, 0);
    c2V2 = dampingMoment(tPostburn, rocket, cAlpha, cp, 200, env, 0);
 
    testCase.verifyEqual(c2V2 / c2V1, 2.0, 'AbsTol', 1e-4, ...
        'Aero damping should scale linearly with velocity (post-burnout)');
end
 
%--------------------------------------------------------------------------
% Test 4 – Increasing sm increases c2
%--------------------------------------------------------------------------
function test_LargerReferenceAreaIncreasesC2(testCase)
    rocket1    = buildRocket(); rocket1.Sm = 0.01;
    rocket2    = buildRocket(); rocket2.Sm = 0.04;
    env        = buildEnvironnement();
    cAlpha     = [4.0];
    cp         = [2.4];
    velocity   = 200;
 
    c2Small = dampingMoment(0, rocket1, cAlpha, cp, velocity, env, 0);
    c2Large = dampingMoment(0, rocket2, cAlpha, cp, velocity, env, 0);
 
    testCase.verifyGreaterThan(c2Large, c2Small, ...
        'Larger reference area must produce greater damping');
end
 
%--------------------------------------------------------------------------
% Test 5 – Higher altitude reduces aerodynamic damping (lower rho)
%--------------------------------------------------------------------------
function test_HigherAltitudeReducesC2(testCase)
    rocket   = buildRocket();
    env      = buildEnvironnement();
    cAlpha   = [5.0];
    cp       = [2.3];
    velocity = 250;
 
    c2Sea  = dampingMoment(0, rocket, cAlpha, cp, velocity, env, 0);
    c2High = dampingMoment(0, rocket, cAlpha, cp, velocity, env, 5000);
 
    testCase.verifyGreaterThan(c2Sea, c2High, ...
        'c2 should decrease at higher altitude due to lower air density');
end
 
%--------------------------------------------------------------------------
% Test 6 – Zero cAlpha gives only thrust contribution
%   With cAlpha = 0, ca2 = 0, so c2 = cr2 = dMdt*(L-cm)^2.
%   We check c2 >= 0 and that it is independent of velocity.
%--------------------------------------------------------------------------
function test_ZeroCalphaOnlyThrustDamping(testCase)
    rocket = buildRocket();
    env    = buildEnvironnement();
    cAlpha = 0;
    cp     = 2.2;
 
    c2V1 = dampingMoment(0, rocket, cAlpha, cp, 100, env, 0);
    c2V2 = dampingMoment(0, rocket, cAlpha, cp, 400, env, 0);
 
    testCase.verifyGreaterThanOrEqual(c2V1, 0, ...
        'Thrust damping must be non-negative');
    testCase.verifyEqual(c2V1, c2V2, 'AbsTol', 1e-10, ...
        'With zero cAlpha, c2 must be velocity-independent (pure thrust term)');
end
 
%--------------------------------------------------------------------------
% Test 7 – Multiple fins: sum over all cAlpha contributions
%   c2 with two fins of equal cAlpha/cp must equal twice the single-fin value
%   (post-burnout so only the aero term remains).
%--------------------------------------------------------------------------
function test_MultipleFinsSumCorrectly(testCase)
    rocket           = buildRocket();
    rocket.Burn_Time = 0;
    env              = buildEnvironnement();
    cpVal            = 2.5;
    cAlphaVal        = 3.0;
    velocity         = 200;
    tPost            = 1.0;
 
    c2One = dampingMoment(tPost, rocket, cAlphaVal,               cpVal,             velocity, env, 0);
    c2Two = dampingMoment(tPost, rocket, [cAlphaVal; cAlphaVal],  [cpVal; cpVal],    velocity, env, 0);
 
    testCase.verifyEqual(c2Two, 2*c2One, 'AbsTol', 1e-8, ...
        'Two identical fins must contribute double the damping of one fin');
end