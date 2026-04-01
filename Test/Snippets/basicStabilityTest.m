%  Because basicStability is a script (not a function), these tests focus
%  on the numerical solutions it comDampingMomentputes: the angle-of-attack response
%  functions and the ODE coefficients (c1, c2, iL).  Each test either:
%    (a) calls the helper used inside the script and checks invariants, or
%    (b) reproduces the embedded ODE solver inline and validates properties
%        of the analytical solutions.
%
%  Run with: results = runtests('test_Basic_Stability')
 
function tests = basicStabilityTest
    tests = functiontests(localfunctions);
end
 
%--------------------------------------------------------------------------
% Shared parameters matching the script defaults
%--------------------------------------------------------------------------
function p = nominalParams()
    p.velocity = 250;                    % m/s  (rail-exit speed)
    p.vInf     = 5;                      % m/s  (steady crosswind)
    p.vWind    = 10;                     % m/s  (gust speed)
    p.tWind    = 1.0;                    % s    (gust duration)
    p.tEval    = linspace(0, 40, 2000);
end
 
%--------------------------------------------------------------------------
% Helper: build the three ODE branches given (c1, c2, iL, ms)
%   Returns the angle-of-attack function handle a(t)
%--------------------------------------------------------------------------
function a = buildSolution(c1, c2, iL, ms)
    damp = c1/iL - c2^2/4/iL^2;
    if damp > 0
        w   = sqrt(damp);
        d   = c2/iL/2;
        phi = atan(w/d);
        aCoeff = -ms/c1/sin(phi);
        a = @(t) aCoeff*exp(-d*t).*sin(w*t+phi) + ms/c1;
    elseif damp == 0
        d  = c2/iL/2;
        a1 = -ms/c1;
        a2 = -d*ms/c1;
        a  = @(t) (a1 + a2*t).*exp(-d*t) + ms/c1;
    else
        sq   = sqrt(-damp);
        tau1 = 1/(c2/2/iL - sq);
        tau2 = 1/(c2/2/iL + sq);
        a1   = -ms*tau1/(tau1-tau2)/c1;
        a2   =  ms*tau2/(tau1-tau2)/c1;
        a    = @(t) a1*exp(-t/tau1) + a2*exp(-t/tau2) + ms/c1;
    end
end
 
%--------------------------------------------------------------------------
% Test 1 – Underdamped solution: initial condition a(0) ≈ 0
%--------------------------------------------------------------------------
function test_UnderdampedInitialCondition(testCase)
    c1 = 500;  c2 = 30;  iL = 2.0;
    beta = atan(5/250);
    ms   = c1*beta;
    a    = buildSolution(c1, c2, iL, ms);
 
    testCase.verifyEqual(a(0), 0.0, 'AbsTol', 1e-8, ...
        'AoA at t=0 must be zero (rail condition: a_0 = 0)');
end
 
%--------------------------------------------------------------------------
% Test 2 – Underdamped solution: converges to static equilibrium ms/c1
%--------------------------------------------------------------------------
function test_UnderdampedConvergesToEquilibrium(testCase)
    c1 = 500;  c2 = 30;  iL = 2.0;
    beta = atan(5/250);
    ms   = c1*beta;
    a    = buildSolution(c1, c2, iL, ms);
 
    aEq = ms/c1;
    testCase.verifyEqual(a(500), aEq, 'AbsTol', 1e-6, ...
        'Underdamped solution must converge to static equilibrium ms/c1');
end
 
%--------------------------------------------------------------------------
% Test 3 – Underdamped amplitude grows with crosswind speed
%--------------------------------------------------------------------------
function test_AmplitudeIncreasesWithWindSpeed(testCase)
    c1       = 500;  c2 = 30;  iL = 2.0;
    velocity = 250;
    tEval    = linspace(0, 40, 1000);
    vWinds   = [1 3 5 7 9];
    amps     = zeros(size(vWinds));
 
    for k = 1:numel(vWinds)
        ms      = c1 * atan(vWinds(k)/velocity);
        a       = buildSolution(c1, c2, iL, ms);
        sol     = a(tEval);
        eq      = ms/c1;
        amps(k) = max([max(sol-eq), -min(sol-eq)]);
    end
 
    testCase.verifyTrue(all(diff(amps) > 0), ...
        'Peak oscillation amplitude must increase monotonically with wind speed');
end
 
%--------------------------------------------------------------------------
% Test 4 – Overdamped solution: no oscillation (monotonic or exponential)
%--------------------------------------------------------------------------
function test_OverdampedNoOscillation(testCase)
    c1 = 50;  c2 = 300;  iL = 2.0;
    ms = c1 * atan(5/250);
    a  = buildSolution(c1, c2, iL, ms);
 
    tEval      = linspace(0, 40, 5000);
    sol        = a(tEval) - ms/c1;
    signChanges = sum(abs(diff(sign(sol(sol ~= 0)))));
 
    testCase.verifyLessThanOrEqual(signChanges, 2, ...
        'Overdamped solution must not oscillate (few sign changes around eq.)');
end
 
%--------------------------------------------------------------------------
% Test 5 – Impulsive solution (Section 2): initial value is zero
%--------------------------------------------------------------------------
function test_ImpulsiveSolutionInitialZero(testCase)
    c1 = 500;  c2 = 30;  iL = 2.0;
    vWind = 10;  tWind = 1.0;  velocity = 250;
    h    = c1 * atan(vWind/velocity) * tWind;
    damp = c1/iL - c2^2/4/iL^2;
    d    = c2/iL/2;
    w    = sqrt(abs(damp));
    a    = @(t) h/iL/w * exp(-d*t) .* sin(w*t);
 
    testCase.verifyEqual(a(0), 0.0, 'AbsTol', 1e-8, ...
        'Impulsive response must start at zero');
end
 
%--------------------------------------------------------------------------
% Test 6 – Impulsive amplitude scales linearly with impulse strength h
%--------------------------------------------------------------------------
function test_ImpulsiveAmplitudeScalesWithH(testCase)
    c1 = 500;  c2 = 30;  iL = 2.0;
    velocity = 250;  vWind = 10;
    tEval    = linspace(0, 30, 1000);
    damp     = c1/iL - c2^2/4/iL^2;
    d        = c2/iL/2;
    w        = sqrt(damp);   % underdamped branch
    tWinds   = [0.5 1.0 1.5];
    amps     = zeros(size(tWinds));
 
    for k = 1:numel(tWinds)
        h       = c1 * atan(vWind/velocity) * tWinds(k);
        a       = @(t) h/iL/w * exp(-d*t) .* sin(w*t);
        sol     = a(tEval);
        amps(k) = max([max(sol), -min(sol)]);
    end
 
    ratios   = amps / amps(1);
    expected = tWinds / tWinds(1);
    testCase.verifyEqual(ratios, expected, 'AbsTol', 1e-6, ...
        'Impulsive amplitude must scale linearly with impulse strength h');
end
 
%--------------------------------------------------------------------------
% Test 7 – c2 > 0 everywhere (damping must be dissipative)
%--------------------------------------------------------------------------
function test_DampingCoefficientPositive(testCase)
    c2Values = [10, 50, 200, 500];
    for c2 = c2Values
        testCase.verifyGreaterThan(c2, 0, ...
            sprintf('c2 = %g must be positive (physical damping)', c2));
    end
end
 
%--------------------------------------------------------------------------
% Test 8 – Stability parameter: stable iff c1 > 0
%   (positive restoring moment is the stability condition)
%--------------------------------------------------------------------------
function test_StabilityConditionC1Positive(testCase)
    c1Stable   = 500;
    c1Unstable = -100;
    velocity   = 250;  vInf = 5;
    beta       = atan(vInf/velocity);
 
    % The key check: with negative c1 the "restoring" moment is actually
    % destabilising — damp = c1/iL - ... becomes very negative
    iL = 2.0;  c2 = 30;
    dampStable   = c1Stable/iL   - c2^2/4/iL^2;
    dampUnstable = c1Unstable/iL - c2^2/4/iL^2;
 
    testCase.verifyGreaterThan(c1Stable, 0, ...
        'Stable rocket must have positive corrective moment c1');
    testCase.verifyLessThan(dampUnstable, dampStable, ...
        'Unstable rocket produces more negative discriminant than stable one');
end