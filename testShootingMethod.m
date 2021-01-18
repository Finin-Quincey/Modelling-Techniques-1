%% Solution to the basic assignment
%
% Script to test the shootingMethod function and calculate the collision
% between the projectile and drone and the subsequent path of the two objects
% until they hit the ground. Outputs various calculated values and plots
% the full trajectory of the projectile.

%% Initial conditions
v0 = 50; % Launch velocity (m/s)
droneCoords = [50, 30]; % Drone x and y coordinates (m)
testAngles = [20, 60]; % First and second guesses for the angle (degrees)
useRK4 = true; % True to use the Runge-Kutta 4 method, false to use Euler

%% Extension conditions
% These values aren't used in the basic problem, but the shootingMethod
% function still requires them to be passed in - they will work though!
droneVel = [0, 0]; % Drone x and y velocity (m/s) relative to ground
% Wind has been omitted from this script for simplicity.

%% Constants
m = 0.5; % Mass of projectile (kg)
M = 0.5; % Mass of drone (kg)
rho = 1.225; % Density of air (kg/m^3)
cd = 0.1; % Drag coefficient
cd2 = 0.9; % New drag coefficient
g = 9.81; % Acceleration due to gravity (m/s^2) at Earth's surface
A = 5e-4; % Cross-sectional area of projectile (m^2)
A2 = 1.5; % Parachute cross-sectional area (m^2)
dt = 0.01; % Timestep (s)

tic; % Starts the timer

%% Shooting method

% Anonymous function which represents the state derivative, with specific
% values for the various constants. This format allows me to keep ivpSolver
% and the step methods completely generic (so they could be used for any
% ivp whatsoever) without having to copy-paste stateDerivProjectile itself.
% The arguments inside the @(...) become the inputs to the anonymous
% function, whereas the rest are substituted in now.
% See http://uk.mathworks.com/help/optim/ug/passing-extra-parameters.html
f = @(t1, z1) stateDerivProjectile(t1, z1, m, rho, cd, g, A);
% (I think t1 and z1 are local to the scope of this function, but I've
% named them differently to t and z to avoid confusion.)

% Calculates the initial trajectory of the projectile, up to where it hits
% the drone.
[launchAngle, t, z, n, iterations] = shootingMethod(0, 1, v0, 10, droneCoords, droneVel, testAngles, 1, dt, f, useRK4);

% Outputs the number of iterations to the console. %i prints an integer, \n
% starts a new line.
fprintf('Solved in %i iterations\n', iterations);

% Outputs the calculated launch angle to the console. %.4g prints a decimal
% number to a maximum of 4 significant figures, with no trailing zeros.
fprintf('Computed launch angle of %.4g degrees\n', launchAngle);

%% Interception and collision

% Outputs the calculated time to reach the drone to the console
fprintf('Successfully intercepted drone at t = %.4g seconds\n', t(n));

% Retrieves the velocity vector of the projectile just as it intercepts the drone
v = [z(2, n), z(4, n)];

% Calculates the velocity of the projectile and drone after the collision
% using conservation of momentum. The drone is stationary beforehand.
v = collideAndCoalesce(m, v, M, [0, 0]);

%% Descent with parachute

% Switches out the constants for the state derivative function
% Using anonymous functions allows this to be done very concisely
f = @(t1, z1) stateDerivProjectile(t1, z1, m+M, rho, cd2, g, A2);

% Calculates the path of the projectile after the collision. The
% conditions after the collision are now the initial conditions.
[t2, z2] = ivpSolver(t(n), [z(1, n), v(1), z(3, n), v(2)], dt, 20, f, useRK4);

% Initialise loop counter; outside of loop since it will be needed later on
% Again, not actually necessary
%m = 0;

% Finds the index at which y first becomes less than 0
for m = 1:length(z2(3, :))
    if z2(3, m) <= 0
        % Outputs the calculated landing coordinates and flight time to the console
        fprintf('Captured drone landed at a distance of %.4gm after %.4g seconds\n', z2(1, m), t(m));
        % Outputs the calculated impact speed to the console
        fprintf('Calculated impact speed of %.4g m/s\n', sqrt(z2(2, m)^2 + z2(4, m)^2));
        break; % Found it so stop incrementing m
    end
end

% Outputs the computation time to the console
fprintf('Total computation time: %.4g seconds\n', toc);

%% Plotting

% Combines the two paths
t(n+1:n+m) = t2(1:m);
z(:, n+1:n+m) = z2(:, 1:m);

% Plots the resulting trajectory
plot(z(1, :), z(3, :));
hold on;
grid on;

% Plots the drone position
plot(droneCoords(1), droneCoords(2), 'k', 'Marker', 'x', 'MarkerSize', 6);

% Makes the scale nice
xlim([0, round(max(z(1, :)*1.2), -1)]);
axis equal;
ylim([0, round(max(z(3, :)*1.2), -1)]);

% Labels the axes
xlabel('x (m)');
ylabel('y (m)');