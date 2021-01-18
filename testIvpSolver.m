%% Test script for the ivpSolver function
%
% Tests just the IVP solver part of the problem and plots the resulting
% (approximately parabolic) trajectory. Written as an intermediate step and
% left here for reference, though it had to be modified slightly since then
% because of changes to stateDerivProjectile.
%
% Also useful for testing the modified state derivative function which
% takes wind velocity into account.

%% Constants
m = 0.5; % Mass of projectile (kg)
rho = 1.225; % Density of air (kg/m^3)
cd = 0.1; % Drag coefficient
g = 9.81; % Acceleration due to gravity (m/s^2) at Earth's surface
A = 5e-4; % Cross-sectional area of projectile (m^2)
dt = 0.01; % Timestep (s)
wind = [0, 0]; % Wind velocity vector (m/s)
% Note that the wind velocity only affects the projectile; it is assumed
% that the drone can maintain its position and velocity regardless of
% the wind (or in the context of this program, that the drone velocity is
% relative to the ground and is constant, regardless of wind). It would be
% near-impossible to predict the effect of wind on the drone since its drag
% coefficient and cross-sectional area are not known.

%% Trajectory calculation

% State derivative function handle (see testShootingMethod line 25 for
% an explanation) for motion of projectile
f = @(t1, z1) stateDerivProjectileWind(t1, z1, m, rho, cd, g, A, wind);

% Computes the trajectory of the projectile, starting at (0, 1) with a
% launch angle of approximately 30 degrees and a launch velocity of
% approximately 50 m/s.
[t, z] = ivpSolver(0, [0; 43; 1; 25], dt, 10, f, true);

% Plots the trajectory
plot(z(1, :), z(3, :));

%% Basic animation test script
% for n = 1:length(t)
%     plot(z(1, n), z(3, n),'MarkerFaceColor',[0 0 0],'MarkerEdgeColor','none',...
%     'MarkerSize',4,...
%     'Marker','o');
%     %hold on
%     xlim([0, 400]);
%     axis equal;
%     ylim([-150, 150]);
%     xlabel('x (m)');
%     ylabel('y (m)');
%     title(strcat('t = ', num2str(t(n), '%03f')));
%     pause(dt);
% end