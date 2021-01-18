function dz = stateDerivProjectileWind(t, z, m, rho, cd, g, A, w)
% Calculates the state derivative for a projectile with air resistance and
% wind.
% 
%   Input Arguments:
%   t   - Value of time for the current step (not used in this particular
%         case, but to conform to the general contract for derivFunc in ivpSolver
%         and the step methods, it must be included)
%   z   - State vector for the current step (of the format [x, vx, y, vy])
%   m   - Mass of the projectile
%   rho - Density of the fluid through which the projectile is travelling
%   cd  - Drag coefficient of the projectile
%   g   - Acceleration due to gravity
%   A   - Cross-sectional area of the projectile perpendicular to its
%         direction of travel
%   w   - Wind velocity vector
%
%   Returned values:
%   dz  - State derivative for the current timestep

% Calculates the relative velocity of the drone to the wind
vx = (z(2)-w(1));
vy = (z(4)-w(2));

% Calculates the state derivative using the equations derived by hand
dz1 = z(2); % dz1/dt = dx/dt = z2
dz2 = -(0.5*rho*cd*A*vx*sqrt(vx^2 + vy^2))/m;
dz3 = z(4); % dz3/dt = dy/dt = z4
dz4 = - g -(0.5*rho*cd*A*vy*sqrt(vx^2 + vy^2))/m;

% dz1 = z(2); % dz1/dt = dx/dt = z2
% dz2 = -sign(z(2)-w(1))*(0.5*rho*cd*A*(z(2)-w(1))^2)/m;
% dz3 = z(4); % dz3/dt = dy/dt = z4
% dz4 = - g -sign(z(4)-w(2))*(0.5*rho*cd*A*(z(4)-w(2))^2)/m;

dz = [dz1; dz2; dz3; dz4];