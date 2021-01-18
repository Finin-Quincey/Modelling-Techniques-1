function znext = stepEuler(t, z, dt, derivFunc)
% stepEuler    Computes one step using the Euler method. This function
% is generic so it will work for any ivp; it is not specific to the anti-
% -drone sentry problem.
%
%   Input Arguments:
%   t           - Value of time at the start of the step being computed
%   z           - State vector at time t
%   dt          - Timestep for the numerical solution; smaller values are
%                 slower but more accurate
%   derivFunc   - Handle to the state derivative function describing the
%                 differential equation(s)
%
%   Returned values:
%   znext       - State vector for time t+dt

% Calculates the state derivative from the current state
dz = derivFunc(t, z);

% Calculates the next state vector from the previous one using Euler's
% update equation
znext = z + dt * dz;