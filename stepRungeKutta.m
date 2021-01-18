function znext = stepRungeKutta(t, z, dt, derivFunc)
% stepRungeKutta    Computes one step using the RK-4 method. This function
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

% Calculates the state derivatives from the current state
a = dt * derivFunc(t, z);
b = dt * derivFunc(t + dt/2, z + a/2);
c = dt * derivFunc(t + dt/2, z + b/2);
d = dt * derivFunc(t + dt, z + c);

% Calculates the next state vector from the previous one using the RK-4
% update equation
znext = z + (a + 2*b + 2*c + d)/6;