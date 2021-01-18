function [t, z] = ivpSolver(t0, z0, dt, tend, derivFunc, useRK4)
% ivpSolver    Solves an initial value problem (IVP) and returns the
% resulting time vector and state vectors. This function is generic so it
% will work for any ivp; it is not specific to the anti-drone sentry
% problem.
%
%   Input Arguments:
%   t0          - Initial value of time to compute from
%   z0          - Initial state vector
%   dt          - Timestep for the numerical solution; smaller values are
%                 slower but more accurate
%   tend        - Final value of time to compute to
%   derivFunc   - Handle to the state derivative function describing the
%                 differential equation(s)
%   useRK4      - If true, the Runge-Kutta 4 method of numerical integration
%                 will be used; if false, the Euler method will be used.
%
%   Returned values:
%   t, z        - Vectors of time and state values from t0 to tend.

% Checks all the arguments are valid
validateargs({t0, z0, dt, tend}, 'numeric');
validateargs({derivFunc}, 'function_handle');
validateargs({useRK4}, 'logical');

% Generates the time vector. Doing this first makes the code much neater,
% and also removes an error (which was present in the code downloaded off
% moodle!) where the last time step was tend + dt and not tend.
t = t0:dt:tend;

% Sets the initial conditions
z(:,1) = z0;

% Iterates through each time value (doesn't do the last time value since we
% don't need to calculate the next one for that)
for n = 1:length(t)-1
    if useRK4
        % Applies the RK-4 method for one time step
        z(:,n+1) = stepRungeKutta(t(n), z(:,n), dt, derivFunc);
    else
        % Applies the Euler method for one time step
        z(:,n+1) = stepEuler(t(n), z(:,n), dt, derivFunc);
    end
end