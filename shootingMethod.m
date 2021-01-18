function [launchangle, t, z, n, m] = shootingMethod(t0, y0, v0, tend, droneCoords, droneVel, theta, accuracy, dt, derivFunc, useRK4)
%SHOOTINGMETHOD Uses the shooting method to find the appropriate trajectory
%to intercept the drone.
%
%   Input Arguments:
%   t0, y0, v0  - Initial values of time, height and velocity respectively
%   tend        - Time period to calculate the trajectory for
%   droneCoords - Coordinate vector of the target drone
%   droneVel    - Velocity vector of the target drone
%   theta       - Vector of two initial guesses for the launch angle in
%                 degrees. Exact values are unimportant, but must be
%                 between 0 and 180.
%   accuracy    - Acceptable deviation from the exact target position
%   dt          - Timestep for the numerical solution; smaller values are
%                 slower but more accurate
%   derivFunc   - Handle to the state derivative function to be used to
%                 calculate the trajectory of the projectile
%   useRK4      - If true, the Runge-Kutta 4 method of numerical integration
%                 will be used; if false, the Euler method will be used.
%
%   Returned values:
%   launchangle - The calculated launch angle for the projectile
%   t, z        - Vectors of time and state values describing the path of
%                 the projectile from t0 to tend. Note that this function
%                 does not handle collisions or the subsequent path of the
%                 projectile.
%   n           - Index of the t and z values closest to the target
%                 interception
%   m           - Number of iterations (guesses) needed to compute a
%                 sufficiently accurate launch angle
%
%   Throws an error if either of the given theta values results in the
%   projectile not reaching the x coordinate of the target in the time
%   limit specified.

% Trying to generalise this function to any BVP proved to be too much
% of a headache since we don't know the time of interception, so this is
% specific to the anti-drone sentry problem.

% Checks all the arguments are valid
validateargs({t0, y0, v0, tend, droneCoords, droneVel, theta}, 'numeric');
validateargs({accuracy}, 'numeric', {{'>', 0}}); % Capture radius must be > 0
validateargs({derivFunc}, 'function_handle');
validateargs({useRK4}, 'logical');

% Vector of y error values. Must be defined outside the loop because
% otherwise its scope is limited to a single loop iteration and the
% previous error values will be lost.
yerror = zeros(1, 100);

% Extrapolates vector of drone coordinates for each time value.
droneCoords = [droneCoords(1) + droneVel(1)*(t0:dt:tend); droneCoords(2) + droneVel(2)*(t0:dt:tend)];

% Times out after 100 iterations, which should be more than enough!
for m = 1:100
    
    % Computes the next guess for the angle if necessary
    if m > length(theta)
        theta(m) = theta(m-1) - yerror(m-1) * ((theta(m-1) - theta(m-2)) / (yerror(m-1) - yerror(m-2)));
    end
    
    % Computes the velocity components at launch
    vx = v0*cos(deg2rad(theta(m)));
    vy = v0*sin(deg2rad(theta(m)));
    
    % Computes the resulting trajectory of the projectile (this gets output
    % so we don't needlessly calculate it again)
    [t, z] = ivpSolver(t0, [0, vx, y0, vy], dt, tend, derivFunc, useRK4);
    
    index = 0; % Records the step number at which the y error is to be tested
    
    % Finds the index at which x first becomes larger than the target x coord
    for n = 1:length(z(1, :))
        if z(1, n) >= droneCoords(1, n)
            index = n;
            break; % Found it so stop incrementing n
        end
    end
    
    % If index is still zero the projectile did not reach the x coord of
    % the drone, so a guess can't be calculated and an error must be thrown
    if index == 0
        error(strcat('The supplied test angle (%g degrees) did not travel', ...
        ' far enough in the given time limit to calculate the error. Try a', ...
        ' different angle or a longer time limit.'), theta(m));
    end
    
    % Calculates the error in y. It makes sense to take the point at which x is
    % correct since the projectile never moves back towards the sentry (except
    % when there is wind, but even then x is still better because we always
    % want the first solution, whereas for y the projectile could reasonably be
    % rising or falling when it hits the drone).
    yerror(m) = z(3, index) - droneCoords(2, index);
    
    % If the error is within the acceptable limit, a solution has been
    % found so the loop can exit.
    if abs(yerror(m)) <= accuracy
        break;
    end
    
end

% Outputs the launch angle
launchangle = theta(m);