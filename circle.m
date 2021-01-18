function [xs, ys] = circle(x, y, r, n)
%CIRCLE Generates the coordinates of points around a circle.
%   circle(x, y, r, n) generates n equally spaced points around a circle
%   with centre (x, y) and radius r and returns them as x and y vectors.

% Creates a vector of equally spaced angle values from 0 to 2pi radians
theta = linspace(0, 2*pi, n);

% Calculates the coordinates of each point around the circumference
xs = r*cos(theta) + x;
ys = r*sin(theta) + y;

end