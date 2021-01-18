function [x] = validateAndClamp(x, min, max, default)
%VALIDATEANDCLAMP Checks that the input string represents a number, and if
%so clamps it to within the specified limits, or sets it to the default if
%not.
%
%   Input Arguments:
%
%   x        The input to be validated and clamped.
%   min      The lower limit to clamp x to.
%   max      The upper limit to clamp x to.
%   default  The default value for x if the input x is not a number.

% Checks max is greater than min
if min > max
    error('Min must be less than or equal to max');
end

% Checks the default is between the two bounds
if default < min || default > max
    error('Default must be within specified range');
end

% Tries to convert x to a number. If this fails, x will be NaN.
x = str2double(x);

% If x is a number...
if ~isnan(x)
    % ... clamps x to within the specified range
    if x < min
        x = min;
    elseif x > max
        x = max;
    end
% If not...
else
    % ... sets x to the specified default
    x = default;
end