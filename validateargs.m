function [] = validateargs(args, types, attributes)
%VALIDATEARGS Validates the arguments of a function, using the built-in
%validateattributes function, and throws an error if they do not match the
%specified crtieria.
%
%   Input Arguments:
%
%   ARGS         A cell array of arguments to be validated.
%
%   TYPES        A cell array of strings representing the classes to
%   validate the arguments against. Nest cell arrays to add multiple
%   acceptable classes for a single argument. Pass in a single string
%   instead of a cell array to validate all arguments against the same
%   class.
%
%   ATTRIBUTES   A cell array of attributes to validate the arguments
%   against. Each entry in this cell array is itself a cell array which
%   defines the attributes, as in validateattributes. Empty trailing cells
%   can be omitted, or the argument itself can be omitted if no attributes
%   are required.
%
%   Example:
%
%   validateargs({true, [3, 4], 'hello'}, {'logical', 'numeric', 'char'},
%   {{}, {'>', 2}})
%   
%   This will validate that: - Argument 1 (true) is a logical array -
%   Argument 2 ([3, 4]) is a numeric array where all the elements are
%   greater than 2 - Argument 3 ('hello') is a char array or a string

% Makes sure the arguments passed to this function are appropriate
validateattributes(args, {'cell'}, {}, 1);
validateattributes(types, {'cell', 'char'}, {}, 2);

% Checks whether the attributes argument was specified
if exist('attributes', 'var')
    % If so, makes sure it is appropriate
    validateattributes(attributes, {'cell'}, {}, 3);
else
    % If not, initialises it as an empty cell array to prevent errors
    attributes = {};
end

% Checks that types and arguments are the same length
if ~ischar(types) && length(args) ~= length(types)
    error('Args and types must be the same length');
end

% Iterates through each argument to be validated in turn
for n=1:length(args)
    % Retrieves the attribute at index n, or sets it to an empty cell array
    % if none exists
    if n > length(attributes)
        attribute = {};
    else
        attribute = attributes{n};
    end
    
    % Validates the nth argument. Different for each of the different ways
    % that types may be input (string, 1D cell array of strings, or 2D cell
    % array of strings)
    if ischar(types)
        validateattributes(args{n}, {types}, attribute, n);
    elseif ischar(types{n})
        validateattributes(args{n}, types(n), attribute, n);
    else
        validateattributes(args{n}, types{n}, attribute, n);
    end
end