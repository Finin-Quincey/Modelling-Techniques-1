function varargout = sentrySimulator(varargin)
%% sentrySimulator Extension to the basic assignment
%
%      GUI-based simulation of the anti-drone sentry problem. Allows the
%      user to define inputs and problem parameters, then launch the
%      projectile. An animation of the projectile is shown in approximately
%      real-time (within the limits of the computer/MATLAB; the animation
%      speed can also be changed by the user), along with the path of the
%      projectile computed by the sentry. This allows the deviation from
%      the actual path to be analysed. The 'actual' path is calculated
%      using the RK-4 method with a sufficiently accurate timestep that the
%      difference from the true values is negligible. A popup message is
%      displayed after the animation with various numerical values from the
%      simulation.
%
%      The simulation also allows the user to manually control the launch
%      angle and velocity instead of allowing the sentry to compute a path.
%
%      Example parameters where the sentry thinks it will hit, but actually
%      misses due to inaccuracies: drone position (100, 50), Euler method
%      with a timestep of 0.1s.
%
%      The input(s) varargin are used for internal purposes by MATLAB.

% Last Modified by GUIDE v2.5 30-Nov-2017 18:10:57

%% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @sentrySimulator_OpeningFcn, ...
    'gui_OutputFcn',  @sentrySimulator_OutputFcn, ...
    'gui_LayoutFcn',  [] , ...
    'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
%% End initialization code - DO NOT EDIT

% Executes just before sentrySimulator is made visible.
function sentrySimulator_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to sentrySimulator (see VARARGIN)

% Choose default command line output for sentrySimulator
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% Initialises the plots
redraw(handles);

% UIWAIT makes sentrySimulator wait for user response (see UIRESUME)
% uiwait(handles.figure1);

% Outputs from this function are returned to the command line.
function varargout = sentrySimulator_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

% -------------------------------------------------------------------------

%% Redraws plots; used when a parameter changes.
% Returns handles to the various plots for later use.
function [projectile, drone, capturezone, velocityarrow, dronearrow] = redraw(handles)

% Initialise axes
axes(handles.axes);
cla;

% Display parameters (could be interactive/zoomable in the future)
width = 250;
height = 150;
groundheight = 25;

% Plots the ground
area([0, width], [0, 0], -groundheight, 'FaceColor', [0.5, 0.8, 0.1], 'LineStyle', 'none');
hold on;

% Plots the projectile
launchy = str2double(get(handles.launchy, 'String'));
projectile = plot(0, launchy, 'MarkerFaceColor', [0 0 0], 'MarkerEdgeColor', 'none', 'MarkerSize', 4, 'Marker', 'o');

% Plots the initial velocity as an arrow
manualmode = get(handles.manualmode, 'Value');
theta = deg2rad(str2double(get(handles.launchangle, 'String')));
launchvel = str2double(get(handles.launchvel, 'String'));
velocityarrow = quiver(0, launchy, launchvel*cos(theta), launchvel*sin(theta), 'k');
velocityarrow.LineWidth = 1;

% Hides the velocity arrow in automatic mode, since the angle hasn't been
% determined yet
if ~manualmode
    velocityarrow.Visible = 'off';
end

% Plots the drone
x = str2double(get(handles.xpos, 'String'));
y = str2double(get(handles.ypos, 'String'));
vx = str2double(get(handles.xvel, 'String'));
vy = str2double(get(handles.yvel, 'String'));
r = 1; %str2double(get(handles.radius, 'String'));
drone = plot(x, y, 'MarkerFaceColor', [0, 0, 1], 'MarkerEdgeColor', 'none', 'MarkerSize', 3, 'Marker', 'square');

% Plots the capture radius
[xs, ys] = circle(x, y, r, 50);
capturezone = plot(xs, ys, 'b');

% Plots the drone velocity as an arrow
dronearrow = quiver(x, y, vx, vy, 'b', 'LineWidth', 1);

% Sets the axis scales and limits
xlim([0, width]);
ylim([-groundheight, height-groundheight]);
daspect([1, 1, 1]);
handles.axes.XTick = (0:25:width);
handles.axes.YTick = (-groundheight:25:height-groundheight);

% Labels were removed since they are obvious and just add clutter
%xlabel('x (m)');
%ylabel('y (m)');

% Sets the sky colour (done last so it doesn't get overwritten)
handles.axes.Color = [0.75, 0.9, 1];

%% Callback functions for button presses, etc.

%% Executes when launch button is pressed. This is where most of the processing is done.
function launch_Callback(launchbutton, eventdata, handles)
% launchbutton    handle to the launch button object
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%% Constants (now retrieved from table)
% Constants in the table do not affect the figure until after launch,
% whereas the parameters in their own boxes usually do.

% Gets the table data vector
constants = handles.constants.Data;

m = constants(1); % Mass of projectile (kg)
A = constants(2); % Cross-sectional area of projectile (m^2)
cd = constants(3); % Drag coefficient of projectile
M = constants(4); % Mass of drone (kg)
A2 = constants(5); % Cross-sectional area of parachute (m^2)
cd2 = constants(6); % Drag coefficient of parachute
failsafeHeight = constants(7); % Height at which parachute is deployed if the projectile misses (m)
g = constants(8); % Acceleration due to gravity (m/s^2) at Earth's surface
rho = constants(9); % Density of air (kg/m^3)
launchvel = constants(10); % Density of air (kg/m^3)

%% Retrieval of data from buttons and user input boxes

% Equation solver
solver = get(handles.solver, 'SelectedObject');
useRK4 = strcmp(get(solver, 'String'), 'Runge-Kutta');

% Environment
windx = str2double(get(handles.windx, 'String'));
windy = str2double(get(handles.windy, 'String'));
r = str2double(get(handles.radius, 'String'));
launchy = str2double(get(handles.launchy, 'String'));

% Drone coordinates and velocity
x = str2double(get(handles.xpos, 'String'));
y = str2double(get(handles.ypos, 'String'));
vx = str2double(get(handles.xvel, 'String'));
vy = str2double(get(handles.yvel, 'String'));

% Timestep
dt = str2double(get(handles.timestep, 'String'));

% Manual control
manualmode = get(handles.manualmode, 'Value');
theta = deg2rad(str2double(get(handles.launchangle, 'String')));
if manualmode
    % Launch velocity is specified as a constant in automatic mode
    launchvel = str2double(get(handles.launchvel, 'String'));
end

% Animation speed
animationspeed = get(handles.animationspeed, 'Value');

% Results checkbox
showResults = get(handles.toggleresults, 'Value');

%% Setup

% Redraws plots and accesses their handles
[projectile, drone, capturezone, arrow, arrow2] = redraw(handles);

% Checks what state the simulator is in (would be much better to use some
% sort of global variable, but we can't because this is a function, which
% is very restrictive in MATLAB.)
if ~strcmp(launchbutton.String, 'Launch')
    % Resets the launch button
    launchbutton.BackgroundColor = [0.467, 0.675, 0.188];
    launchbutton.String = 'Launch';
    
else
    % Changes the launch button to read 'computing...' with an orange background
    launchbutton.BackgroundColor = [0.871, 0.49, 0.0];
    launchbutton.String = 'Computing...';
    % Gives the button a chance to redraw (I'm not sure why it works... but it works!)
    pause(0.001);
    
    % Disables the manual mode button
    handles.manualmode.Enable = 'off';
    
    % Hides the constants table
    handles.constants.Visible = 'off';
    handles.showmore.Value = false;

    % Hides the arrows
    arrow.Visible = 'off';
    arrow2.Visible = 'off';
    
    %% Trajectory calculations
    
    % State derivative function handle (see testShootingMethod line 25 for
    % an explanation) for initial trajectory
    ascent = @(t1, z1) stateDerivProjectileWind(t1, z1, m, rho, cd, g, A, [windx, windy]);
    
    % Automatic (normal) operation; extra step to calculate launch angle
    % and velocity using the shooting method
    if ~manualmode
        
        try
            % Computes the path predicted by the sentry
            [deg, ~, z2, k] = shootingMethod(0, launchy, launchvel, 10, [x, y], [vx, vy], [20, 60], r, dt, ascent, useRK4);
        catch
            % If the trajectory cannot be computed (probably because the
            % drone is out of range), button turns red
            launchbutton.BackgroundColor = [0.679, 0.078, 0.18];
            launchbutton.String = 'Target out of range (click to reset)';
            return; % Computation failed, exit the function
        end
        
        % Discards the part after it reaches the drone
        z2 = z2(:, 1:k);
        
        theta = deg2rad(deg); % Angle determined using shooting method
        
        % Plots the computed path
        predictedpath = plot(z2(1, :), z2(3, :), 'r', 'LineStyle', '--');
        
    end
    
    % Calculates the components of the launch velocity
    launchvelx = launchvel*cos(theta);
    launchvely = launchvel*sin(theta);
    
    % Computes the 'actual' ascent path using very accurate values
    steptime = 0.001;
    [t, z] = ivpSolver(0, [0; launchvelx; launchy; launchvely], steptime, 10, ascent, true);
    
    % Determines whether the path hits the drone or not
    impact = 0;
    for k = 1:length(t)
        % Calculates the drone coordinates at time t(m)
        dronex = x + t(k)*vx;
        droney = y + t(k)*vy;
        % Tests whether the distance between the drone and the projectile
        % is less than or equal to the capture radius
        if sqrt((z(1, k) - dronex)^2 + (z(3, k) - droney)^2) <= r
            impact = k;
            break;
        end
    end
    
    % If impact succeeded, replaces the path after impact with the
    % parachute descent; if not replaces the path within 5m of the ground
    % with parachute descent
    if impact > 0
        % Discards the part after it reaches the drone
        z = z(:, 1:impact);
        t = t(1:impact);
        % Calculates velocity after impact using conservation of momentum
        v = collideAndCoalesce(m, [z(2, end), z(4, end)], M, [vx, vy]);
        z(2, end) = v(1);
        z(4, end) = v(2);
        % State derivative function handle for descent with drone
        descent = @(t1, z1) stateDerivProjectileWind(t1, z1, m+M, rho, cd2, g, A2, [windx, windy]);
    else
        % Searching backwards, finds the index at which y first becomes
        % more than the failsafe height
        for k = 0:length(z(3, :))-1
            if z(3, end - k) > failsafeHeight
                break; % Found it so stop incrementing k
            end
        end
        % Discards the rest
        z = z(:, 1:length(z(3, :))-k);
        t = t(1:length(t)-k);
        % State derivative function handle for descent without drone
        descent = @(t1, z1) stateDerivProjectileWind(t1, z1, m, rho, cd2, g, A2, [windx, windy]);
    end
    
    % Calculates the descent trajectory
    [t2, z2] = ivpSolver(t(end), z(:, end), steptime, 30, descent, true);
    % Concatenates the two
    z = [z, z2];
    t = [t, t2];
    
    % Finds the index at which y first becomes less than 0
    for k = 1:length(z(3, :))
        if z(3, k) <= 0
            break; % Found it so stop incrementing k
        end
    end
    
    % Discards the rest
    z = z(:, 1:k);
    t = t(1:k);
    
    %% Animation
    
    % Initialises the plot for the 'actual' path
    path = plot(0, 0, 'k');
    
    % Creates (or hides) the legend
    if manualmode
        legend off; % No point having a legend for one line!
    else
        key = legend([path, predictedpath], 'Actual Trajectory', 'Computed Trajectory');
        set(key, 'Color', 'w');
    end
    
    % Changes the launch button to read 'stop' with a red background
    launchbutton.BackgroundColor = [0.679, 0.078, 0.18];
    launchbutton.String = 'Stop';
    
    % Starts the timer for this iteration
    time = tic;
    % Time for each frame. 0.05 (20fps) seems to be roughly how long each
    % step takes to plot on my computer, any shorter and it runs slowly.
    frametime = 0.05;
    
    % Initialises the loop counter
    n = 1;
    
    % The loop ends under any of the following conditions (in order):
    % - The end of the calculated trajectory has been reached
    % - The figure has been closed
    % - The stop button has been pressed
    while n <= length(t) && ishandle(handles.togglegrid) && launchbutton.get('Value')
        % Moves the drone
        x = x + vx*frametime*animationspeed;
        y = y + vy*frametime*animationspeed;
        [xs, ys] = circle(x, y, r, 50);
        
        % Updates the plot (quicker than re-plotting each time)
        if impact > 0 && n > impact
            drone.Visible = 'off';
            capturezone.Visible = 'off';
        end
        set(path, 'xdata', z(1, 1:n), 'ydata', z(3, 1:n));
        set(projectile, 'xdata', z(1, n), 'ydata', z(3, n));
        set(drone, 'xdata', x, 'ydata', y);
        set(capturezone, 'xdata', xs, 'ydata', ys);
        drawnow;
        
        % Adds the time as a title. %.2f prints t(n) to 2dp, with trailing
        % zeros if required (this stops it moving about).
        title(sprintf('t = %.2fs', t(n)));
        
        % Updates the grid setting
        if get(handles.togglegrid, 'Value')
            grid on;
        else
            grid off;
        end
        
        % Ensures the animation plays at the correct real-time speed by
        % pausing for 1 timestep minus however long the iteration took to
        % execute. On my computer, this works above dt ~ 0.05.
        pause(frametime - toc(time));
        
        % Restarts the timer ready for the next iteration
        time = tic;
        
         % Increments the loop counter. The faster the animation speed, the
         % more steps it jumps for each frame.
        n = n + round((frametime/steptime)*animationspeed);
        
    end
    
    % Resets the launch button
    launchbutton.BackgroundColor = [0.467, 0.675, 0.188];
    launchbutton.String = 'Launch';
    
    % Displays numeric results in popup box if enabled
    if showResults
        % Slightly different depending on whether drone was captured or not
        if impact > 0
            msgbox(sprintf(strcat('Launch angle: %.3g degrees\n', ...
                'Intercepted target at t = %.3gs\n', ...
                'Total flight time: %.3gs\n', ...
                'Captured drone landing distance: %.3gm\n', ...
                'Ground impact speed: %.3gm/s\n'), ...
                rad2deg(theta), t(impact), t(end), z(1, end), sqrt(z(2, end)^2 + z(4, end)^2)), 'Results', 'help');
        else
            msgbox(sprintf(strcat('Launch angle: %.3g degrees\n', ...
                'Failed to intercept target\n', ...
                'Total flight time: %.3gs\n', ...
                'Projectile landing distance: %.3gm\n', ...
                'Ground impact speed: %.3gm/s\n'), ...
                rad2deg(theta), t(end), z(1, end), sqrt(z(2, end)^2 + z(4, end)^2)), 'Results', 'help');
        end
    end
    
    % Re-enables the manual mode button
    handles.manualmode.Enable = 'on';
end

% Executes when the manual mode button is pressed
function manualmode_Callback(hObject, eventdata, handles)
% Greys-out or un-greys-out the manual control inputs
if get(hObject,'Value')
    set(handles.launchvel, 'Enable', 'on');
    set(handles.launchangle, 'Enable', 'on');
else
    set(handles.launchvel, 'Enable', 'off');
    set(handles.launchangle, 'Enable', 'off');
end
% Redraws the plot
redraw(handles);

% Executes when the drone x position box is modified
function xpos_Callback(hObject, eventdata, handles)
% Validates the new user input and changes it if necessary
set(hObject, 'String', validateAndClamp(get(hObject,'String'), 0, 500, 0));
% Redraws the plot
redraw(handles);

% Executes when the drone y position box is modified
function ypos_Callback(hObject, eventdata, handles)
% Validates the new user input and changes it if necessary
set(hObject, 'String', validateAndClamp(get(hObject,'String'), 0, 500, 0));
% Redraws the plot
redraw(handles);

% Executes when the drone x velocity box is modified
function xvel_Callback(hObject, eventdata, handles)
% Validates the new user input and changes it if necessary
set(hObject, 'String', validateAndClamp(get(hObject,'String'), -250, 250, 0));
% Redraws the plot
redraw(handles);

% Executes when the drone y velocity box is modified
function yvel_Callback(hObject, eventdata, handles)
% Validates the new user input and changes it if necessary
set(hObject, 'String', validateAndClamp(get(hObject,'String'), -250, 250, 0));
% Redraws the plot
redraw(handles);

% Executes when the launch angle box is modified
function launchangle_Callback(hObject, eventdata, handles)
% Validates the new user input and changes it if necessary
set(hObject, 'String', validateAndClamp(get(hObject,'String'), -90, 90, 40));
% Redraws the plot
redraw(handles);

% Executes when the launch velocity box is modified
function launchvel_Callback(hObject, eventdata, handles)
% Validates the new user input and changes it if necessary
set(hObject, 'String', validateAndClamp(get(hObject,'String'), 0, 100, 50));
% Redraws the plot
redraw(handles);

% Executes when the grid checkbox is clicked
function togglegrid_Callback(hObject, eventdata, handles)
% Updates the grid setting
axes(handles.axes);
if get(hObject, 'Value')
    grid on;
else
    grid off;
end

% Executes when the results checkbox is clicked
function toggleresults_Callback(hObject, eventdata, handles)
% Don't need to do anything here

% Executes when the the axes are clicked
function axes_ButtonDownFcn(hObject, eventdata, handles)
% Don't need to do anything here

% Executes when the equation solver is changed
function solver_SelectionChangedFcn(hObject, eventdata, handles)
% Don't need to do anything here

% Executes when the timestep box is modified
function timestep_Callback(hObject, eventdata, handles)
% Validates the new user input and changes it if necessary
set(hObject, 'String', validateAndClamp(get(hObject,'String'), 0.001, 10, 0.01));

% Executes when the launch height box is modified
function launchy_Callback(hObject, eventdata, handles)
% Validates the new user input and changes it if necessary
set(hObject, 'String', validateAndClamp(get(hObject,'String'), 0, 1000, 1));
% Redraws the plot
redraw(handles);

% Executes when the capture radius box is modified
function radius_Callback(hObject, eventdata, handles)
% Validates the new user input and changes it if necessary
set(hObject, 'String', validateAndClamp(get(hObject,'String'), 0.001, 100, 1));
% Redraws the plot
redraw(handles);

% Executes when the wind x velocity box is modified
function windx_Callback(hObject, eventdata, handles)
% Validates the new user input and changes it if necessary
set(hObject, 'String', validateAndClamp(get(hObject,'String'), -1000, 1000, 0));

% Executes when the wind y velocity box is modified
function windy_Callback(hObject, eventdata, handles)
% Validates the new user input and changes it if necessary
set(hObject, 'String', validateAndClamp(get(hObject,'String'), -1000, 1000, 0));

% Executes when the animation speed slider is moved
function animationspeed_Callback(hObject, eventdata, handles)
% Updates the displayed speed (1x, 5x, etc.)
set(handles.animspeedlabel, 'String', sprintf('%.2gx', get(hObject, 'Value')));

% Executes when the more parameters... button is pressed
function showmore_Callback(hObject, eventdata, handles)
% Shows or hides the constants table
if get(hObject,'Value')
    handles.constants.Visible = 'on';
else
    handles.constants.Visible = 'off';
end

% Executes when a cell in the constants table is edited
function constants_CellEditCallback(hObject, eventdata, handles)
% hObject    handle to constants (see GCBO)
% eventdata  structure with the following fields (see MATLAB.UI.CONTROL.TABLE)
%	Indices: row and column indices of the cell(s) edited
%	PreviousData: previous data for the cell(s) edited
%	EditData: string(s) entered by the user
%	NewData: EditData or its converted form set on the Data property. Empty if Data was not changed
%	Error: error string when failed to convert EditData to appropriate value for Data
% handles    structure with handles and user data (see GUIDATA)


%% Creation functions

function xpos_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function ypos_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function xvel_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function yvel_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function launchangle_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function launchvel_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function axes_CreateFcn(hObject, eventdata, handles)

function timestep_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function launchy_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function radius_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function windx_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function windy_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function animationspeed_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

%% These (I assume) have something to do with the inner workings of the figure - they were autogenerated by MATLAB

function FileMenu_Callback(hObject, eventdata, handles)

function OpenMenuItem_Callback(hObject, eventdata, handles)
file = uigetfile('*.fig');
if ~isequal(file, 0)
    open(file);
end

function PrintMenuItem_Callback(hObject, eventdata, handles)
printdlg(handles.figure1)

function CloseMenuItem_Callback(hObject, eventdata, handles)
selection = questdlg(['Close ' get(handles.figure1,'Name') '?'],...
    ['Close ' get(handles.figure1,'Name') '...'],...
    'Yes','No','Yes');
if strcmp(selection,'No')
    return;
end

delete(handles.figure1)
