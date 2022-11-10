%% LunaSim Offloader Simulation
% This code should offer a basic simulator for the z-offloader, which
%     allows for changes in spring, motor, and spool values, and shows what
%     is needed otherwise.
%
% Assumptions: Motor acts instantaneously
%              No Strain in cable
%              User CG at center of mass. 
%              Weight of attachment system is 100 lbf
%
%% Changelog
% 11-3-22, 12:30 - Original Code Written [Brennan]
% ==========================================
% 11-5-22 - Fixed gravity acceleration unit mismatch, added motor power,
% changed motor torque model to not need I [Kevin]
% ==========================================
% 11-8-22 - Fixed spring torque calculation to be max torque at bottom of
%       jump [Kevin]
% -Fixed power calculation
% -Fixed theta to be calculated based on position of the end of the cable
%       (user head position) rather than the position of the user's cg



%% Simulator set-up and constants
all_fig = findall(0,'type','figure');
close(all_fig);
clf; close all; clear; clc;

%==================== INPUT DATA ===================================
IPPT = 92; %Inch Pounds per Turn for Spring
W = 300; % Weight of load [lbf]
ag = 1.62/9.81;
F_c = W*(1-ag); % tension in the cable [lbf]
R_spool = 1.875; % Radius of Cable Spool [in]
T_s0 = F_c*R_spool; %Torque that the spring is set to at rest [in*lbf]
%===================================================================




% W = 300; %Maximum weight to lift [lbf]
% Radius = 1.875; %Spool Radius [in]
% k_spring = 1; %Spring Constant [in*lbf / turn]
% t_motor = 1; %Max motor torque [in*lbf]

g_e = 32.2; %Gravitational Acceleration on earth [ft/s^2]
g_m = 5.33; %Gravitational Acceleration on moon [ft/s^2]
h_max = 9*12; %Maximum height of the structure [in]
format_spec = '%.2f'; %format var for nice looking numbers



%% Creating Perfect Jump Simulation to start
user_h = 60; %Height of user [in]    Min: 60 in  Max: 73.6 in
user_w = 244; %Weight of user [lbf]  Min: 113 lbf Max: 244 lbf

% Initial Conditions for ode for a perfect height jump.
% Determined by max height jump.
z0 = user_h/2; %Initial position of users CG [in]
v0 = sqrt(2*(g_m*12)*(h_max-(z0+(user_h/2)))); %Initial velocity required to reach max height [in/s]
Y0 = [z0; v0; 0; 3.6]; %Initial conditions vector. last 2 values for graphical looks. [in]; [in/s]; [in?]; [in/s?];
T_f = 2*sqrt((2*(h_max-(z0+(user_h/2)))) / (g_m*12));
TSPAN = [0 T_f]; %Time it takes to go up and come down.

[T,Y] = ode45(@(t,y) PerfJumpODE(t,y,g_m*12), TSPAN, Y0);


% Kevin: testing something
% c = [g_m, g_e, W, R_spool, h_max, IPPT]; %[g_m, g_e, W_load, R_spool, h_max, IPPT]
% [T,Y] = ode45(@(t,y) SpringJumpODE(t,y,c), TSPAN, Y0);


% Calculate Spring Torque and motor torque over this jump
z_cg = Y(:,1); % user cg position [in]
z = z_cg + (user_h/2); % end of cable position (user head position)
theta = (h_max-z) ./ R_spool; % [rad]
T_s = (IPPT/(2*pi)) .* theta; %Torque from springs [in*lbf]
% T_c = (user_w + 100) * R_spool; %Torque from user weight [in*lbf]
T_c = R_spool * W * (1-(g_m/g_e)); % Torque from moving user [in*lbf]
T_m = T_c - T_s; % Torque from motor [in*lbf]


% Calculate motor RPM
% T_diff = diff(T);
% Z_diff = diff(Y(:,1));
% RPM = (Z_diff ./ (2*pi*R_spool)) ./ T_diff .*60;
% Instantaneous RPM from instantaneous z_vel:
omega = Y(:,2)./R_spool; % [rad/s]
RPM = (30/pi) .* omega;

% Calculate Motor Power
% c = 33000 / (2*pi); % unit conversion constant
% Power = (12/(745.7*c)) .* T_m .* abs(RPM); % motor power [W]
Power = (T_m .* RPM)./63025; % Motor Power in Horsepower
% Power = P_hp .* 745.7; % Motor Power in Watts



%% Creating UI for data display
UI = uifigure('Name', "Offloading Simulator", 'Units', 'normalized'); %UI window
UI.Position = [.1 .1 .8 .8];
G1 = uigridlayout(UI, [1,2]); %Grid to arrange ui
G1.ColumnWidth = {'6x', '4x'};
G2 = uigridlayout(G1, [3,1]); %Subgrid to display plot+time slider
G2.RowHeight = {'8x', '1x', '1x'};
G3 = uigridlayout(G1, [5,1]);


VIS = uiaxes(G2); %axes to plot user motion over time

%Show a plot of the users trajectory in UI. 
UpdateAxes(VIS, Y(:,3), Y(:,1), user_h); 
xlabel(VIS, "Horizontal Position[in]")
ylabel(VIS, "Z-Height [in]")
ylim(VIS, [0 h_max]);
xlim(VIS, [-h_max/2, h_max/2]);

% Slider that adjusts time shown
SLD = uislider(G2, 'ValueChangedFcn',... 
    @(SLD,event) timeSlide(SLD, VIS, user_h, T, Y));
SLD.Limits = TSPAN;
SLD.Value = TSPAN(2);


%Showing plots on the right side of UI
%Plot for Motor Torque over time
MTOR = uiaxes(G3);
plot(MTOR, T, T_m);
grid(MTOR,'on');
xlabel(MTOR, "Time [s]");
ylabel(MTOR, "Motor Torque [in*lbf]");
title(MTOR, "Motor Torque over Time");

%Plot for Motor RPM over time
MRPM = uiaxes(G3);
plot(MRPM, T, RPM);
grid(MRPM,'on');
xlabel(MRPM, "Time [s]");
ylabel(MRPM, "Motor RPM");
title(MRPM, "Motor RPM over Time");

%WIP: Plot for Motor Power over Time
MPOW = uiaxes(G3);
plot(MPOW, T, Power);
grid(MPOW,'on');
xlabel(MPOW, "Time [s]");
ylabel(MPOW, "Motor Power [W]");
title(MPOW, "Motor Power over Time");

%Plot for Spring Torque over time
STOR = uiaxes(G3);
plot(STOR, T, T_s);
grid(STOR,'on');
xlabel(STOR, "Time [s]");
ylabel(STOR, "Spring Torque [in*lbf]");
title(STOR, "Spring Torque over Time");

% Display numbers for max torques and rpm
G4 = uigridlayout(G3, [3,1]);
INFO1 = uilabel(G4);
INFO1.Text = append("The Maximum Motor Torque is ", num2str(max(T_m), format_spec), " in*lbf");
INFO1.FontSize = 15;

INFO2 = uilabel(G4);
INFO2.Text = append("The Maximum Motor RPM is ", num2str(max(RPM), format_spec));
INFO1.FontSize = 15;

INFO3 = uilabel(G4);
INFO3.Text = append("The Maximum Spring Torque is ", num2str(max(T_s), format_spec), " in*lbf");
INFO1.FontSize = 15;

%%UI functions

%timeSlide makes sure all plots are updated to according time when the
%slider is moved
function timeSlide(SLD, VIS, h, T, Y)
    time = SLD.Value;
    t = find(T >= time);
    t = t(1);
    
    x = Y(1:t, 3); y = Y(1:t, 1);
    UpdateAxes(VIS, x, y, h);
end


%Update axes updates all plots based on provided points
function UpdateAxes(VIS, x, y, h)
    cla(VIS);
    plot(VIS, x, y, 'LineWidth', 2);
    
    [c_x, c_y] = UserPoints(x(end), y(end), h);
    
    hold(VIS, 'on')
    plot(VIS, c_x, c_y);
    dy = h/2;
    plot(VIS, x, y+dy,"Color",[11 64 9]./256);
    hold(VIS, 'off');
    legend(VIS,"CG Position","User","Cable End Position","Location","northeast")
end


% Draws a general human shape for plotting. based on measurments in
% requirements
function [user_x, user_y] = UserPoints(x,y,h)
    feet = y-h/2;
    hip = y-((.530-.5)*h);
    hand = y-((.5-.485)*h);
    shoulder = y+((.819-.5)*h);
    head_b = y+((.870-.5)*h);
    head = y+h/2;
    
    x1 = x-.15*h; x2 = x-.1*h; x3 = x; x4 = x+.1*h; x5 = x+.15*h; x6 = x-.05*h; x7 = x+.05*h;
    
    user_x = [x3, x3, x2, x2, x1, x1, x6+.025*h, x6+.025*h, x6, x6, x7, x7, x7-.025*h, x7-.025*h, x5, x5, x4, x4, x3, x3];
    user_y = [hip, feet, feet, hand, hand, shoulder, shoulder, head_b, head_b, head, head, head_b, head_b, shoulder, shoulder, hand, hand, feet, feet, hip];
end


%%Ode function library
function Y = PerfJumpODE(T,Y,c)
    %Y is vector of velocities and positions laid out as such:
    %     [z_pos; z_vel; x-y_pos; x-y_vel]
    %c is vector of other info for simulation laid out as such:
    %     [g_m]

%This ode is basic and says that we have some constant velocity horizontal
%movement (for graphical asthetic sake) and some vertical movement coming
%from offloader to create a parabola.

dz = Y(2);
ddz = -c(1);
dx = Y(4);
ddx = 0;

Y = [dz;ddz;dx;ddx];
end

function dY = SpringJumpODE(T,Y,c)
    %Y is a vector of velocity, position, and torque such that:
    %   [z_pos; z_vel; x-y_pos; x-y_vel;]
    %c is a vector containing constants and other info:
    %   [g_m, g_e, W_load, R_spool, h_max, IPPT]

    %% Give names to input vector components
    z_pos = Y(1); % [in]
    z_vel = Y(2); % [in/s]
    xy_pos = Y(3); % [in]
    xy_vel = Y(4); % [in/s]

    g_m = c(1); % [ft/s^2]
    g_e = c(2); % [ft/s^2]

    % convert accelerations in ft/s^2 to in/s^2 for consistent distance units
    g_m = 12*g_m; % [in/s^2]
    g_e = 12*g_e; % [in/s^2]

    W = c(3); % [lbf]
    R = c(4); % [in]
    h_max = c(5); % [in]
    IPPT = c(6); % [in-lbf/turn]
    k = IPPT / (2*pi); % spring constant [in-lbf/rad]

    L = h_max - z_pos; % length of cable spooled out [in]
    theta = L/R; % angular position of the spool [rad]
    tau_s = k*theta; % [in-lbf]
    tau_m = W*R*(1-(g_m/g_e)) - tau_s; % [in-lbf]
    
    top = tau_m + tau_s;
    bot = W*R;
    a_z = g_e*(1 - (top/bot)); % z-acceleration [in/s^2]

    dz = z_vel;
    ddz = -a_z;
    dx = xy_vel;
    ddx = 0;

    dY = [dz;ddz;dx;ddx];
end