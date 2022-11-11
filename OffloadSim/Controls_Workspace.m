%% LunaSim Offloader Control Simulation
% This code should offer a basic simulator for the z-offloader, which
%     allows for changes in spring, motor, and spool values, and shows what
%     is needed otherwise. Should allow for the implementation of control
%     laws.
%
%   This code will first run simulations of predetermined cases, allowing
%   the user to change the motors control law in a function. Then,
%   everything is added to a UI for display at the end. The data will
%   largely be managed in struct data types, because it makes everything
%   much more manageable.
%
% Assumptions: Motor acts instantaneously
%              No Strain in cable
%              User CG at center of mass. 
%              Weight of attachment system is 100 lbf
%
%% Changelog
% 11-7-22, - Original Code Written [Brennan]

%% Proposed Additions:
% - Make it so you can entirely use the UI window to check values
%     * Add Input Data and User Data input fields to the UI window
% - Physics verification ODE
%     * take the torque estimates and apply that to a person to verify the dynamics are correct
% - Monte Carlo Simulation
%     * Primarily, check all user height and weight combos within the expected range
%     * Also check all IPPTs (within a certain range, maybe 20-100)
%     * optimize for minimum motor wattage

%% Input Data
% clc; clear; cla;
all_fig = findall(0,'type','figure');
close(all_fig);
clf; close all; clear; clc;

%==================== INPUT DATA ===================================
IPPT = 90; %Inch Pounds per Turn for Spring
R_Spool = 1.875; %Radius of Cable Spool [in]
%===================================================================

k = IPPT / (2*pi); % spring constant (in*lbf / rad)

%==================== USER DATA ====================================
user_h = 60; %Height of user [in]    Min: 60 in  Max: 73.6 in
user_w = 244; %Weight of user [lbf]  Min: 113 lbf Max: 244 lbf
%===================================================================

% Constants
g_e = 32.2*12; %Gravitational Acceleration on earth [in/s^2]
g_m = 5.33*12; %Gravitational Acceleration on moon [in/s^2]
h_max = 9*12; %Maximum height of the structure [in]
x_max = 9*12; %Maximum Length of structure [in]
y_max = 10*12; %Maximum Width of structure [in]
W = user_w + 100; %Weight of Cable Load

% Determine the initial spring condition
L_0 = h_max - user_h; % initial cable-out length
ST_0 = R_Spool * W * (1 - (g_m/g_e)); % initial spring torque required at rest (zeroed to standing)
theta_0_s = ST_0 / k; % initial rotational displacement of the spring [rad]
theta_0 = L_0 / R_Spool; % initial rotational displacement of the spool [rad]
theta_offset = theta_0 - theta_0_s; % preloading angle of the spring, + is same direction as theta_0 (spool out) [rad]

resting_spring_turns = theta_0_s / (2*pi); % total displacement of the spring when standing

Constants = [IPPT, ST_0, R_Spool, W, h_max, g_e, g_m, theta_offset, user_h];

%% Motion Simulations
%% Standing Jump
% This is the simplest situation. The user will stand still for one second,
% then jump to the top of the structure and fall back down and stand still
% for one more second. 

%determining start conditions and standing still for 1s
xstart = x_max / 2;
ystart = y_max / 2;
zstart = user_h/2;

T1 = (0:.2:.8)'; 
X1 = ones(length(T1), 1) * xstart;
Y1 = ones(length(T1), 1) * ystart;
Z1 = ones(length(T1), 1) * zstart;

%Compute initial velocity and time to reach top of structure for ODE IC's
v0 = sqrt(2*g_m*(h_max-user_h)); %Initial velocity required to reach max height
time = v0/g_m*2;

%Set up variables for ODE
IC = [X1(end), Y1(end), Z1(end), 0, 0, v0];
TSPAN = [1, T1(end)+time];
c = g_e;

[T,P] = ode45(@(t,p) JumpODE(t,p,c, Constants), TSPAN, IC);

% Calculate the motor and spring torques
z_cm = P(:,3); % user center of mass positions
[MT,ST,UT] = TorqueSolver(z_cm, Constants);

% Adding a second of standing to the end of the data.

MT1 = ones(length(T1),1) * MT(1);
ST1 = ones(length(T1),1) * ST(1);

MT1 = [MT1; MT; MT1];
ST1 = [ST1; ST; ST1];

temp = zeros(length(T1),1);

T1 = [T1; T; T(end)+T1+.2];
X1 = [X1; P(:,1); X1];
Y1 = [Y1; P(:,2); Y1];
Z1 = [Z1; P(:,3); Z1];

% Calculate motor RPM
vel_z = P(:,6); % [in/s]
omega = vel_z / R_Spool; % [rad/s]
RPM1 = omega * (30/pi); % [rev/min]
% T_diff = diff(T1);
% Z_diff = diff(Z1);
% RPM1 = (Z_diff ./ (2*pi*R_Spool)) ./ T_diff .*60;

% Calculate the motor power
Pow_hp = abs((MT .* RPM1)./63025); % Motor Power in Horsepower
Pow_w = Pow_hp .* 745.7; % Motor Power in Watts

% Putting all data into the Jump struct.
Jump = struct; Jump.T = T1;
Jump.X = X1; Jump.Y = Y1; Jump.Z = Z1;
Jump.MT = MT1; Jump.ST = ST1;
% Jump.Td = T_diff;
Jump.RPM = [temp; RPM1; temp];
Jump.POW = [temp; Pow_w; temp];


%% Slow Walk




%% Fast Walk




%% Pitch Forwards (Fall Forwards)



%% Running Laps Around the Space



%% Walk & Jump to End, Turn Around, and Repeat





%% Creating a UI to Display everything.
% Variables for the UI
format_spec = '%.2f'; %format var for nice looking numbers
sitches = {'Stationary Jump', 'Slow Walk', 'Fast Walk', 'Fall Forwards', 'Circular Walk', 'Complex Movement'};
C = [user_h];
Active = Jump;

UI = uifigure('Name', 'LunaSim Control Workspace');
set(UI, 'Position', [100 100 1000 700]);
    G1 = uigridlayout(UI, [1,2]);
     G1.ColumnWidth = {'6x', '4x'};
        G2 = uigridlayout(G1, [3,1]);
         G2.RowHeight = {'1x', 57, 40};
            AX = uiaxes(G2);
             title(AX, "LunaSim User Space");
             grid(AX, 'on');
             xlim(AX, [0 x_max]); ylim(AX, [0 y_max]); zlim(AX, [0 h_max]);
            G6 = uigridlayout(G2, [1,2]);
             G6.ColumnWidth = {45, '1x'};
            LBL = uilabel(G6, 'Text', "Time [s]");
            SLD = uislider(G6);
             SLD.Limits = [0, max(Jump.T)];
             SLD.Value = max(Jump.T);
             SLD.MajorTicks = 0:1:max(Jump.T);
            G3 = uigridlayout(G2, [1,2]);
             G3.ColumnWidth = {'1x', 80};
                DD = uidropdown(G3);
                 DD.Items = sitches;
                 DD.Value = sitches(1);
                BTN = uibutton(G3);
                 BTN.Text = 'Play';
        G4 = uigridlayout(G1, [5,1]);
         G4.RowHeight = {'2x', '2x', '2x', '3x', '2x'};
            INFO1 = uiaxes(G4);
             title(INFO1, "Spring Torque [in*lbs]");
             grid(INFO1, 'on');
            INFO2 = uiaxes(G4);
             title(INFO2, "Motor Torque [in*lbs]");
             grid(INFO2, 'on');
            INFO3 = uiaxes(G4);
             title(INFO3, "Motor RPM");
             grid(INFO3, 'on');
            INFO4 = uiaxes(G4);
             title(INFO4, "Motor Power [W]");
             grid(INFO4, 'on');
            G5 = uigridlayout(G4, [4,1]);
             G5.RowHeight = {'1x', '1x', '1x', '1x'};
                MAX1 = uilabel(G5);
                 MAX1.Text = "Max Spring Torque = " + max(Active.ST) + " [in*lbf]";
                MAX2 = uilabel(G5);
                 MAX2.Text = "Max Motor Torque = " + max(Active.MT) + " [in*lbf]";
                MAX3 = uilabel(G5);
                 MAX3.Text = "Max Motor RPM = " + max(Active.RPM) + "     Max Spring Turns = " + resting_spring_turns;
                MAX4 = uilabel(G5);
                 MAX4.Text = "Max Motor Power = " + max(Active.POW) + " [W]";

        
             SLD.ValueChangedFcn = @(SLD,event) TimeSlide(Active, SLD, AX, INFO1, INFO2, INFO3, INFO4, C);
% Plotting Initial Conditions for UI


PlotUser(Active, AX, INFO1, INFO2, INFO3, INFO4, length(Active.T), C);








%% Function Definitions

%% UI Functions
function TimeSlide(Active, SLD, AX, INFO1, INFO2, INFO3, INFO4, C)
    time = SLD.Value;
    Time = find(Active.T >= time);
    Time = Time(1);

    PlotUser(Active, AX, INFO1, INFO2, INFO3, INFO4, Time, C);

end


function PlotUser(Active, AX, INFO1, INFO2, INFO3, INFO4, Time, C)
    %Plotting User Position
    plot3(AX, Active.X(Time), Active.Y(Time), Active.Z(Time), 'o');
    hold(AX, 'on');
    
    %Plotting Path behind user
    if (Time <= 4)
        n = Time-1;
    else
        n = 4;
    end
    
    XPath = Active.X((Time-n):(Time));
    YPath = Active.Y((Time-n):(Time));
    ZPath = Active.Z((Time-n):(Time));
    
    plot3(AX, XPath, YPath, ZPath);
    
    
    %Plotting Box around user
    Box = GetUserCoords(Active.X(Time), Active.Y(Time), Active.Z(Time), C(1));
    plot3(AX, Box(:,1), Box(:,2), Box(:,3));
    hold(AX, 'off');
    
    
    %Plotting info graphs
    range = max(Active.ST) - min(Active.ST);
    timebar = [min(Active.ST) - .1*range, max(Active.ST) + .1*range];
    plot(INFO1, Active.T, Active.ST, 'Marker','.') 
    hold(INFO1, 'on');
    plot(INFO1, [Active.T(Time), Active.T(Time)], timebar);
    ylim(INFO1, timebar);
    hold(INFO1, 'off');
    
    range = max(Active.MT) - min(Active.MT);
    timebar = [min(Active.MT) - .1*range, max(Active.MT) + .1*range];
    plot(INFO2, Active.T, Active.MT, 'Marker','.') 
    hold(INFO2, 'on');
    plot(INFO2, [Active.T(Time), Active.T(Time)], timebar);
    ylim(INFO2, timebar);
    hold(INFO2, 'off');

    range = max(Active.RPM) - min(Active.RPM);
    timebar = [min(Active.RPM) - .1*range, max(Active.RPM) + .1*range];
    plot(INFO3, Active.T, Active.RPM, 'Marker','.') 
    hold(INFO3, 'on');
    plot(INFO3, [Active.T(Time), Active.T(Time)], timebar);
    ylim(INFO3, timebar);
    hold(INFO3, 'off');

    range = max(Active.POW) - min(Active.POW);
    timebar = [min(Active.POW) - .1*range, max(Active.POW) + .1*range];
    plot(INFO4, Active.T, Active.POW, 'Marker','.') 
    hold(INFO4, 'on');
    plot(INFO4, [Active.T(Time), Active.T(Time)], timebar);
    ylim(INFO4, timebar);
    hold(INFO4, 'off');
end

function BoxCoords = GetUserCoords(x,y,z,user_h)

    x1 = x - 20/2;
    x2 = x + 20/2;
    y1 = y - 40/2;
    y2 = y + 40/2;
    z1 = z - user_h/2;
    z2 = z + user_h/2;

    BoxCoords = [x1 y1 z1; ...
                 x1 y2 z1; ...
                 x2 y2 z1; ...
                 x2 y1 z1; ...
                 x1 y1 z1; ...
                 x1 y1 z2; ...
                 x1 y2 z2; ...
                 x1 y2 z1; ...
                 x1 y2 z2; ...
                 x2 y2 z2; ...
                 x2 y2 z1; ...
                 x2 y2 z2; ...
                 x2 y1 z2; ...
                 x2 y1 z1; ...
                 x2 y1 z2; ...
                 x1 y1 z2];
end

%% Physics Functions
function [P, MT, ST] = JumpODE(t,p,c, Constants)
    %This is the ODE function for the first case, in determining a perfect
    %jump. So far, the gravity of earth is the only data in the c input,
    %and Constants's layout can be seen in the motorFcn Function.
    
    
    % Input data
    Weight = Constants(4);
    g_e = c(1);
    
    %Assign easy derivatives for a simple jump
    xdot = p(4); ydot = p(5); zdot = p(6);
    xddot = 0; yddot = 0;
    
    % Send data to motor torque function to get torques
    [MT, ST] = TorqueSolver(p(3), Constants); %Determining Torques
    SumTorq = ST + MT; %Sum of torques applying force to user
    
    % Determine sum of forces, and divide by mass to get acceleration
    F_Applied = SumTorq/Constants(3); %Torques transfered to weights
    sumF = -Weight + F_Applied; %Sum of forces on user
    zddot = sumF / (Weight/g_e); %F=ma, a = [in/s^2]
    
    P = [xdot; ydot; zdot; xddot; yddot; zddot];

end

function [MT, ST, UT] = TorqueSolver(z, Constants)
    %This is largely a placeholder until actual control laws are
    %determined. For now, motor force will just perfectly equal ideal case.
    % MT = motor torque [in*lbs]
    % ST = spring torque [in*lbs]
    
    %Input data for basic computation of known quantities
    IPPT = Constants(1); % [in*lbf / rev]
    ST_0 = Constants(2); % [in*lbf]
    R_Spool = Constants(3); % [in]
    W = Constants(4); % [lbf]
    h_max = Constants(5); % [in]
    g_e = Constants(6); % [in/s^2]
    g_m = Constants(7); % [in/s^2]
    theta_offset = Constants(8); % [rad]
    user_h = Constants(9); % [in]

    z_c = z + (user_h/2); % position of the end of the cable

    % Calculate Spool Angular Position
    theta = (h_max - z_c) ./ R_Spool; % angular position of spool [rad]

    theta_spring = theta - theta_offset; % angular displacement of spring [rad]

    % Calculate Spring Torque
    ST = theta_spring .* (IPPT / (2*pi)); % Torque from spring [in*lbf]
    
    % Calculate torque from the User
    UT = W*R_Spool*(1-(g_m/g_e));

    % Calculate Motor Torque
    MT = UT - ST; %Motor Torque
end








