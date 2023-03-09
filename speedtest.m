clear all;
clc;

tic
gravityinput = 1.6;
%% Moon Gravity Calculation
weight = 150;
mass = weight/9.8;
moonweight = mass*gravityinput;
offload = weight - moonweight;

%% Spool  and Spring Calculation

SpoolCir= 3.5*pi; %in
SpringCir = 4.05*2*pi;%in
MotorRadius = 2;%in
k = 5.5704;%in*lb/radian
orispringforce= 100;%lb
dist = zeros(1000,1);
dist(1) = 2.8*12;
Equilibriumdist = 2.7*12;%in
initialzstop = Equilibriumdist+(2*12); %in
lswitch = 0;





%% IMU Data
acc = [1.9 1.8 2 2.1 1.9 2 1.9];
z = [50.4 50.6 50.7 50.6 50.1];
averz = testSpeed(z);
averacc = testSpeed(acc);

%if the limitswitch is hit
if averz >= initialzstop
    lswitch = 1;
end
averacc = 100;
if(averacc > 0)

%     if (averacc < -1.8) || (averacc>-1.4)
        Spoolturn = ((averz-Equilibriumdist)/SpoolCir)*2*pi;%radians
        Springforce = orispringforce - (k*Spoolturn);
        %control laws
        s = tf('s');
        G = (s+2)/(s^2 + s + 2); % tranfer function
        dt = 0.02;
        i = 1;
        ud = zeros(1000,1);
        y = zeros(1000,1);
        yd = zeros(1000,1);
        ydd = zeros(1000,1);
        u = zeros(1000,1);
        while (dist(i) >= Equilibriumdist)
            if i == 1
            u(i) = averacc;
            ud(i) = 0;
            ydd(i) = averacc;
            yd(i) = 0;
            y(i) = 0;
%             accelerationeq = ud + 2*u - yd - 2*y;
            else
                if i<10
                    averacc = averacc-9.81;
                else
                    averacc = -9.81;
                end
                u(i) = u(i-1)+ud(i-1);
                ud(i) = (u(i) - u(i-1))/dt;         %ud = du/dt
                yd(i) = yd(i-1) + dt*ydd(i-1);      %integral of ydd
                y(i) = y(i-1) + dt*yd(i-1);         %integral of yd
                ydd(i) = averacc; % differential equation 
                
            end
            dist(i+1) = dist(i) + y(i);
            i = 1+i;
        end
        time = dt*i;
        dist;
        plot([1:1:1000],dist);
%         dt = 0.001;
%         t = 0:dt:10;
% 
%         u=ones(size(t));
%         u(1) = averacc;
% 
%         %initial conditions
%         y0= averacc;
%         yd0 = 0;
%         ud0 = 0;
% 
%         %predefine the size of input/output vectors
%         ud = zeros(size(t));
%         y = zeros(size(t));
%         yd = zeros(size(t));
%         ydd = zeros(size(t));
% 
%         for i = 1:length(t)
%             if i ==1
%                 ud(i) = ud0;
%                 y(i) = y0;
%                 yd(i) = yd0;
%                 ydd(i) = ud(i) + 2*u(i) - yd(i) - 2*y(i);
%             else
%                 ud(i) = (u(i) - u(i-1))/dt;         %ud = du/dt
%                 yd(i) = yd(i-1) + dt*ydd(i-1);      %integral of ydd
%                 y(i) = y(i-1) + dt*yd(i-1);         %integral of yd
%                 ydd(i) = ud(i) + 2*u(i) - yd(i) - 2*y(i); % differential equation
%             end
%         end
%         scatter(t,y,3);
%     end

else
    
    Spoolturn = ((averz-Equilibriumdist)/SpoolCir)*2*pi;%radians
    Springforce = orispringforce - (k*Spoolturn);%lbs
    Motorforce = offload - Springforce;%lbs
    if lswitch == 1
        Motorforce = Motorforce - softstop(averz, intialzstop);
    end

end
% MotorTorque = MotorF*MotorRadius;
% MotorCurrent = MotorTorque*.6;

toc

Gz = c2d   (G, 0.01, 'tustin')

function test = testSpeed(num)
test = sum(num(:))/numel(num);
end

