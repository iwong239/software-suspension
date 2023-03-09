%% Moon Gravity Calculation
gravityinput = 1.6;
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
% dist = zeros(1000,1);
dist1 = 0;
dist2 = 0;
Equilibriumdist = 0;%in
initialzstop = Equilibriumdist+(2*12); %in
lswitch = 0;
dt = 0.02;
i = 1;
acceldata = [10:-1.62*dt:-100];
distance = zeros(size(acceldata));
yd=0;
ydtemp = 0;
y=0;
ytemp = 0;
ydd = 0;
yddtemp = 0;
g=0;

if acceldata(1) >=5
    while (i <= 3396 & dist2 >=Equilibriumdist)
        if(dist2>=dist1)
            disttemp = dist1;
            dist1 = dist2;
            ydd = acceldata(i); % differential equation
            yd = ydtemp + dt*ydd;      %integral of ydd
            y = ytemp + dt*yd;         %integral of yd
            dist2 = disttemp+y;
        else
            ydd = acceldata(i);
            if (ydd < -1.8) || (ydd>-1.4)
                disttemp = dist1;
                dist1 = dist2;
                ydd = -1.62
                yd = ydtemp + dt*ydd;      %integral of ydd
                y = ytemp + dt*yd;         %integral of yd
                dist2 = disttemp+y;
                g=g+1;
            end
        end
        ydtemp = yd;
        ytemp = y;
        Spoolturn = ((dist2)/SpoolCir)*2*pi;%radians
        Springforce = orispringforce - (k*Spoolturn);
        Motorforce = offload - Springforce;%lbs
        i= i+1;
        distance(i) = dist2;
    end

end