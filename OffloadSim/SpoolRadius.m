%% Spool Radius Analysis

close all; clear; clc;

%% Constant
ag = 1.62/9.81; %[unitless]
W = 300; % [lbf]
F_c = W*(1-ag); % [lbf]
ma = W * ag; % [lbf]
v = 5.67; % [ft/s]
a = 1.62 * 3.281; % [ft/s]
m_pulley = 2; % [lbf]
m_pulley = m_pulley/32.17; % [slug]

R = linspace(0.1/12,2/12,100); % [ft]

k_ft = (F_c.*(R.^2))./5; % [ft-lbf / rad]
k_in = k_ft * 12; % [in-lbf / rad]
IPPT = k_in * (2*pi); % [in-lbf / rotation]

% % Plot Spring Constant vs. Spool Radius
% figure;
% plot(R*12,IPPT);
% xlabel("Spool Radius [in]"); ylabel("IPPT [in-lbf / rotation]");
% ylim([0 90]); xlim([0 2]);


L = linspace(0,5,100);
% 
% theta = zeros(length(L),length(R));
% for i = 1:length(L)
%     for j = 1:length(R)
%         theta(i,j) = L(i)/R(j);
%         
% %         tau_cb(j) = 300 * R(j);
%         tau_sp(i,j) = k_ft(j) * theta(i,j); % spring torque
%         tau_c(i,j) = R(j) * F_c; % cable torque
% %         I = 0.5 * m_pulley * (R(j)^2);
%         tau_m(i,j) = W*R(j)*(1-(ag)) - tau_sp(i,j); % motor torque
%         RPM(j) = (v/R(j)) * ((60)/(2*pi)); % motor RPM
%     end 
% end

% % Plot Spring Torque vs. Distance from Top
% figure; hold on; subplot(3,1,1);
% plot(L,tau_sp(:,[1 10 20 30 40 50 60 70 80 90 100]));
% hold off;
% title("Spring Torque");
% xlabel("Distance from Top [ft]");
% ylabel("Spring Torque");
% R_legend = string(R([1 10 20 30 40 50 60 70 80 90 100]).*12);
% legend(strcat("R=",R_legend," in"));
% 
% % Plot Motor Torque vs. Distance from Top
% hold on; subplot(3,1,2);
% plot(L,tau_m(:,[1 10 20 30 40 50 60 70 80 90 100]));
% hold off;
% title("Motor Torque");
% xlabel("Distance from Top [ft]");
% ylabel("Motor Torque");
% legend(strcat("R=",R_legend," in"));
% 
% % Plot Cable Torque vs. Distance from Top
% hold on; subplot(3,1,3);
% plot(L,tau_c(:,[1 10 20 30 40 50 60 70 80 90 100]));
% hold off;
% title("Cable Torque");
% xlabel("Distance from Top [ft]");
% ylabel("Cable Torque");
% legend(strcat("R=",R_legend," in"));

% % Plot Motor RPM vs Spool Radius
% figure; hold on;
% plot(R*12,RPM);
% hold off;
% title("Max Motor RPM");
% xlabel("Spool Radius [in]");
% ylabel("Max Motor RPM");
% % legend(strcat("R=",R_legend," in"));

RadSelect(L,F_c,m_pulley,ag,v,W);
% KnownSpring(L,F_c,ma,v);

%% Radius Selector
function RadSelect(L,F_c,m_pulley,ag,v,W)
    R = input("Insert desired spool radius (in inches): ")/12;
%     k_ft = (F_c.*(R^2))./5; % [ft-lbf / rad]
    k_ft = (W*(R^2)*(1-ag)) ./ 5; % [ft-lbf / rad]
    k_in = k_ft * 12; % [in-lbf / rad]
    IPPT = k_in * (2*pi); % [in-lbf / turn]
    theta_min = 0; theta_max = L(end)/R;
    N_turns = theta_max / (2*pi);
    tau_sp_max = k_ft * theta_max;
    tau_sp_min = k_ft * theta_min;
    tau_c = R * F_c;
    I = 0.5 * m_pulley * (R^2);
    tau_m_max = W*R*(1-(ag)) - tau_sp_min;
    RPM = (v/R) * ((60)/(2*pi));
    
    fprintf("Spring Constant k: %f [in-lb/rad] %f [IPPT]\nMax Spring Torque: %f [ft-lbf]\nMax Number of Pulley Turns: %f\nMax Motor Torque: %f [ft-lbf]\nMotor RPM: %f\n",k_in,IPPT,tau_sp_max,N_turns,tau_m_max,RPM);
end





