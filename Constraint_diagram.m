close all;
clear all;
clc;

%% Author Yuvarajendra Anjaneya Reddy (nakkan naaaaaaaaaaaaaaane)
neta=0.85
g=9.81;
Sg=500;                             %% Ground roll of about 100 ft
AR=8;                               %% Aspect ratio
b=2.10;                             %% Wing span
S=(b^2)/AR;                         %% Wetted area
MTOW=13*9.81;                       %% Total weight
Density=1.08791;                     %% Density at 4000 ft
Density_SL=1.225;
Cl_cruise=0.7;                      %% Cruise coefficient of lift
Cl_max=1.4;
Cd_cruise=0.027;                    %% Cruise coefficient of drag
Cd_TO=0.040;                        %% Take-off coefficient of drag
Cl_TO=0.7;                          %% Take-off coefficient of lift
V_cruise=sqrt((2*MTOW)/(Density*S*Cl_cruise));
V_service_ceiling=0.508;            %% Vertical speed in 100 ft/min
e=1.78*(1-0.045*(8)^0.68)-0.64;     %% Oswalds efficiency factor
k=1/(pi*e*AR);                      %% Lift induced drag factor
q=(0.5*Density*V_cruise^2);         %% Dynamic pressure
n=2;                                %% Load factor
V_vertical_speed=3.048;             %% Vertical speed in 10 ft/sec
V_stall=sqrt((2/Density)*(MTOW/S)*(1/Cl_max))               %% Stall speed
V_LOF=1.1*V_stall;                  %% Lift-off speed, 110% of stall speed
q_takeoff=(0.5*Density_SL*(V_LOF/sqrt(2))^2);
mu=0.04;                            %% Ground friction coefficient
sigma=Density/Density_SL;
V_approach=22;

W_S_landing=0.5*Cl_max*Density_SL*V_approach^2;


W_S=linspace(0,450,30);
T_W_landing=linspace(0,0.08,30);

for(i=1:1:length(W_S))
    
T_W_cruise_speed(i)=(q*Cd_cruise*(1/W_S(i))+k*(1/q)*(W_S(i)))*(V_cruise/(neta*746))/(1.132*sigma-0.132);

T_W_Climb(i)=((V_vertical_speed/V_cruise)+(q/W_S(i))*Cd_cruise+(k/q)*(W_S(i)))*(V_cruise/(neta*746))/(1.132*sigma-0.132);

T_W_constant_veloc_turn(i)=(q*((Cd_cruise/W_S(i))+k*(n/q)^2 *(W_S(i))))*(V_cruise/(neta*746))/(1.132*sigma-0.132);

T_W_service(i)= ((V_service_ceiling/sqrt((2/Density)*W_S(i)*sqrt(k/(3*Cd_cruise))))+4*sqrt((k*Cd_cruise)/3))*(V_cruise/(neta*746))/(1.132*sigma-0.132);

T_W_takeoff(i)=(((V_LOF^2)/(2*g*Sg))+((q_takeoff*Cd_TO)/W_S(i)) + mu*(1-((q_takeoff*Cl_TO)/W_S(i))))*(V_cruise/(neta*550))/(1.132*sigma-0.132);

W_S_landings(i)=W_S_landing;
end



plot(W_S,T_W_cruise_speed,'Linewidth',2)
hold on
plot(W_S,T_W_Climb,'Linewidth',2)
hold on
plot(W_S,T_W_constant_veloc_turn,'Linewidth',2)
hold on
plot(W_S,T_W_service,'Linewidth',2)
hold on
plot(W_S,T_W_takeoff,'--','Linewidth',2)
hold on
plot(W_S_landings,T_W_landing,'Linewidth',2)

xlabel('Wing loading [N/m^2]')
ylabel('Power to Weight ratio [BHP/N]')
legend('Cruise speed','Rate of climb','Constant velocity turn','Service ceiling','Take-off')


%%% Stall speeds
Stall_speeds=25;
% for(k=1:1:length(W_S))
%     
% for(j=1:1:length(Stall_speeds))    
Cl_maxx=(W_S)*(1/(0.5*Density*Stall_speeds^2));
% end

% end
yyaxis right;
plot(W_S,Cl_maxx,'k','Linewidth',1.5)
ylim([0 5.5])

ylabel('Required Clmax')
title('Constraint diagram')
legend('Cruise speed= 30m/s','Rate of climb= 3m/s','Constant velocity turn','Service ceiling','Take-off, Vlof= 26m/s','Landing, Vapp =22m/s','Vstall=20 m/s')%,'Vs=30 m/s','Vs=35 m/s','Vs=40 m/s')