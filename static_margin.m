clc
clear all
close all
 
% WEIGHTS
fprintf('\n Tail Size\t Fuselage Length  Static Margin\t  Weight\t  Est Wing Area \t  Req. Wing Area');
 
servo = .0025;  % all weights in slugs
motor = .0044;
gearbox = .0039;
prop = .0012;
v_stab = .0006;
fusel = 45;
fuse = .75*.114*(fusel/12)/32.2;
batt = .0039;
s_cont = .0012;
gyro = .0019;
rec = .0019;
fairing = .0053;
 
S_h = 250;                %in^2
h_tail = .000252*S_h*.0787;    %slugs
 
%AC Derivatives
 
cl_alh =2*pi;
cl_alwf=5.80;
eta_h=1;
% S=558.72;      %in^3
x_ach=fusel-4;
de_dal=.2;  %?
x_acwf = 8;
 
wingsize = 3.5:.01:5;
 
for i = 1:length(wingsize)
    wing(i) = (1/12)*1.5*wingsize(i)/32.2;
    S(i) = wingsize(i)*144;
    tweight(i) = prop + gearbox + motor + wing(i) + fairing + servo + batt + s_cont + gyro + rec + fuse + h_tail + v_stab;
    CG(i) = (prop*0 + gearbox*1.125 + motor*.5 + wing(i)*12 + fairing*12 + 4*servo*13 + batt*8 + s_cont*8 + gyro*12 + rec*11 + fuse*(fusel/2) + h_tail*(fusel-5) + v_stab*(fusel-2.5)) / ...
    (prop + gearbox + motor + wing(i) + fairing + 4*servo + batt + s_cont + gyro + rec + fuse + h_tail + v_stab)/12;
    wload(i) = tweight(i)*32.2 / .405;
    AC2(i) = ((x_acwf/12 + (cl_alh/cl_alwf)*eta_h*((S_h)/S(i))*x_ach/12*(1-de_dal))/(1 + (cl_alh/cl_alwf)*eta_h*((S_h)/S(i))*(1-de_dal)));
 
end
 
SM2 = (AC2-CG);
 
for i = 1:length(wingsize)
    fprintf('\n%6.2f \t        %6.2f        %6.2f\t      %6.4f \t      %6.2f\t     %6.2f', S_h/144, fusel/12, SM2(i), tweight(i)*32.2, wingsize(i), wload(i));
end
 
% fprintf('\n For a tail size of:  ')
% fprintf('%6.2f ft^2', S_h(167)/144)
% fprintf('\n Your computed Static Margin is: ')
% fprintf('%6.2f\n', SM2(167))
% fprintf('\n Your aircraft total weight is: ')
% fprintf('%6.4f lbf\n\n', tweight(167)*32.2)
 
 
% plot(S_h/144,CG)
% hold on
% plot(S_h/144,AC2,'--r')
% grid
% xlabel('Horizontal Tail Size Sh (ft^2)');
% ylabel('Xcg and Xac bar');
% legend('Xcg','Xac')
% title('X-Plot');
