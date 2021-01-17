close all;
clear all;
clc;

g=9.81;
MTOW = 15;
Weight=MTOW*g;
Sref=0.5351;
Swet=Sref*2*1.1;            % Twice the reference area of the wing with 10% extra area for curved edges
Density_SL=1.225;
Cl_max=1.3;

%%% Basic V-n diagram
%% Establishing load factors
n_posit=2.1 + ((24000)/(Weight+10000));

if(n_posit > 3.80)          %%  Limit posed according to the Riga technical university paper on V_n diagram
 n_posit=3.8;
end

n_negat= -0.4 *n_posit;

%% Design cruising speed
V_c =
%% V-n diagram points allocation

Fx = V_dive;      Fy = n_posit;         %% Positive load factor structural damage point
Gx = V_dive;      Gy = n_negat;         %% Negative load factor structural damage point
Ax = V_stall;     Ay = 1;               %% Normal stall speed point positive load factor
Bx = V_man_posit; By = n_posit;         %% Positive Maneuvering speed point 
Kx = V_si;        Ky = -1;  
Jx = V_man_negat; Jy = n_negat;         %% Negative Maneuvering speed point 
Ox = 0;           Oy =0;                %% Origin


plot(Fx,Fy,'-o','LineWidth', 2)
hold on
plot(Gx,Gy,'-o','LineWidth', 2)
hold on
plot(Ax,Ay,'-o','LineWidth', 2)
hold on
plot(Bx,By,'-o','LineWidth', 2)
hold on
plot(Kx,Ky,'-o','LineWidth', 2)
hold on
plot(Ox,Oy,'-o','LineWidth', 2)
hold on
plot(Jx,Jy,'-o','LineWidth', 2)
hold on
line([Bx, Fx], [By, Fy], 'Color', 'r','LineWidth', 2); % Line between point B and point F.
line([Fx, Gx], [Fy, Gy], 'Color', 'r','LineWidth', 2); % Line between point F and point G.
line([Gx, Jx], [Gy, Jy], 'Color', 'r','LineWidth', 2); % Line between point G and point J.
hold on

% plotting upper spline (OAB)
x = [Ox Ax Bx];
y = [Oy Ay By];
%Cubic spline data interpolation
t = 1:numel(x);
xy = [x;y];
pp = spline(t,xy);
tInterp = linspace(1,numel(x));
xyInterp = ppval(pp, tInterp);
hold on
plot(xyInterp(1,:),xyInterp(2,:), 'Color', 'r','LineWidth', 2)
xlim([0 50])
ylim([-4 5])
hold on

% plotting lower spline (OKJ)
x2 = [Ox Kx Jx];
y2 = [Oy Ky Jy];
%Cubic spline data interpolation
t2 = 1:numel(x2);
xy2= [x2;y2];
pp2 = spline(t2,xy2);
tInterp2 = linspace(1,numel(x));
xyInterp2 = ppval(pp2, tInterp2);
hold on
plot(xyInterp2(1,:),xyInterp2(2,:), 'Color', 'r','LineWidth', 2)
xlabel('Speed of UAV in m/s')
ylabel('Load factor')
title('Basic V-n diagram')
grid on ;


%% Gust V-n diagram

% Aircraft load at sea level
AR=8;
b=2.10;
c=b/AR;