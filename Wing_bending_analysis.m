% Wing Bending Analysis
close all
clear all
clc;
% Assumes Elliptic Lift Distribution
W = 15*9.81; % weight
Span = 2.10;
n = 5;
SF = 1.5;
g = 9.81;
L = n*SF*W;
a = Span/2;
A = L/2;
b = 4*A/(pi*a);
n = 11;
x = linspace(0,a,n);
y = sqrt(b^2*(1-x.^2/a^2)); % Lift
for i = 1:length(x)-1
x_avg(i) = (x(i)+x(i+1))/2;
y_avg(i) = (y(i)+y(i+1))/2;
end
dx = x(2);
dl = x(2)/2;
n_dl = [1 3 5 7 9 11 13 15 17 19];
for j = 1:length(x_avg)
L_local(j) = dx*y_avg(j);
end
for k = 1:length(L_local)
M_local = L_local(k:length(L_local)).*(dl*n_dl(1:length(L_local)+1-k));
M_station(k) = sum(M_local);
end

M = L/2*4*a/(3*pi)
M_approx = M_station(1)
Half_Lift = A
Half_Lift_approx = sum(dx*y_avg)
plot(x,y,'o-')
hold on
bar(x_avg,y_avg)
title('Elliptic Lift Distribution and Discretized Approximation')
xlabel('Distance from Root [ft]')
ylabel('Lift [lbf]')
figure
plot(x(1:n-1),M_station,'o-')
grid on
title('Bending Moment vs. Span')
xlabel('Distance from Root [ft]')
ylabel('Bending Moment [ft-lb]')