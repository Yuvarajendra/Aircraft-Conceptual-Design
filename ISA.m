function [mu,T,a,P,rho]= ISA(rohith)
g=9.81;  % ac du to graity m/s2
gamma=1.4; %ratio of specific heat  
rho0=1.225; %kg/m3
P0=101325; % newton/m2
T0=288.15; %kelvin temperature at sea level
mu0=1.789*10.^-5; % viscoisty at Sea level
R=287; % gas constant
a0=340.294; % speed of sound at sea level
%% lapse rate in K/km 
B1=-6.5;       %0-11 km
B2=0;          %11-20 km
B3=1;          %20-32
B4=2.8;        %32-47
B5=0;          %47-51
B6=-2.8 ;      %51-71
B7=-2;         %86-71
B=6.5;
z=rohith;
for i=1:length(z)     
    if z(i)<=11
        z0=0;
        T(i)=T0+(B1.*(z(i)-z0));
        T11=T(end);
        
    elseif z(i)<=20
        z11=11;
        T(i)=T11+(B2.*(z(i)-z11));
        T20=T(end);
        
    elseif z(i)<=32
        z20=20;
        T(i)=T20+(B3.*(z(i)-z20));
        T32=T(end);
        
    elseif z(i)<=47
        z32=32;
        T(i)=T32+(B4.*(z(i)-z32));
        T47=T(end);
        
    elseif z(i)<=51
        z47=47;
        T(i)=T47+(B5.*(z(i)-z47));
        T51=T47(end);
        
    elseif z(i)<=71
        z51=51;
        T(i)=T47+(B6.*(z(i)-z51));
        T51=T(end);
        z71=71;
        
    elseif z(i)>71
        T(i)=T51+(B7.*(z(i)-z71));
 
    end
end


 Bp  =0.0065;
 P   = P0.*(1-(Bp.*(z*1000).*T0.^-1)).^(g*(R.*Bp).^-1);
 P_pa=P*10.^5;
 a   = sqrt(gamma*R*T);
 mu  = (1.458*10.^-6)*((T.^1.5).*(T+110.4).^-1 ) ;
 rho =  P.*((R.*T).^-1);


end


