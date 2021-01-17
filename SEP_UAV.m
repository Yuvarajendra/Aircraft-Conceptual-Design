clear all
close all
clc

% Rohith Prem Maben(rohma417)
% Yuvarajendra Anjaneya Reddy(yuvan983)
tic
g=9.81;                                       %Acceleration due to gravity
rho0=1.225;                                   %Density at MSL
MTOW=15;                                      %MTOW of the aircraft
Sref=0.5351;                                  %refernce wing area
CLmaxTo=1.3;                                  %Occurs at stalling velocity
CLmaxLo=1.1;  
CLcruise=0.7;                                %Calculated for cruise speed
statThr=101;                              %For 2 enginesof Fokker 70
servCeil=1220;                               %From internet reference
altitude=linspace(0,5,50);
V=linspace(5,50,50);
mach_limit=NaN*ones(1,50);
VV=linspace(5,50,50);
vcruise_design=30;
Weight=MTOW*9.8;
a=335.582;                                  %airspeed at 4000 ft
V_topspeed= 40;                             %top speed in m/s
mach_max=V_topspeed/a;                      %From AC data table
mach_cruise=vcruise_design/a;

 %% calling atmosphere function
[mu,T,a,P,rho]= ISA(altitude);       
u=mu;

%% computing drag and thrust for various altitudes

 for i=1:length(altitude)
     for j=1:length(V)
          mach(i,j)=V(j)./a(i);
          Vsquare(i,j)=V(j).^2.*a(i).^2;
          [Dtotal,T,Di,D_parasitic,CL]=DRAG2(mach,rho,u,a);
          Mne(1,i)=0.84;                         % Never exceed mach number
     end
 end 
 
 
 %% Calculating Dynamic pressure
q=0.5*rho.*vcruise_design.^2;
qmax=max(q);
V_qmax=sqrt(2*qmax.*(rho).^-1);
mq=V_qmax./a;  % mach trim 

 %% Calculating stall velocities
 Vstall_Lo=sqrt((2.*MTOW.*9.81)./(rho.*CLmaxLo.*Sref));  
 Vstall_To=sqrt((2.*MTOW.*9.81)./(rho.*CLmaxTo.*Sref));
 for i=1:1:length(altitude)                  
     for j=1:1:length(V)
        Vst(i,j)=Vstall_To(j);
        V(i,j)=V(1,j);
        V_qm(i,j)=V_qmax(j);
        M_qm(i,j)=mq(j);
        altitud(i,j)=altitude(j);
     end
 end
 
Vstall_To=Vst.';
Mach_stall=Vstall_To./a;
V_qmax=V_qm.';
Mach_stall_To=Vstall_To./a;

%% SEP calculation

Preq=Dtotal.*V;                     % Required power calculation
Pa=T.*V;                            % Available power calculation
SEP=(Pa-Preq)./Weight;              % Specific excess power calculation
Ps=V./Weight.*(T-Dtotal);
SEP2=SEP;
SEP3=SEP;
he=altitude+(V.*V/2./g);            %Specific energy calculation
 
 
 %% 3 e)Limiting the contour for high coefficient of lift
for i=1:1:length(a)                  
      for j=1:1:length(V)
            if(V(i,j)<Vstall_To(i,j))
                SEP2(i,j)=NaN;
            else
             SEP2(i,j)=SEP(i,j);
            end
      end
end
Vstall_Lo2=Vstall_Lo(:,1)';
V2(i,j)=V(i,j); 

 %% 3 c) airspeed limitation of 250 kts(128.611) upto 3000 m

 for i =1:1:10
     mach_limit(i)=0.378;
 end
 
 for i=1:1:length(a)
       for j=1:1:length(V)
             if SEP3(i,j)>=128.611%mach_limit(i)
                  SEP3(i,j)=NaN;
             else 
                 SEP3(i,j)=SEP2(i,j);
             end
       end
  end
   
SEP4=SEP3;
Mach_SEP=SEP4./a;  
V_qmax2=V_qmax(:,1)';
Mach_stall_Lo2=Vstall_Lo2./a;

% trimming the curve for high lift coefficient configuration( Take off and
% landing condition curve)
for i=1:1:length(a)
    if Mach_stall_Lo2(i)<=0.343
        Mach_stall_Lo2(i)=Mach_stall_Lo2(i);
    else
       Mach_stall_Lo2(i)=NaN;
    end
end

for i=1:1:length(a)
    if Mach_stall_To(i)<=0.283
        Mach_stall_To(i)=Mach_stall_To(i);
    else
       Mach_stall_To(i)=NaN;
    end
end
% for dynamic pressure right side contour trim
   for i=1:1:length(a)                  
       for j=1:1:length(V)
             if((V(i,j)./a)<Mne)
                  V2(i,j)=NaN;
             else
                  SEP4(i,j)=NaN;
                
         end
      end
    end
%%  Limting for maximum dynamic pressure
qmax_mach=V_qmax2./a;

for i=1:1:length(a)
    if qmax_mach(i)<=0.85%mach_max
        qmax_mach(i)=qmax_mach(i);
    else
        qmax_mach(i)=NaN;
    end
end 
   %% cruise condition curve
   CL_cr=0.84;
   V_cruise_SEP=sqrt((2*Weight).*(rho.*Sref.*CL_cr).^-1);
   machcruise=V_cruise_SEP./a;
   
   % trim the cruise curve 
  for i=1:1:length(a)
    if machcruise(i)<=0.6726
       machcruise(i)=machcruise(i);
    else
       machcruise(i)=NaN;
    end
  end
   
   

%% mach limit
   Mno=0.77*ones(1,45);
   
   
%%Optimal climb path
% Time optimal climb path
  
  for i=1:length(altitude)
      
       Optimal_path(i)=max(SEP4(i,:));    
       [x,y]=find(Optimal_path(i)==(SEP4(i,:)));
       mac_optimal(i)=mach(x,y);
      %Optimal_mach(i)=ind2sub(max(SEP(i,:)));
    
  end
 
  for i=1:length(altitude)
      if(altitude(i)<3)                     
         mach_climbpath(i)=0.378;
      else
          mach_climbpath(i)=mac_optimal(i);
      end
  end
          
%% plots   
f=figure(1);
movegui(f,'northwest')
M=VV./a;


Vstall_plot=sqrt((2.*MTOW.*9.81)./(rho.*CLmaxTo.*Sref));

contour(VV,altitude,SEP,[8 7.5 7 6.5 6 5.5 5 4.5 4 3.5 3 2.5 2 1.5 1 0.5 0 -0.5 -1 -1.5 -2 -2.5 -3 -3.5 -4 -4.5 -5 -5.5 -6 -6.5 -7],'ShowText','on','LineWidth',2)
hold on
plot( Vstall_plot,altitude,qmax_mach.*a,altitude,machcruise.*a,altitude,'linewidth',2)%,'linewidth'
% hold on
% plot(Mne,altitude);                                %Mach number limit line plotting
% hold on
% % plot(mach_climbpath,altitude,'k','linewidth',3)     %optimal path plotting
% % hold on
% 
 legend('Specific excess power(m/s)','Stall speed','Max dynamic pressure','Cruise speed');%,'Never exceed Mach','Optimal Climb Path with airspeed restriction')
 axis([0 40 0 5])


title('Specific Excess Power, SEP (m/s)');
xlabel('Velocities in m/s');
ylabel('Altitude in kilometres');

