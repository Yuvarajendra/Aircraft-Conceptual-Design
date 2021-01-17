function [Dtotal,T,Di,D_parasitic,CL]=DRAG2(mach,rho,u,a)

%% lab 4 drag estimation of UAV (Medico-trans)
% This script computes the parasitic drag using component drag build up
% method and also estimation of the induced drag and the thrust line as a
% variation of velocity in mach number

%% environmental variables
u       =1.7723*10.^-5;       %dynamic viscosity
v	    =u./rho;              %kinematic viscositydensity by rho
rho0    =1.225;               %density at MSL
vcruise =30;                  %cruise velocity 1
Vcruise2=32;                  %cruise veloicty 2
Vapp    =22;                  %UAV approach speed in m/s
altitude=linspace(0,5,50);    %Altitude in kilometres
V=linspace(5,50,50);          % Array of velocities in m/s

%% Parasitic drag component estimation
% Wetted area calculation

% Fuselage
L1=0.3;                  %cockpit section
L2=0.96;                 %passenger section
L3=0.2;                  %rear upsweep section
Dfuse=0.254;
d1=0.254;
d2=0.020;                %averaged diametere at fuselage tip
fuse_length=0.3;
Fuse_width=0.254;

 Swet_fuse_1  =(((pi*Dfuse)*(12*L1).^-1)*((4*L1.^2)+((Dfuse.^3)*8.^-1))-((pi*Dfuse.^2)/4));
 Swet_fuse_2  =(pi*Dfuse*L2);
 Swet_fuse_3  =((pi*(d1+d2)*0.5)*sqrt((L3.^2)+(((d1.^2)-(d2.^2))/4)));
   
 Swet_fuse    = Swet_fuse_1+Swet_fuse_2+Swet_fuse_3;

% Main wing
Sref     =0.5351;   %reference area for all calculations is the wing area
Cr_wing= 0.2561;
lambda_wing=0.6;
MAC_wing =Cr_wing*(2/3)*((1+lambda_wing+(lambda_wing.^2)).*(1+lambda_wing).^-1);
tc_wing  =0.15;
Sw_wing= 0;                 % Sweep angle

Swet_wing=(2-(0.15/3))*Sref;  % the airfoil is NACA 23015 with 15 pc thickness( ref anatomy of an airplane(Darrol stinton)

% Horizontal tail
Sht=0.1312;
lambda_ht=0.6;
Cr_ht=0.188;
MAC_ht=Cr_ht*(2/3)*((1+lambda_ht+(lambda_ht.^2)).*(1+lambda_ht).^-1);
Swet_ht=(2+(0.12/3))*Sht;  % for NACA 0012
Sw_ht=0;                   % Sweep angle

% Vertical tail
Svt=0.0965;
lambda_vt=0.6;
Cr_vt=0.1942;
MAC_vt=Cr_vt*(2/3)*((1+lambda_vt+(lambda_vt.^2)).*(1+lambda_vt).^-1);
Swet_vt=(2+(0.12/3))*Svt;    % for NACA 0012
Sw_vt=40;                    % Sweep angle
TC_HT=0.12;
TC_VT=0.12;

% Nacelle
nacelle_length =0.0050;
nacelle_width  =0.0050;
Swet_nacelle=0.0025;   

% Pylon
Cr_pylon=0.004;
Ct_pylon=0.004;

MAC_pylon=0.004;
Sw_pylon=0;
TC_pylon=0.12;
Spylon=0.0005;                                                                                                     %%%
Swet_pylon=(2+(0.12/3))*Spylon;                                                                                  %%%

Swet=[Swet_wing,Swet_fuse,Swet_ht,Swet_vt,Swet_pylon,Swet_nacelle];                                            %%%%
                                

% Reynolds number calculation
 for x=1:length(a)                  
     for y=1:1:length(V)
         rh(y,x)=rho(1,y); %% Converting the row array into a column matris
     end
 end
rho=rh;

Re_wing=(rho.*vcruise*MAC_wing).*u.^-1;
Re_HT=(rho.*vcruise*MAC_ht).*u.^-1;
Re_VT=(rho.*vcruise*MAC_vt).*u.^-1;
Re_pylon=(rho.*vcruise*MAC_pylon).*u.^-1;
Re_fuselage=(rho.*vcruise*fuse_length).*u.^-1;
Re_nacelle=(rho.*vcruise*nacelle_length).*u.^-1;
Re=[Re_wing;Re_fuselage;Re_HT;Re_VT;Re_pylon;Re_nacelle];

altitude=linspace(0,5,50);
V=linspace(5,50,50);
machsweep=V/a;

% Skin friciton calculation

Cf_wing    =0.455*((log10(Re_wing).^2.58)*(1+(0.144*machsweep.^2))).^-1;
Cf_fuselage=0.455*((log10(Re_fuselage).^2.58)*(1+(0.144*machsweep.^2))).^-1;
Cf_HT      =0.455*((log10(Re_HT).^2.58)*(1+(0.144*machsweep.^2))).^-1;
Cf_VT      =0.455*((log10(Re_VT).^2.58)*(1+(0.144*machsweep.^2))).^-1;
Cf_nacelle =0.455*((log10(Re_nacelle).^2.58)*(1+(0.144*machsweep.^2))).^-1;
Cf_pylon   =0.455*((log10(Re_pylon).^2.58)*(1+(0.144*machsweep.^2))).^-1;

Cf         =[Cf_wing;Cf_fuselage;Cf_HT;Cf_VT;Cf_pylon;Cf_nacelle];


% Form factor calculation

xc_wing=0.25*MAC_wing;
xc_Htail=0.25*MAC_ht;
xc_Vtail=0.25*MAC_vt;
xc_pylon=0.25*MAC_pylon;

for n=1:length(machsweep)
FF_wing(n)  =(1+((0.6*(xc_wing).^-1).*(tc_wing))+(100*(tc_wing.^4))).*(1.34*(machsweep(n).^0.18)*(cos(deg2rad(Sw_wing)).^0.28));
FF_Htail(n) =(1+((0.6*(xc_Htail).^-1).*(TC_HT))+(100*(TC_HT.^4))).*(1.34*(machsweep(n).^0.18)*(cos(deg2rad(Sw_ht)).^0.28));
FF_Vtail(n) =(1+((0.6*(xc_Vtail).^-1).*(TC_VT))+(100*(TC_VT.^4))).*(1.34*(machsweep(n).^0.18)*(cos(deg2rad(Sw_vt)).^0.28));
FF_pylon(n) =(1+((0.6*(xc_pylon).^-1).*(TC_pylon))+(100*(TC_pylon.^4))).*(1.34*(machsweep(n).^0.18)*(cos(deg2rad(Sw_pylon)).^0.28));
Sr_fuselage=fuse_length/Fuse_width;
Sr_nacelle=nacelle_length/nacelle_width;

FF_fuse(n)=(1+(60*Sr_fuselage.^-3)+(Sr_fuselage*400.^-1));
FF_nacelle(n)=1+(0.35*Sr_nacelle.^-1);
end

FF=[FF_wing;FF_fuse;FF_Htail; FF_Vtail ;FF_pylon;FF_nacelle];

% Interference factor calculation
Qc_fuse    =1.0;
Qc_wing    =1.0;
Qc_nacelle =1.3;
Qc_Htail   =1.06;
Qc_pylon   =1;
Qc_Vtail   =1.06;

Qc=[Qc_fuse;Qc_wing;Qc_nacelle;Qc_Htail;Qc_Htail];

% Parasitic drag calculation
CD0_wing=Cf_wing.*FF_wing.*Qc_wing.*Swet_wing.*Sref.^-1;
CD0_fuselage=Cf_fuselage.*FF_fuse.*Qc_fuse.*Swet_fuse.*Sref.^-1;
CD0_htail=Cf_HT.*FF_Htail.*Qc_Htail.*Swet_ht.*Sref.^-1;
CD0_vtail=Cf_VT.*FF_Vtail.*Qc_Vtail.*Swet_vt.*Sref.^-1;
CD0_nacelle=Cf_nacelle.*FF_nacelle.*Qc_nacelle.*Swet_nacelle.*Sref.^-1;
CD0_pylon=Cf_pylon.*FF_pylon.*Qc_pylon.*Swet_pylon.*Sref.^-1;

CD0=(CD0_wing+CD0_fuselage+CD0_htail+CD0_vtail+CD0_nacelle+CD0_pylon);

D_parasitic=CD0.*(0.5*rho.*V.^2*Sref); %% Parasitic drag in Newtons

MTOW=15;        %% Guessed MTOW
W=MTOW*9.81;
CL=W*(0.5.*rho.*V.^2*Sref).^-1;

AR_wing=8;
Sw_wing_LE=rad2deg(tan(deg2rad(Sw_wing)) + (4./AR_wing)*((0.25).*((1-lambda_wing).*(1+lambda_wing).^-1)));                                           
e=0.8106;
CDi=(CL.^2)*(pi*e*AR_wing).^-1; %% Lift induced drag coefficient
Di=CDi.*(0.5.*rho.*V.^2*Sref);  %% Lift induced drag in Newtons


CD0_1=[CD0_wing,CD0_fuselage,CD0_htail,CD0_vtail,CD0_nacelle,CD0_pylon];
CDexcer=1.10*sum(CD0_1);        %% Adding 10% extra excerscence drag

CD=CD0+CDi;%+CDexcer;
D=0.5.*rho.*V.^2*Sref.*CD;
Dtotal=D_parasitic+Di;

To=96; % thrust prosuced from electrical motors in UAV (2 engine) in Newtons
T=To.*(rho.*(rho0.^-1))*(1-(0.25.*machsweep)) ; %% Variation of thrust required with altitude
end

