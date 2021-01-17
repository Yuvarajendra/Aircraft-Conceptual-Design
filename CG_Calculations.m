%% Longitudinal CG calculation
Wing_wt=0.710; %% All weights in kilogrammes
Empennage=0.164;
Fuse_wt=0.714+0.192+0.046;
ESC1_wt=0.028;
ESC2_wt=0.028;
Engine1_wt=0.098;         %% Survey results value
Engine2_wt=0.098; 
Battery_wt= 0.191 ;                  %% Survey results value
Servo_motors_wing=0.023*2;
Servo_motor=0.023;
Receiver =0.050;           %% Mission specific parameter
Wtotal= Wing_wt+ Empennage + Fuse_wt +ESC1_wt+ ESC2_wt+Engine1_wt+Engine2_wt+Servo_motor + Servo_motors_wing +Receiver+ Battery_wt;
Fuselage_length=1.25;   %% 55pc of wing span (from some fucking source)

Xw=0.3488*Fuselage_length;         %% Initial guesses of the location of Cg from the nose of fuselage
Xemp=0.99*Fuselage_length;
Xfuse=0.28*Fuselage_length;
Xesc=0.20*Fuselage_length;
Xengi=0.45*Fuselage_length;
Xbatt=0.10*Fuselage_length;
Xservo_ail=0.32*Fuselage_length;
Xservo_pay=0.20*Fuselage_length;
Xrece=0.20*Fuselage_length;

Xcg=( Wing_wt*Xw +  Empennage*Xemp + Fuse_wt * Xfuse + (ESC1_wt+ESC2_wt)*Xesc + (Engine1_wt+Engine2_wt)*Xengi + (Battery_wt)*Xbatt +(Servo_motors_wing)*Xservo_ail +(Servo_motor)*Xservo_pay +Receiver*Xrece )/Wtotal

Xcg_percent_fuselage=(Xcg)/Fuselage_length   %% CG in terms of percentage of fuselage length


% Zw=0.45*Fuselage_length;         %% Initial guesses of the location of Cg from the nose of fuselage
% Zht=0.90*Fuselage_length;
% Zvt=0.85*Fuselage_length;
% Zfuse=0.4*Fuselage_length;
% Zesc=0.47*Fuselage_length;
% Zengi=0.45*Fuselage_length;
% Zbatt1=0.28*Fuselage_length;
% Zbatt2=0.28*Fuselage_length;
% Zservo_ele=0.91*Fuselage_length;
% Zservo_rudder=0.91*Fuselage_length;
% Zservo_aile=0.60*Fuselage_length;
% Zautop=0.32*Fuselage_length;
% Zrece=0.40*Fuselage_length;
% Zparachute=0.40*Fuselage_length;
% Zpay=0.4009*Fuselage_length;
% 
% Zcg=( Wing_wt*Zw +  HT_Wing_wt*Zht + VT_Wing_wt*Zvt + Fuse_wt*Zfuse + (ESC1_wt+ESC2_wt)*Zesc + (Engine1_wt+Engine2_wt)*Zengi + (Battery_wt*2)*Zbatt1 +(Battery_wt*2)*Zbatt2 +(Servo_motors*2)*Zservo_ele+Servo_motors*Zservo_rudder+(Servo_motors*2)*Zservo_aile+ Autopilot*Zautop +Receiver*Zrece+Payload*Zpay +Parachute_pod*Zparachute)/Wtotal
% 
% Zcg_percent_fuselage=(Zcg)/Fuselage_length   %% CG in terms of percentage of fuselage length
% 
% 
