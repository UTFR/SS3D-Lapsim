function [Points_total] = FSAE_Simulator(m,cofg,w,track,WD,ETS,Fuel_corr,C_Drag,C_DF)% m=220;          %mass in kg
% cofg=0.27;      %center of gravity height in m
% w=1.6;          %wheelbase in m
% track=1.2;      %trackwidth in m
% WD=0.5;         %percent rear distribution
% ETS=1;          %percent of engine torque
% Fuel_corr=6;    %fuel correction factor
% 
% A_Fr=0.8;       %frontal area (m^2)
% A_Plan_wings=1.5;   %wing planform area (m^2)
% C_DF=0;         %Coeff of Downforce
% C_Drag=0.7;     %Coeff of Drag
% air_dens=1.2;   %air density (kg/m^3)
% 
% C_Drag=0.5*C_Drag*air_dens*A_Fr;         %new C_Drag satisfies DF=C_Drag*v^2
% C_DF=0.5*C_DF*air_dens*A_Plan_wings;     %new C_DF satisfies DF=C_DF*v^2


%Skidpad 
t_min_skidpad=4.847;
r_corner=8.5;
distance=8.5*2*pi;
[t_skidpad,v_skidpad]=SScorner(m,cofg,w,track,WD,C_DF,r_corner,distance);
Points_skidpad=47.5*((1.25*t_min_skidpad/t_skidpad)^2-1)/0.5625+2.5;
if t_skidpad > 1.25*t_min_skidpad 
    Points_skidpad=2.5; 
end

%Acceleration
t_min_accel=4.131;
t_accel=AccelEvent( m, cofg, w,track, WD, C_DF, C_Drag, ETS);
Points_accel=71.5*(1.5*t_min_accel/t_accel-1)/0.5+3.5;
if t_accel > 1.5*t_min_accel
    Points_accel=3.5;
end

%Autocross and Endurance
t_min_endurance=42; %per lap - 1195.4/20;
[t_endurance,E_total]=endurance_calculator(m,cofg,w,track,WD,C_DF,C_Drag,ETS);
Points_autocross=142.5*(1.45*t_min_endurance/t_endurance-1)/0.45+7.5;
Points_endurance=250*(1.45*t_min_endurance/t_endurance-1)/0.45+50;
if t_endurance > 1.45*t_min_endurance 
    Points_autocross=7.5;
    Points_endurance=22;
end
    
%Fuel Economy
Fuel_min=1.8;    %93 octane in L (xFactor due to shorter endurance)
CO2_min=Fuel_min*2.31;
E_total=E_total*3*Fuel_corr*20;        %scaled due to thermal efficiency and # of laps
CO2_equiv=E_total/(32.4*10^6)*2.31;      %CO2 equivalent in kg (E(Joules) multiplied gasoline energy density (MJ/L))

FEF_min=0.272;      %2015 FSAEM results
FEF_max=0.800;      %2015 FSAEM results
FEF=t_min_endurance/t_endurance*CO2_min/CO2_equiv;

Points_efficiency=100*(FEF_min/FEF-1)/(FEF_min/FEF_max-1);
if (CO2_equiv  >60.06 || t_endurance > 1.45*t_min_endurance)
    Points_Efficiency=0;
end


%TOTAL POINTS
Points_total=Points_skidpad+Points_accel+Points_autocross+Points_endurance+Points_efficiency;
Grade=Points_total/675*100;
end
