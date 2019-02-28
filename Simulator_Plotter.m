 %initial parameters

m=220;          %mass in kg
cofg=0.27;      %center of gravity height in m
w=1.6;          %wheelbase in m
track=1.2;      %trackwidth in m
WD=0.55;        %percent rear distribution
ETS=1;          %percent of engine torque
Fuel_corr=6;    %fuel correction factor

A_Fr=0.8;       %frontal area (m^2)
A_Plan_wings=1; %wing planform area (m^2)
C_Dforce=0;         %Coeff of Downforce
C_Dr=0.8;     %Coeff of Drag
air_dens=1.2;   %air density (kg/m^3)

C_Drag=0.5*C_Dr*air_dens*A_Fr;         %new C_Drag satisfies DF=C_Drag*v^2
C_DF=0.5*C_Dforce*air_dens*A_Plan_wings;     %new C_DF satisfies DF=C_DF*v^2

    % %1 parameter sweep
    % 
    % min=1.1;        %change or verify before sim.
    % max=1.3;       %change or verify before sim.
    % n=10;
    % 
    % for i=1:n
    %     air_dens=min+(i-1)*(max-min)/(n-1);     %change variable depending on sim.
    %     C_Drag=0.5*C_Dr*air_dens*A_Fr;      %(aero only) calculate aero changes due to changing cofficient
    %     C_DF=0.5*C_Dforce*air_dens*A_Plan_wings;
    %     Param(i)=air_dens;                      %change variable depending on sim.
    %     Results(i)=FSAE_Simulator(m,cofg,w,track,WD,ETS,Fuel_corr,C_Drag,C_DF)
    % end
    % 
    % plot(Param,Results)
    % title('Points vs Air Density','fontweight','bold','fontsize',14)   %Change Title
    % xlabel('Air Density (kg/m^3)','fontweight','bold','fontsize',12)                          %Change X label
    % ylabel('Points','fontweight','bold','fontsize',12)


%two parameter sweep
 
const1=C_Dforce;       %set sw1 variable
min1=0;       %set sw1 variable min
max1=3;       %set sw1 variable max
n1=5;           %set # iterations

const2=WD;    %set sw2 variable
min2=0.5;      %set sw2 variable min
max2=0.75;      %set sw2 variable max
n2=5;

Results=zeros(n1,n2);
para1=zeros(n1,1);
para2=zeros(n2,1);

for i=1:n1
    C_Dforce=min1+(i-1)*(max1-min1)/(n1-1);    %change variable
    para1(i)=C_Dforce;     %parameter 1
    C_DF=0.5*C_Dforce*air_dens*A_Plan_wings;
    for j=1:n2
        WD=min2+(j-1)*(max2-min2)/(n2-1);  %change variable
        para2(j)=WD;      %parameter 2
        %C_Drag=0.5*C_Dr*air_dens*A_Fr;        
        
        Results(j,i)=FSAE_Simulator(m,cofg,w,track,WD,ETS,Fuel_corr,C_Drag,C_DF)
    end
end
 
surf(para1,para2,Results)
title('The Effect of Downforce and Weight Distribution on Points','fontweight','bold','fontsize',14)
xlabel('Coeff of Downforce','fontweight','bold','fontsize',12)
ylabel('Weight Distribution (Rear)','fontweight','bold','fontsize',12)
zlabel('Points','fontweight','bold','fontsize',12)