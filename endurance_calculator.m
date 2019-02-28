function [t_endurance,E_total] = endurance_calculator(m, cofg, w,track, WD, C_DF, C_Drag, ETS)

Data_Corner=[26 22 0 0; %FSAEM 2012 Endurance
    33 6 0 0;
    54 55 0 0;
    29 8 0 0;
    20 13 0 0;
    14 2 0 0;
    35 20 0 0;
    44 6 0 0;
    24 65 0 0;
    22 9 0 0;
    19 14 0 0;
    31 13 0 0;
    30 11 0 0;
    27 14 0 0;
    46 12 0 0;
    22 10 0 0;
    35 5 0 0;
    31 5 0 0;
    25 10 0 0;
    25 10 0 0;
    27 36 0 0;
    ]; % [radius, length, time, speed;...]

Data_Straight=[15 0 0;
    116 0 0;
    46 0 0;
    22 0 0;
    97 0 0;
    41 0 0;
    28 0 0;
    13 0 0;
    15 0 0;
    9 0 0;
    81 0 0;
    5 0 0;
    3 0 0;
    98 0 0;
    38 0 0;
    94 0 0;
    6 0 0;
    9 0 0;
    7 0 0;
    12 0 0]; % [length, time, energy;...]


t_corner=0; %total time spent in corners
t_straight=0; %total time spent in straights
E_total_cor=0;      %total energy in corners
E_total_str=0;      %total energy in straights

for i=1:21
    [Data_Corner(i,3),Data_Corner(i,4)]=SScorner(m,cofg,w,track,WD,C_DF,Data_Corner(i,1),Data_Corner(i,2));
    t_corner=t_corner+Data_Corner(i,3);
    
    E_total_cor=E_total_cor+(Data_Corner(i,4))^2*C_Drag*Data_Corner(i,2); %calculate energy (W=F*d=C*v^2*L) due to drag at SS corner speed
end

for j=1:20
    [Data_Straight(j,2), Data_Straight(j,3)]=Straight(m,cofg,w,track,WD,C_DF,C_Drag,ETS,Data_Straight(j,1),Data_Corner(j,4),Data_Corner(j+1,4));
    
    t_straight=t_straight+Data_Straight(j,2);
    E_total_str=E_total_str+Data_Straight(j,3);
end

t_endurance=t_straight+t_corner;
E_total=E_total_cor+E_total_str;


%Data_Corner %for debugging purposes
%Data_Straight %for debugging purposes

end

