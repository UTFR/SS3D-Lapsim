function [ wheel_torque, gear ] = GearLookup( vel, ETS, gear )
% TorqueCurve = [ 
%     3000 20.8; %20.6;
%     3250 22.5; %20.5;
%     3500 24.2; %18.8;
%     3750 28.0; %20.7;
%     4000 31.8; %22.9;
%     4250 33.0; %24.4;
%     4500 34.3; %25.2;
%     4750 33.3; %26.9;
%     5000 32.4; %32.3;
%     5250 32.9; %35.5;
%     5500 33.3; %36.1;
%     5750 32.9; %35.1;
%     6000 32.4; %34.3;
%     6250 34.2; %34.2;
%     6500 36.0; %34.7;
%     6750 37.2; %35.8;
%     7000 38.5; %35.8;
%     7250 38.2; %36.1;
%     7500 37.9; %35.3;
%     7750 37.4; %34.9;
%     8000 36.8; %33.9;
%     8250 36.4; %32.8;
%     8500 36.0; %31.6;
%     8750 35.2; %30.4;
%     9000 34.5; %28.9;
%     9250 33.7; %27.4;
%     9500 32.8; %25.8;
%     9750 31.9; %24.9;
%     10000 31.0; %24;
%     10250 29.8; %23.8;
%     ];

TorqueCurve = [ 
    3000 20.6;
    3250 20.5;
    3500 18.8;
    3750 20.7;
    4000 22.9;
    4250 24.4;
    4500 25.2;
    4750 26.9;
    5000 32.3;
    5250 35.5;
    5500 36.1;
    5750 35.1;
    6000 34.3;
    6250 34.2;
    6500 34.7;
    6750 35.8;
    7000 35.8;
    7250 36.1;
    7500 35.3;
    7750 34.9;
    8000 33.9;
    8250 32.8;
    8500 31.6;
    8750 30.4;
    9000 28.9;
    9250 27.4;
    9500 25.8;
    9750 24.9;
    10000 24;
    10250 23.8;
    ];

[num_points,~] = size(TorqueCurve);
index = 1;
while (index<num_points+1)
    TorqueCurve(index,2)=TorqueCurve(index,2)*ETS;
    index = index +1;
end

First=2.739*29/14;
Second=2.739*26/16;
Third=2.739*24/18;
Fourth=2.739*28/25;
Fifth=2.739*26/27;
Final=3.25; %final drive ratio

%Find Shift fron 1 to 2
low_v = 8000/First/Final/60*(0.226*2*3.14159);
high_v= TorqueCurve(num_points,1)/Second/Final/60*(0.226*2*3.14159);
v=low_v;
exit = 0;
while(v<=high_v&&exit<1)
   First_RPM = v*First*Final*60/(0.226*2*3.14159);
   Second_RPM = v*Second*Final*60/(0.226*2*3.14159);
   index = 1;
   while(index<=num_points && First_RPM>TorqueCurve(index,1))
       index = index+1;       
   end
   First_Torque = ((TorqueCurve(index,2)-TorqueCurve(index-1,2))/(TorqueCurve(index,1)-TorqueCurve(index-1,1))*(First_RPM-TorqueCurve(index-1,1))+TorqueCurve(index-1,2))*First;
   if(Second_RPM > 5000)
        index = 1;
        while(index<=num_points && Second_RPM>TorqueCurve(index,1))
            index = index+1;       
        end
        Second_Torque = ((TorqueCurve(index,2)-TorqueCurve(index-1,2))/(TorqueCurve(index,1)-TorqueCurve(index-1,1))*(Second_RPM-TorqueCurve(index-1,1))+TorqueCurve(index-1,2))*Second;
        if (Second_Torque>First_Torque)
            First_Shift = v;
            exit = 1;
        end
   end
   v = v+0.1;
end

%find shift from 2 to 3
low_v = 8000/Second/Final/60*(0.226*2*3.14159);
high_v= TorqueCurve(num_points,1)/Third/Final/60*(0.226*2*3.14159);
v=low_v;
exit = 0;
while(v<=high_v&&exit<1)
   Third_RPM = v*Third*Final*60/(0.226*2*3.14159);
   Second_RPM = v*Second*Final*60/(0.226*2*3.14159);
   index = 1;
   while(index<=num_points && Second_RPM>TorqueCurve(index,1))
       index = index+1;       
   end
   Second_Torque = ((TorqueCurve(index,2)-TorqueCurve(index-1,2))/(TorqueCurve(index,1)-TorqueCurve(index-1,1))*(Second_RPM-TorqueCurve(index-1,1))+TorqueCurve(index-1,2))*Second;
   if(Third_RPM > 5000)
        index = 1;
        while(index<=num_points && Third_RPM>TorqueCurve(index,1))
            index = index+1;       
        end
        Third_Torque = ((TorqueCurve(index,2)-TorqueCurve(index-1,2))/(TorqueCurve(index,1)-TorqueCurve(index-1,1))*(Third_RPM-TorqueCurve(index-1,1))+TorqueCurve(index-1,2))*Third;
        if (Third_Torque>Second_Torque)
            Second_Shift = v;
            exit = 1;
        end
   end
   v = v+0.1;
end

%find shift from 3 to 4
low_v = 8000/Third/Final/60*(0.226*2*3.14159);
high_v= TorqueCurve(num_points,1)/Fourth/Final/60*(0.226*2*3.14159);
v=low_v;
exit = 0;
while(v<=high_v&&exit<1)
   Third_RPM = v*Third*Final*60/(0.226*2*3.14159);
   Fourth_RPM = v*Fourth*Final*60/(0.226*2*3.14159);
   index = 1;
   while(index<=num_points && Third_RPM>TorqueCurve(index,1))
       index = index+1;       
   end
   Third_Torque = ((TorqueCurve(index,2)-TorqueCurve(index-1,2))/(TorqueCurve(index,1)-TorqueCurve(index-1,1))*(Third_RPM-TorqueCurve(index-1,1))+TorqueCurve(index-1,2))*Third;
   if(Fourth_RPM > 5000)
        index = 1;
        while(index<=num_points && Fourth_RPM>TorqueCurve(index,1))
            index = index+1;       
        end
        Fourth_Torque = ((TorqueCurve(index,2)-TorqueCurve(index-1,2))/(TorqueCurve(index,1)-TorqueCurve(index-1,1))*(Fourth_RPM-TorqueCurve(index-1,1))+TorqueCurve(index-1,2))*Fourth;
        if (Fourth_Torque>Third_Torque)
            Third_Shift = v;
            exit = 1;
        end
   end
   v = v+0.1;
end

%find shift from 4 to 5
low_v = 8000/Fourth/Final/60*(0.226*2*3.14159);
high_v= TorqueCurve(num_points,1)/Fifth/Final/60*(0.226*2*3.14159);
v=low_v;
exit = 0;
while(v<=high_v&&exit<1)
   Fifth_RPM = v*Fifth*Final*60/(0.226*2*3.14159);
   Fourth_RPM = v*Fourth*Final*60/(0.226*2*3.14159);
   index = 1;
   while(index<=num_points && Fourth_RPM>TorqueCurve(index,1))
       index = index+1;       
   end
   Fourth_Torque = ((TorqueCurve(index,2)-TorqueCurve(index-1,2))/(TorqueCurve(index,1)-TorqueCurve(index-1,1))*(Fourth_RPM-TorqueCurve(index-1,1))+TorqueCurve(index-1,2))*Fourth;
   if(Fifth_RPM > 5000)
        index = 1;
        while(index<=num_points && Fifth_RPM>TorqueCurve(index,1))
            index = index+1;       
        end
        Fifth_Torque = ((TorqueCurve(index,2)-TorqueCurve(index-1,2))/(TorqueCurve(index,1)-TorqueCurve(index-1,1))*(Fifth_RPM-TorqueCurve(index-1,1))+TorqueCurve(index-1,2))*Fifth;
        if (Fifth_Torque>Fourth_Torque)
            Fourth_Shift = v;
            exit = 1;
        end
   end
   v = v+0.1;
end

if(vel<First_Shift)
    gear = 1;
    RPM = vel*First*Final*60/(0.226*2*3.14159);
    index = 1;
    while(index<=num_points && RPM>TorqueCurve(index,1))
        index = index+1;       
    end
    if (RPM<TorqueCurve(1,1))
        wheel_torque = TorqueCurve(1,2)*First*Final;
    else
        wheel_torque = ((TorqueCurve(index,2)-TorqueCurve(index-1,2))/(TorqueCurve(index,1)-TorqueCurve(index-1,1))*(RPM-TorqueCurve(index-1,1))+TorqueCurve(index-1,2))*First*Final;
    end
else
    if(vel>First_Shift && vel<Second_Shift)
        gear = 2;
        RPM = vel*Second*Final*60/(0.226*2*3.14159);
        index = 1;
        while(index<=num_points && RPM>TorqueCurve(index,1))
            index = index+1;       
        end
        wheel_torque = ((TorqueCurve(index,2)-TorqueCurve(index-1,2))/(TorqueCurve(index,1)-TorqueCurve(index-1,1))*(RPM-TorqueCurve(index-1,1))+TorqueCurve(index-1,2))*Second*Final;
    else
        if(vel>Second_Shift && vel<Third_Shift)
            gear = 3;
            RPM = vel*Third*Final*60/(0.226*2*3.14159);
            index = 1;
            while(index<=num_points && RPM>TorqueCurve(index,1))
                index = index+1;       
            end
            wheel_torque = ((TorqueCurve(index,2)-TorqueCurve(index-1,2))/(TorqueCurve(index,1)-TorqueCurve(index-1,1))*(RPM-TorqueCurve(index-1,1))+TorqueCurve(index-1,2))*Third*Final;
        else
            if(vel>Third_Shift && vel<Fourth_Shift)
                gear = 4;
                RPM = vel*Fourth*Final*60/(0.226*2*3.14159);
                index = 1;
                while(index<=num_points && RPM>TorqueCurve(index,1))
                    index = index+1;       
                end
                wheel_torque = ((TorqueCurve(index,2)-TorqueCurve(index-1,2))/(TorqueCurve(index,1)-TorqueCurve(index-1,1))*(RPM-TorqueCurve(index-1,1))+TorqueCurve(index-1,2))*Fourth*Final;
            else if(vel<TorqueCurve(num_points,1)/Fifth/Final/60*(0.226*2*3.14159))
                    gear=5;
                    RPM = vel*Fifth*Final*60/(0.226*2*3.14159);
                    index = 1;
                    while(index<=num_points && RPM>TorqueCurve(index,1))
                        index = index+1;       
                    end
                    if (index < num_points)
                        wheel_torque = ((TorqueCurve(index,2)-TorqueCurve(index-1,2))/(TorqueCurve(index,1)-TorqueCurve(index-1,1))*(RPM-TorqueCurve(index-1,1))+TorqueCurve(index-1,2))*Fifth*Final;
                    else
                        wheel_torque = TorqueCurve(num_points,2)*Fifth*Final;
                    end
                else 
                    gear=5;
                    wheel_torque=0;
                end                    
            end
        end
    end
end


end

