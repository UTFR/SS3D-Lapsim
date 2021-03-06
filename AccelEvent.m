function [ tot_time ] = AccelEvent( m, cofg, w,track, WD, C_DF, C_Drag, ETS)
tot_time = 0;
length = 75; 
position = 0;
shift_time = 0.1; %seconds
increment = 0.1; %metres
v1 = 0.1;
[~ , gear] = GearLookup(v1, ETS);
aLat =0;
while (position <length) %iterates through distance, calculates time for each increment
    [wheel_torq,new_gear] = GearLookup(v1, ETS);
    wheel_force = wheel_torq/(9*2.54/100);
    drag_force = C_Drag*v1^2;
    DF=C_DF*(0.1)*v1^2; 
    aLong = 0;
    aLong_new = (wheel_force - drag_force)/m;%check weight transfer here

    while(abs(aLong-aLong_new)>0.1) %compares weight transfer to check wheel slip
    
        aLong=aLong_new; 
        [N_IF,N_OR,N_OF,N_IR]=Cornerweights(m,aLat,aLong,track,w,cofg,WD,DF);

        if(N_IF<0 || N_OR<0 || N_OF<0 || N_IR<0) 
            error('Error: negative weight on tire')
        end

        LongF_OR=MaxLongForce(N_OR);
        LongF_IR=MaxLongForce(N_IR);

        if (wheel_force > (LongF_OR+LongF_IR))
            wheel_force = (LongF_OR+LongF_IR);
            aLong_new = (wheel_force - drag_force)/m;
        end
    
    
    end
    
    
    
    v1_new = sqrt(v1^2+2*aLong*increment);
    position = position + increment;
    tot_time = tot_time + increment/((v1+v1_new)/2);
    if(new_gear>gear)
        position = position + shift_time*((v1+v1_new)/2);
        tot_time = tot_time + shift_time;
    end
    v1 = v1_new;
    gear=new_gear;
end




end

