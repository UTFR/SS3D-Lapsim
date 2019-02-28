function [t,E_total] = Straight(m,cofg,w,track,WD,C_DF,C_Drag,ETS,d,v1,v2)
%compute the time taken for a straight section
%takes in initial and final velocity
%then decides how to split up accelerating and braking

Decel=Deceleration(m,cofg,w,track,WD,C_DF,C_Drag);
Accel=AccelMatrix(m,cofg,w,track,WD,C_DF,C_Drag,ETS);

i=1;
j=1;


while(Accel(i,3) < v1)
    i=i+1;
end

while(Decel(j,3) < v2)
    j=j+1;
end

v_start=i;      %acceleration staring index (starting speed index)
v_finish=j;     %deceleration final index (ending speed index)

while(Accel(i,1)-Accel(v_start,1)+Decel(j,1)-Decel(v_finish,1) < d)
    if Accel(i,3) < Decel(j,3)
        i=i+1;
    else j=j+1;
    end
end

if(abs(Accel(i,3)-Decel(j,3))>1)
    error('Error: %d m straight is too short!', d)
end

t=Accel(i,2)-Accel(v_start,2)+Decel(j,2)-Decel(v_finish,2);

E_braking=Decel(j,4)-Decel(v_finish,4); %calculate energy loss through brakes
E_acceldrag=(Accel(i,5)-Accel(v_start,5)); %energy loss through drag

if E_braking<0
    E_braking=0;
end

E_total=E_braking+E_acceldrag;

end

