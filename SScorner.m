function [t,v] = SScorner(m,cofg,w,track,WD,C_DF,r_corner,distance)
%Calculate SS cornering t,v given corner radius

aLat=0;
aLat_new=4;
aLong=0;

while(abs(aLat-aLat_new)>0.02)
    
    aLat=aLat_new;
    v=(aLat*r_corner)^0.5;
    DF=C_DF*v^2;    
    
    [N_IF,N_OR,N_OF,N_IR]=Cornerweights(m,aLat,aLong,track,w,cofg,WD,DF);

    if(N_IF<0 || N_OR<0 || N_OF<0 || N_IR<0) 
        %wheels coming off the ground
        %need to write something here
    end

    LatF_IF=MaxLatForce(N_IF);
    LatF_OR=MaxLatForce(N_OR);
    LatF_OF=MaxLatForce(N_OF);
    LatF_IR=MaxLatForce(N_IR);

    TotalLatF=LatF_IF+LatF_OR+LatF_OF+LatF_IR;
    aLat_new=TotalLatF/m;

end

t=(r_corner)^(-0.5)/aLat_new^0.5*distance;
v=distance/t;

end

