 function [Decel] = Deceleration(m,cofg,w,track,WD,C_DF,C_Drag)
% Compute the speed over time given a distance, final speedsgiven starting

v=0.1;
n_increments=400; 
d_increment=0.2;

Drag_energy=0;
Decel=zeros(n_increments,4);

d=0;
t=0;
aLat=0;
aLong=5;
a_tires=14;

for i=2:n_increments
    
    a_drag=C_Drag*v^2/m; 
    F_DF=C_DF*v^2; 
    
    aLong_new=0;
       
    while (abs(aLong-aLong_new)>0.02)
        
        aLong=a_tires+a_drag;

        [N_IF,N_OR,N_OF,N_IR]=Cornerweights(m,aLat,a_tires,track,w,cofg,WD,F_DF); %calculate corner weights due to WT

         if(N_IF<0 || N_OR<0 || N_OF<0 || N_IR<0) 
                %wheels coming off the ground
                %need to write something here
         end

            LongF_IF=MaxLongForce(N_IF);
            LongF_OR=MaxLongForce(N_OR);
            LongF_OF=MaxLongForce(N_OF);
            LongF_IR=MaxLongForce(N_IR);

            TotalLongF=LongF_IF+LongF_OR+LongF_OF+LongF_IR;
            a_tires=TotalLongF/m;
            aLong_new=a_tires+a_drag;
    end
    
    v_old=v;
    v=(v^2+2*aLong*d_increment)^0.5;
    d=d+d_increment;
    t=t+2*d_increment/(v+v_old);
    
    Decel(i,1)=d;  %may need to shift d and t down and add first column as 0
    Decel(i,2)=t;   %assign time
    Decel(i,3)=v_old; %assign velocity
    Drag_energy = Drag_energy + (v^2+v_old^2)/2*C_Drag*(Decel(i,1)-Decel(i-1,1));
    Decel(i,4) = Drag_energy;
end

end

% v=40; %max simulation speed for matrix
% n_increments=40; 
% d_increment=1; % user defined distance increment for calculation purposes
% 
% Decel=zeros(n_increments,3);
% 
% d=0;
% t=0;
% aLat=0;
% aLong=5;
% a_tires=14; %guess max acceleration to begin iteration
% 
% 
% for i=1:n_increments
%     
%     a_drag=0.5*C_Drag*0.1*v^2/m; %need to update formula
%     F_DF=0.5*C_DF*v^2; %need to update formula
%     
%     aLong_new=0;
%        
%     while (abs(aLong-aLong_new)>0.05)
%         
%         aLong=a_tires+a_drag;
% 
%         [N_IF,N_OR,N_OF,N_IR]=Cornerweights(m,aLat,a_tires,track,w,cofg,WD,F_DF); %calculate corner weights due to WT
% 
%          if(N_IF<0 || N_OR<0 || N_OF<0 || N_IR<0) 
%                 %wheels coming off the ground
%                 %need to write something here
%          end
% 
%             LongF_IF=MaxLongForce(N_IF);
%             LongF_OR=MaxLongForce(N_OR);
%             LongF_OF=MaxLongForce(N_OF);
%             LongF_IR=MaxLongForce(N_IR)
% 
%             TotalLongF=LongF_IF+LongF_OR+LongF_OF+LongF_IR;
%             a_tires=TotalLongF/m;
%             aLong_new=a_tires+a_drag
%     end
%     
%     v_old=v;
%     v=(v^2-2*aLong*d_increment)^0.5;
%     d=d+d_increment;
%     t=t+2*d_increment/(v+v_old);
%     
%     Decel(i,1)=d;  %may need to shift d and t down and add first column as 0
%     Decel(i,2)=t;
%     Decel(i,3)=v_old;
%     
% end
% 
% end


