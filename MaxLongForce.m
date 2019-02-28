function [ LongF ] = MaxLongForce(NF)
%Calculate Max Lateral Force
%LongF=(1.6-NF/10000)*NF;
LongF=(-0.00031*NF^2-0.0472*NF+1853.3)/1000*0.8*NF;
end


