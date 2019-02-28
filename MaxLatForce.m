function [ LatF ] = MaxLatForce(NF)
%Calculate Max Lateral Force
%not valid for very high tire forces (>1500N)
LatF=(-0.00031*NF^2-0.0472*NF+1853.3)/1000*0.8*NF;
end

