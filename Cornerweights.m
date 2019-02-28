function [N_IF,N_OR,N_OF,N_IR] = Cornerweights(m,aLat,aLong,track,w,cofg,WD,DF)
%calculate Inside and Outside Normal Force
LatT=m*aLat*cofg/track;
LongT=m*aLong*cofg/w;
N_IF=m*(1-WD)/2*9.81-LatT/2-LongT/2+DF/4;
N_OR=m*WD/2*9.81+LatT/2+LongT/2+DF/4;
N_OF=m*(1-WD)/2*9.81+LatT/2-LongT/2+DF/4;
N_IR=m*WD/2*9.81-LatT/2+LongT/2+DF/4;

end

