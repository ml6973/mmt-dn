function out=SelectedTarget1(in,P,nt)
NN=6*nt;
TargetIndex=in(NN+1);
out=in(6*(TargetIndex-1)+1:6*(TargetIndex-1)+6);