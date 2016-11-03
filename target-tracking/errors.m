
load est_targetDelay

X= est_targetDelay;

figure;
error_n1 = abs(X(2,:) - X(92,:));
error_e1 = abs(X(3,:) - X(93,:));
me1= mean(error_e1);
mn1= mean(error_n1);
subplot(3,1,1)
plot(error_n1)
hold on 
plot(error_e1,'r')
legend('error_n1','error_e1')
title(['mean n1= ',num2str(mn1),  'mean e1= ',num2str(me1)])


error_n2 = abs(X(32,:) - X(98,:));
error_e2 = abs(X(33,:) - X(99,:));
me2= mean(error_e2);
mn2= mean(error_n2);
subplot(3,1,2)
plot(error_n2)
hold on 
plot(error_e2,'r')
legend('error_n2','error_e2')
title(['mean n2= ',num2str(mn2),  'mean e2= ',num2str(me2)])

error_n3 = abs(X(62,:) - X(104,:));
error_e3 = abs(X(63,:) - X(105,:));
me3= mean(error_e3);
mn3= mean(error_n3);
subplot(3,1,3)
plot(error_n3)
hold on 
plot(error_e3,'r')
legend('error_n3','error_e3')
title(['mean n3= ',num2str(mn3),  'mean e3= ',num2str(me3)])

meanN=mean([mn1 mn2 mn3])
meanE=mean([me1 me2 me3])
%%%%%%
% figure(5)
% load UAVGimbal
% UAVGimbal = UAVGimbal;
% 
% R_min=100;
% h_d= 100;
% hold on
%  plot(UAVGimbal(2,:),UAVGimbal(3,:),'r')