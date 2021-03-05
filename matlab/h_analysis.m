clear all
close all
clc

Q2 = -pi/4:0.001:pi/4;
FK = zeros(3,length(Q2));
for i = 1:length(Q2)
    q2 = Q2(i);        
    FK(:,i) = phantom_fk(0,q2,0);
end

figure(1)  
plot(Q2,FK(3,:));
hold on
plot(Q2,FK(1,:));
       
figure(2)
plot(FK(3,:), FK(1,:))
% hold on
% plot(FK(3,:), Q2);

[M,I] = max(FK(1,:));
FK(3,I)

dI = 120;
FK(3,I + dI) - FK(3,I - dI)

hold on 
plot([FK(3,I + dI),FK(3,I - dI)],[FK(1,I + dI),FK(1,I - dI)],'o')
