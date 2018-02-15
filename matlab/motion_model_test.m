clc, clear, close all

n = 1000;

x0 = [0,0,1];
x(1,:) = x0;
x(2,:) = [0,0,1];

p = zeros(n,3);

a1 = 0.1;
a2 = 0.05;
a3 = 0.01;
a4 = 0.01;

for i = 1:n
    drot1 = atan2(x(2,2)-x(1,2),x(2,1)-x(1,1)) - x(1,3);
    dtrs = sqrt((x(2,1)-x(1,1))^2 + (x(2,2)-x(1,2))^2);
    drot2 = x(2,3) - x(1,3) - drot1;
    
    drothat1 = drot1 - normrnd(0,a1*drot1^2+a2*drot2^2);
    dtrshat  = dtrs - normrnd(0,a3*dtrs^2 + a4*drot1^2 + a4*drot2^2);
    drothat2 = drot2 - normrnd(0,a1*drot1^2+a2*drot2^2);
    
    p(i,1) = x(1,1) + dtrshat*cos(x(1,3) + drothat1);
    p(i,2) = x(1,2) + dtrshat*sin(x(1,3) + drothat1);
    p(i,3) = x(1,3) + drothat1 + drothat2;
end

figure
scatter(x(1,1),x(1,2),'b*','LineWidth',3)
hold on
scatter(x(2,1),x(2,2),'r*','LineWidth',3)
plot(x(:,1),x(:,2),'k--')
scatter(p(:,1),p(:,2),'k.')
axis equal