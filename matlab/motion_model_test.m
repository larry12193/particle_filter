clc, clear, close all

n = 1000;

x0 = [0,0,0];
x(1,:) = [-84.751747 -178.306763 -1.292131];
x(2,:) = [-83.688889 -182.257324 -1.280818];
x1 = x0 + (x(2,:)-x(1,:));
x1(1:2) = x1(1:2)/10;
p = zeros(n,3);

a1 = 0.01;
a2 = 0.05;
a3 = 0.005;
a4 = 0.005;

dx = (x(2,1) - x(1,1))/10;
dy = (x(2,2) - x(1,2))/10;
dth = x(2,3) - x(1,3);

r1 = atan2(dy,dx) - x(1,3);
tr = sqrt(dx*dx + dy*dy);
r2 = dth - r1;

for i = 1:n
    r1h = r1 - normrnd(0,sqrt(a1*r1^2 + a2*tr^2));
    trh = tr - normrnd(0,sqrt(a3*tr^2 + a4*r1^2 + a4*r2^2));
    r2h = r2 - normrnd(0,sqrt(a1*r1^2 + a2*tr^2));
    
    p(i,1) = x0(1,1) + trh*cos(x0(1,3) + r1h);
    p(i,2) = x0(1,2) + trh*sin(x0(1,3) + r1h);
    p(i,3) = x0(1,3) + r1h + r2h;
end

figure
scatter(x0(1,1),x0(1,2),'b*','LineWidth',5)
hold on
scatter(p(:,1),p(:,2),'k.')
scatter(mean(p(:,1)),mean(p(:,2)),'r*','LineWidth',5)
line([0 mean(p(:,1))],[0 mean(p(:,2))],'LineStyle','--','Color','k')
axis equal
title('Odometry Motion Model','FontSize',14)
xlabel('X (cm)','FontSize',14)
ylabel('Y (cm)','FontSize',14)
text(0.1,0.17,['a1 = ',num2str(a1)],'FontSize',12)
text(0.1,0.14,['a2 = ',num2str(a2)],'FontSize',12)
text(0.1,0.11,['a3 = ',num2str(a3)],'FontSize',12)
text(0.1,0.08,['a4 = ',num2str(a4)],'FontSize',12)
saveas('odom_model.png')
function s = sample(d)
    s = 0;
    for i = 1:12
        s = s + 2*d*rand - d;
    end
    s = s/12;
end