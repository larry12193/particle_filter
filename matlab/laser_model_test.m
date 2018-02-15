clc, clear, close all

laserMean = 4000/100;
laserStd = 5;
lambda = 0.1;
laserMax = 8183/100;

n = 1000;

factor = [0.5 0.2 0.1 0.2];

dz = laserMax/(n-1);
z = 0:dz:laserMax;
p = zeros(n,5);

for i = 1:n
    
    fun = @(x) normpdf(x,z(i),laserStd);
    if( z(i) <= laserMax && z(i) >= 0 )
        nm1(i) = 1/integral(fun,0,laserMax);
        p(i,1) = nm1(i)*normpdf(z(i),laserMean,laserStd);
    end
    
    if( z(i) >= 0 && z(i) <= laserMean )
        nm2 = 1/(1-exp(-lambda*laserMean));
        p(i,2) = nm2*lambda*exp(-lambda*z(i));
    end
    
    if( z(i) == laserMax )
        p(i,3) = 1;
    end
    
    if( z(i) > 0 && z(i) < laserMax )
        p(i,4) = 1;
    end
    
    p(i,5) = factor*p(i,1:4)';
end

figure
plot(z,p(:,5))
axis([0 laserMax 0 max(p(:,5))*1.2])