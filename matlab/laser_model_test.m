clc, clear, close all

map = dlmread('/home/lawrence/CMU/classes/localization/16833_hw1/particle_filter/data/map/wean2.dat',' ');

nmap = zeros(800,800,3);

for row = 1:numel(map(:,1))
    for col = 1:numel(map(1,:))
        if( map(row,col) == -1 )
            nmap(row,col,1) = 0.9;
            nmap(row,col,2) = 0.9;
            nmap(row,col,3) = 0.9;
        elseif( map(row,col) == 0 )
            nmap(row,col,1) = 0.5;
            nmap(row,col,2) = 0.5;
            nmap(row,col,3) = 0.5;
        elseif( map(row,col) == 1 )
            nmap(row,col,2) = 1;
        else
            nmap(row,col,1) = 1-map(row,col);
            nmap(row,col,2) = 1;
            nmap(row,col,3) = 1-map(row,col);
        end
    end
end

scan = [66 66 66 66 66 65 66 66 65 66 66 66 66 66 67 67 67 66 67 66 67 67 67 68 68 68 69 67 530 514 506 508 494 481 470 458 445 420 410 402 393 386 379 371 365 363 363 364 358 353 349 344 339 335 332 328 324 321 304 299 298 294 291 288 287 284 282 281 277 277 276 274 273 271 269 268 267 266 265 265 264 263 263 263 262 261 261 261 261 261 193 190 189 189 192 262 262 264 194 191 190 190 193 269 271 272 274 275 277 279 279 281 283 285 288 289 292 295 298 300 303 306 309 314 318 321 325 329 335 340 360 366 372 378 384 92 92 91 89 88 87 86 85 84 83 82 82 81 81 80 79 78 78 77 76 76 76 75 75 74 74 73 73 72 72 72 71 72 71 71 71 71 71 71 71 71 70 70 70 70];

x = 416;
y = 400;
th = 180;
mask = nmap;
thScan = -90:89;
smap = [];
ds = [];
for i = 1:numel(thScan)
   d = 0;
   px = x + 2.5*cosd(th);
   py = y + 2.5*sind(th);
   while( map(round(py),round(px)) > 0.95 )
      %mask = nmap;
      d = d + 0.01;
      px = px + d*cosd(th + thScan(i));
      py = py + d*sind(th + thScan(i));
      %mask(round(py),round(px),1) = 1;
      %mask(round(py),round(px),2) = 0;
      %mask(round(py),round(px),3) = 0;
      %imshow(mask);
      %pause(0.1);
   end
   ds(i) = sqrt((px-x)^2 + (py-y)^2)*10;
end

figure
polarscatter(thScan.*(pi/180),ds,'r*');
hold on
polarscatter(-thScan.*(pi/180),scan,'b*');
rlim([0 500])
title('Ray Tracer Performance','FontSize',14)
legend('Ray Tracer','Scan Data')
saveas(gcf,'tracer.png')

laserMean = 5000;
laserStd = 500;
lambda = 0.001;
laserMax = 8183;

n = 1000;

%factor = [0.5 0.2 0.1 0.2];
factor = [500 75 0.3 5500];

dz = laserMax/(n-1);
z = 0:dz:laserMax;
p = zeros(n,5);

for i = 1:n
    
    % Hit
    %fun = @(x) normpdf(x,z(i),laserStd);
    if( z(i) <= laserMax && z(i) >= 0 )
        %nm1(i) = 1/integral(fun,0,laserMax);
        p(i,1) = normpdf(z(i),laserMean,laserStd);
    end
    
    % Short
    if( z(i) >= 0 && z(i) <= laserMean )
        %nm2 = 1/(1-exp(-lambda*laserMean));
        p(i,2) = lambda*exp(-lambda*z(i));
    end
    
    % Zmax
    if( z(i) >= 0.99*laserMax )
        p(i,3) = 1;
    end
    
    % Rand
    if( z(i) >= 0 && z(i) <= laserMax )
        p(i,4) = 1/laserMax;
    end
    
    p(i,5) = factor*p(i,1:4)';
end

figure
plot(z,p(:,5),'LineWidth',2)
axis([0 laserMax 0 max(p(:,5))*1.2])
title('LIDAR Sensor Model','FontSize',14)
xlabel('Distance (cm)','FontSize',14)
ylabel('Probability','FontSize',14)
saveas(gcf,'sensor_mod.png')