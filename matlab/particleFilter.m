clc, clear, close all

map = dlmread('/home/lawrence/CMU/classes/localization/16833_hw1/particle_filter/data/map/wean2.dat',' ');
free = dlmread('/home/lawrence/CMU/classes/localization/16833_hw1/particle_filter/data/map/open_map.dat',' ');

image = map;

nParticle = 100;

laserMean = 4000/100;
laserStd = 5;
lambda = 0.1;
laserMax = 8183/100;
factor = [0.5 0.2 0.1 0.2];

a1 = 0.1;
a2 = 0.05;
a3 = 0.01;
a4 = 0.01;

X    = zeros(nPart,4);
Xbar = X;

for i = 1:nParticle
    samp = rand*numel(free(:,1));
    X(i,1) = free(round(samp),1)/10;
    X(i,2) = free(round(samp),2)/10;
    X(i,3) = pi*(rand*360 - 180)/180;
    
end