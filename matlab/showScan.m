% clc, clear, close all

log = '/home/lawrence/CMU/classes/localization/16833_hw1/particle_filter/data/log/robotdata1.log';
%import(
%data = {};
%data = dlmread(log,' ');
% robotdata1 = uiimport(log);
figure
th = -pi/2:pi/180:pi/2-(pi/180);
for i = 1:numel(robotdata1(:,1))
    if( strcmp(robotdata1{i,1},'L') )
      polarplot(th,cell2mat(robotdata1(i,8:187)),'r.')
      rlim([0 3000])
      pause(0.1)
    end
end