clc, clear, close all

cd('../data/map');

map = dlmread('wean2.dat',' ');

openMap = [];
count = 1;
for row = 1:numel(map(:,1))
    for col = 1:numel(map(1,:))
        if( map(row,col) == 1 )
            openMap(count,:) = [row-1,col-1];
            count = count + 1;
        end
    end
end

dlmwrite('open_map.dat',openMap,'delimiter',' ','newline','unix');