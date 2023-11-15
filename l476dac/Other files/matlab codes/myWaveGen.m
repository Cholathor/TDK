close all; clear all;

NS = 64;
RES = 12;
OFFSET = 0;
nPoints = 5;

T = 0:(NS-1);

f = figure;
hold on;
xlim([0, NS]);
ylim([0, 4096]);

result = zeros([NS, 1]);

xcoords = zeros([1, nPoints]);
ycoords = zeros([1, nPoints]);
xcoords(1) = 1;
ycoords(1) = 2048;

xcoords(end) = NS;
ycoords(end) = 2048;

plot(1, 2048, 'ro');
plot(NS, 2048, 'ro');

for i = 2:(nPoints-1)
    [x, y] = ginput(1);
    xcoords(i) = round(x);
    ycoords(i) = round(y);
    plot(xcoords(i), ycoords(i), 'ro');
end

for i = 1:nPoints
    
    if i == nPoints
        break;
    end
    x = xcoords(i);
    y = ycoords(i);

    nextX = xcoords(i+1);
    nextY = ycoords(i+1);
    result(x:nextX) = round(linspace(y,nextY, size(x:nextX, 2)));
end

plot(result)


fprintf('%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, \n', result);


