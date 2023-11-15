clear all; close all;

NS = 64;
amplitude = 1500;
dev = 0.03;
offset = 1910;

x = linspace(0, 1, NS);
f = uint32(amplitude*exp(-(x-0.5).^2./(dev)) + offset);
plot(x, f);
fprintf('%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, \n', f);