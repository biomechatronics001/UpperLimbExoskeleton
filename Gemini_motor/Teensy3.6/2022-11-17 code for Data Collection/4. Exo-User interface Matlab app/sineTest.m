clear all
close all
clc

%%
phi = 0;
duration = 99;
magnitude = 1;

timeList = 0:99;
torqueTempList = zeros(size(timeList));
torqueTempList(1+phi:1+phi+duration) = magnitude * sin(pi/duration*(timeList(1:1+duration)));
offset = max([0,phi+duration-99]);
torqueList = circshift(torqueTempList(1+offset:1+offset+99),offset);

xList = [0:100];
yList = [torqueList(end),torqueList];
figure(1)
plot(xList, yList)

%%
phi = 0;
duration = 99.9;
durationInt = round(duration*10);
magnitude = 1;

timeList = 0:999;
torqueTempList = zeros(size(timeList));
torqueTempList(1+phi:1+phi+durationInt) = magnitude * sin(pi/durationInt*(timeList(1:1+durationInt)));
offset = max([0,phi+durationInt-999]);
torqueList = circshift(torqueTempList(1+offset:1+offset+999),offset);

xList = [0:1000]/10;
yList = [torqueList(end),torqueList];
figure(1)
plot(xList, yList)