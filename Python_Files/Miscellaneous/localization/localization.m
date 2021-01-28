clc
close all
clearvars
%% Variable List
%{

micDist - distance from the center of the Roomba to the microphones

speedOfSound - speed of sound

xLoc, yLoc - xy coords of source

t1, t2, t3 - time that sound reached each speaker

delta - differences in distances from mics [delta12, delta23, delta31]

micLoc - matrix containing the xy coords of the mics [x1 y1;x2 y2; x3 y3]

x,y,r,t,i - used to graph

A - 2 x 2 matrix containing differences in x and y values for mic locations

B - 2 x 1 matrix containing time differences times speed of sound

difAngle - difference in calculated and real angles

result - result vector pointing to source

resutl - result normalized
%}

%% User Input

micDist = 5;

%% Generate Microphone Coordinates
micLoc(1,1) = -17/2; % x
micLoc(1,2) = 16; % y

micLoc(2,1) = 17/2;
micLoc(2,2) = 16;

micLoc(3,1) = 0;
micLoc(3,2) = 0;

micLoc = micLoc .* 0.0254; % inches to meters

%% MicData

speedOfSound = 340; % m/s
[ t1, t2, t3, xLoc, yLoc] = MicData(speedOfSound, micLoc, micDist);
% 180
t1 = 0;
t2 = 0.00153265;
t3 =   0.0011325;
% 90
t1 = 7.884e-05;
t2 = 0;
t3 = 0.00105996;
% out from 3
t1 = 0;
t2 = 0.00140732;
t3 = 0.00140732;

xLoc = 0;
yLoc = 0;

%% Plot

figure('color','white')
hold on
grid on
axis([-10 * micDist, 10 * micDist, -10 * micDist, 10 * micDist])

% plot circle for Roomba
r = 1.5 * micDist;
t = linspace(0,2*pi);
x = r*cos(t);
y = r*sin(t);
plot(x,y,'Color','k','LineWidth',2)

% plot microphones
for i = 1:3
    plot(micLoc(i,1),micLoc(i,2),'MarkerFaceColor','k','Marker','o','MarkerEdgeColor','k')
end

% plot lines between microphones
x = [micLoc(1,1), micLoc(2,1)];
y = [micLoc(1,2), micLoc(2,2)];
plot(x,y,'Color','[.85, .85, .85]')

x = [micLoc(2,1), micLoc(3,1)];
y = [micLoc(2,2), micLoc(3,2)];
plot(x,y,'Color','[.85, .85, .85]')

x = [micLoc(3,1), micLoc(1,1)];
y = [micLoc(3,2), micLoc(1,2)];
plot(x,y,'Color','[.85, .85, .85]')

%% Calculation

A = [micLoc(2,1)-micLoc(1,1), micLoc(2,2)-micLoc(1,2);micLoc(3,1)-micLoc(1,1), micLoc(3,2)-micLoc(1,2)];
B = [t1-t2;t1-t3];
result = A^(-1)*B;
% make into unit vector
resutl = result./norm(result);

%% print
LineAtHeading2(atan2(result(2),result(1)),20);
plot(resutl(1),resutl(2),'k*')
plot(xLoc,yLoc,'r*')
difAngle = atan2(yLoc,xLoc) - atan2(micLoc(1,2)+result(2),result(1));