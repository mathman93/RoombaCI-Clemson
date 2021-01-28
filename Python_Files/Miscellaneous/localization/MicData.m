function [ t1, t2, t3, x, y] = MicData( speed, location, size)
%MicData generate mic data for each roomba randomly

x = 0;
y = 0;

while (abs(x) < 2*size || abs(y) < 2 * size)

    x = 5 * (2 * (rand - 0.5)) * size;
    y = 5 * (2 * (rand - 0.5)) * size;
    
end


d1 = Distance(x,y,location(1,1),location(1,2));
d2 = Distance(x,y,location(2,1),location(2,2));
d3= Distance(x,y,location(3,1),location(3,2));


% d1 = ((x-location(1,1))^2+(y-location(1,2))^2)^.5;
% d2 = ((x-location(2,1))^2+(y-location(2,2))^2)^.5;
% d3 = ((x-location(3,1))^2+(y-location(3,2))^2)^.5;

t1 = d1/speed;
t2 = d2/speed;
t3 = d3/speed;

end