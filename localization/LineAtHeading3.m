function [  ] = LineAtHeading3( heading, length )

x = [0, 10 *length * cos(heading)];
y = [0, 10 * length * sin(heading)];
y = y ;
plot(x,y,'Color','r','LineStyle','--')

end

