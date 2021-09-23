function [x,y] = getEllipse(rx,ry,C,theta,n)
if nargin == 4
    n = 1000;
end
beta = linspace(0,2*pi,n);
x = rx*cos(beta);
y = ry*sin(beta);
X = [cos(theta) -sin(theta);sin(theta) cos(theta)]*([x;y]) + C;
x = X(1,:);
y = X(2,:);
end