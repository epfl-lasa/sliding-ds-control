% function used to check if a point is inside a given ellipse
function check = checkInsideEllipse(rx,ry,C,theta,ref)
A = [cos(-theta) -sin(-theta);sin(-theta) cos(-theta)];
ref = A*(ref-C) + C;
if (((ref(1)-C(1))^2/rx^2) + ((ref(2)-C(2))^2/ry^2)) <= 1
    check = 1;
else
    check = 0;
end
end