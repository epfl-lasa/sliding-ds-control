function q = unicycle(x,u)
    A = [ cos(x(3,:)) 0;
        sin(x(3,:)) 0;
        0 1];
    q = A*u;
end