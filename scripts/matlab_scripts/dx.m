% Robot dynamics: single attractor 
function dx = dx(t,x,x_ref,dx_ref)
    dx = -10*eye(2)*(x-x_ref) + dx_ref;
    %dx = -(x-x_ref);
end