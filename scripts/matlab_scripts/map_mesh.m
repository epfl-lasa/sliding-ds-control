function [a,X,Y] = map_mesh(obs,robot,reference,goal, robot_radius)
    
    n_obs = length(obs);
    
    x = [];
    y = [];
    
    for j = 1:n_obs
        x_left(j) = obs{j}.c(1) - obs{j}.rx - obs{j}.eps - robot_radius;
        x_right(j) = obs{j}.c(1) + obs{j}.rx + obs{j}.eps + robot_radius;
        x = [x linspace(x_left(j)-0.05,x_right(j)+0.05,15)];
        
        y_left(j) = obs{j}.c(2) - obs{j}.ry - obs{j}.eps - robot_radius;
        y_right(j) = obs{j}.c(2) + obs{j}.ry + obs{j}.eps + robot_radius;
        y = [y linspace(y_left(j)-0.05,y_right(j)+0.05,15)];
    end
    
    x_left(n_obs+1) = min([robot(1),reference(1),goal(1)]) - robot_radius;
    x_right(n_obs+1) = max([robot(1),reference(1),goal(1)]) + robot_radius;
    y_left(n_obs+1) = min([robot(2),reference(2),goal(2)]) - robot_radius;
    y_right(n_obs+1) = max([robot(2),reference(2),goal(2)]) + robot_radius;
    
    a(1) = min(x_left) - 0.5;
    a(2) = max(x_right) + 0.5;
    a(3) = min(y_left) - 0.2;
    a(4) = max(y_right) + 0.2;
    
%     x = [x linspace(a(1),a(2),100)];
%     y = [y linspace(a(3),a(4),100)];
%     x = uniquetol(sort(x),1e-3);
%     y = uniquetol(sort(y),1e-3);
    x = [linspace(a(1),a(2),80)];
    y = [linspace(a(3),a(4),80)];
    [X,Y] = meshgrid(x,y);
    
end