function obs = obs_description(rx,ry,c,theta,ref,eps,robot_radius,n_points,linear_vel,angular_vel)

    if nargin == 8
        linear_vel = [0;0];
        angular_vel = 0;
    end
        
    obs.rx = rx;
    obs.ry = ry;
    obs.c = c;
    obs.theta = theta;
    obs.ref = ref;
    obs.linear_vel = linear_vel;
    obs.angular_vel = angular_vel;
    [obs.x_el,obs.y_el] = getEllipse(obs.rx,obs.ry,obs.c,obs.theta,n_points);
    obs.eps = eps;
    [obs.enlarged_x_el,obs.enlarged_y_el] = getEllipse(obs.rx+robot_radius+obs.eps,obs.ry+robot_radius+obs.eps,obs.c,obs.theta,n_points);
end