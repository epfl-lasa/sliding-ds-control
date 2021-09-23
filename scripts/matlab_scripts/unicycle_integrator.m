function next_pose = unicycle_integrator(initial_pose,inputs,board_step,dt)

    for kk = 1:board_step
        f = @(x,u) unicycle(x,u);
        next_pose = rk4(initial_pose,inputs,dt,f);

        initial_pose = next_pose;
    end
end