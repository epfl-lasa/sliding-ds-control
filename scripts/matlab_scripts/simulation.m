%% Simulating a Mobile Robot Contact Control and Dynamic Obstacles Avoidance
% Author: Diego F. Paez G., Vaibahv Gupta
% Date: Jan 2021
% Unicycle model and nMPC from Gabriele Coppola (Master thesis - LASA 2020)


%% Envirnoment Setup
clearvars;
close all;
clc;

global SAVE_PLOTS

SAVE_PLOTS = true;
% addpath('/path-to/casadi');

 simulate();

%% Simulate Function
 
function simulate()
    DEBUG = 0;    
    UNKNOWN_OBS_VELOCITY = false;
    COLORS = struct(...
        'goal'  , '#08A115', ...
        'obs'   , '#A82311', ...
        'MDS'   , '#6B8DFA', ...
        'robot' , '#000000');

    %% Simualtion Parameters
    Tsim = 7;          % Simulation time
    f_DS = 20;      % DS frequency
    f_control = 20; % Control frequency
    f_int = 30;    % Integration frequency

    % Simulations steps
    simulation_step = Tsim*f_DS;
    control_step = f_control/f_DS;
    integration_step = f_int/f_control;
    dt = 1/f_DS;

    HL_max_vel = 1.5;       % High level maximum velocity
    % HL_max_ang_speed = 2;   % High level maximum angular speed

    %% Obstacles and pose definition
    %Final goal
    goal = [3;3];
    Goal.state = goal;
    Goal.vel = [0;0];

    %Robot starting point
    robot_pose = [-1.1;0.3;0];
    robot_state = robot_pose([1,2]);

    %Desired pose (target pose)
    target_pose = [3;3;pi/4];
    target_state = target_pose([1,2]);

    % Radius of the robot
    robot_radius = 0.3;
    % Wheel radius
    wheel_rad = 0.152;

    %Number of points used to define the contour of each obstacle
    n_points = 300;
    
    % Obstacle eps
    obs_eps = 0;
    
    % Obstacle description
    obs{1} = obs_description(0.2, 0.2, ...
        [1;4], 0, [0;4], obs_eps, ...
        robot_radius, n_points, [0.3;0], 0);

    obs{2} = obs_description(0.2, 0.2, ...
        [-0.3;0.8], 0, [0.0;2], obs_eps, ...
        robot_radius, n_points, [0.2;-1.7], 0);
 
    obs{3} = obs_description(0.3, 0.3, ...
        [2;-1], 0, [2.5;-1], obs_eps, ...
        robot_radius, n_points, [0;0.5], -2);

    % number of obstacles 
    n_obs = length(obs);

    % Init Figure
    figure;
    
    [axis_corner, X, Y] = map_mesh(obs, robot_state, target_state, goal, robot_radius);
    axis(axis_corner)
    hold off

    target_velocity = 0;

    %% Passive DS + Obstacles

    F_nominal = 30;
    
    for j = n_obs:-1:1
        obstacles(j) = Obstacle( ...
            obs{j}.c(1), ...
            obs{j}.c(2), ...
            obs{j}.rx, ...
            1e5, ...
            robot_radius);
    end

    n_ghost_obstacles = 1;
    for i = n_ghost_obstacles:-1:1
%         ghost_obstacles(i) = Obstacle(1.5, 2.2, 0.5, 200, robot_radius);
        ghost_obstacles(i) = Obstacle(1.45, 1.9, 0.5, 200, robot_radius);
    end
    ghost_obstacles(1).vx = 0;%-0.05;
    ghost_obstacles(1).vy = 0;%.05;

    passive_ds = Passive_DS(F_nominal);
    passive_ds.timestep = dt;
    passive_ds.robot_mass = 5;
    passive_ds.Lambda = diag([0, 0.5]);

    % Init previous velocity
    velocity_prev = [0; 0];

    %% Record Video
    movieflag = 1;
    vid_name = "figures/qolo_mds_passiveDS_damped_static";

    % Video Format
    %  * 'MPEG-4' for Mac
    %  * 'Motion JPEG AVI' for Linux
    vid_format = 'MPEG-4';

    if movieflag
        vidobj = VideoWriter(vid_name, vid_format);
        vidobj.FrameRate = 30;
        open(vidobj);
    end

    %% Simulation

    % Initial time of the simulation
    Time = zeros(1, simulation_step+1);

    % Control inputs
    U_v = zeros(1, simulation_step+1);
    U_w = zeros(1, simulation_step+1);
    U_r = zeros(1, simulation_step+1);
    U_l = zeros(1, simulation_step+1);

    Theta = zeros(1, simulation_step+1);                % Desired orientation
    Robot_desired_state = zeros(2, simulation_step+1);  % Desired robot state  
    Robot_orientation = zeros(1, simulation_step+1);    % Real orientation

    Collision_force = zeros(1, simulation_step+1);
    Penetration = zeros(1, simulation_step+1);
    velocity_vec = zeros(1, simulation_step+1);
    
    % Goal
    Goal.state = goal;
    Goal.vel = [0; 0];

    [axis_corner, mesh_points] = updateMesh();
    flagSnap = false;
    for i = 1:simulation_step 
        Time(i+1) = Time(i) + dt;   % Time steps
        t = i*dt;                   % Time instant

        Target = updateTarget(i);
        [~, ~, U1, U2] = getModulationFlow(i);

        %% High-level Controller
        % Compute the DS for the robot
        velocity = modulation(i, dt, robot_state(:,i), Target, obs);

        % Velocity saturation (ensure reasonable MDS velocities)
        if norm(velocity) > HL_max_vel
            velocity = HL_max_vel*velocity/norm(velocity);
        end

        % Passive DS
        [Fx, Fy] = ghost_obstacles.getForce(robot_state(1,i), robot_state(2,i));
        [vx, vy] = passive_ds.getVelocityField(...
            robot_state(1,i), robot_state(2,i), ...
            Fx, Fy, ...
            velocity(1), velocity(2), ...
            velocity_prev(1), velocity_prev(2));
        
        if UNKNOWN_OBS_VELOCITY
            velocity = [vx; vy];
        else
            velocity = [vx + ghost_obstacles(1).vx; vy + ghost_obstacles(1).vy];
        end

        % Velocity saturation
        if norm(velocity) > HL_max_vel
            velocity = HL_max_vel*velocity/norm(velocity);
        end

        % Update previous velocity
        velocity_prev = velocity;
        velocity_vec(i+1) = norm(velocity, 2);
        Collision_force(i+1) = norm([Fx, Fy], 2);
        Penetration(i+1) = Collision_force(i+1) / ghost_obstacles(1).k;
        if DEBUG
            if Collision_force(i+1) > 0
                fprintf("Collision Force = %6.3f\n", Collision_force(i+1));
            end
        end

        %% Error and state update

        % Sigmoid parameters
        slope = 50;     % slope of the sigmoid
        near = 0.1;     % distance at witch we start feeling the sigmoid effect
        dist = ...      % distance between robot and desired final position
            norm(robot_state(:,i) - target_state) - near;

        % Desired position of the robot: computed using the high level
        % controller that does not include the non-holonomic constraints
        robot_state(:, i+1) = robot_state(:, i) + velocity*dt; 
        Robot_desired_state(:, i+1) = robot_state(:, i+1);
        % Desired orientation of the robot: we use the sigmoid function to smoothly pass from
        % the orientation given by the high level controller (given by the orientation 
        % of the velocity vector) to the desired final orientation (chosen at the beginning)
        theta = (atan2(velocity(2),velocity(1)))*1./(1+exp(-slope*dist)) + target_pose(3)*1./(1+exp(slope*dist));

        % theta has to be continuous
        if ~isempty(Theta)
            while abs(Theta(end)-theta) > pi
               if  (Theta(end)-theta) > pi
                   theta = theta + 2*pi;
               else
                   theta = theta - 2*pi;
               end
            end
        end   

        % We round theta in order to avoid too many oscillation in the control
        % inputs
        theta = round(theta,2);

        % Desired next pose
        goal_pose = [robot_state(:,i+1); theta];

        Theta(i+1) = theta;

        %% Low level control: the control inputs are generated
        % - u_v: linear velocity control inputs applied in dt time
        % - u_w: angular speed control inputs applied in dt time

        A = [ cos(robot_pose(3,:))  , 0;
              sin(robot_pose(3,:))  , 0;
              0                     , 1];
        err = (goal_pose - robot_pose) / dt;
        open_loop_inputs = repmat(A \ err, 1, control_step);

        for k = 1:control_step
            inputs = open_loop_inputs(:,k);
            U_v(i+1) = inputs(1);
            U_w(i+1) = inputs(2);

            inputs_wheel(1,1) =  (inputs(1) - robot_radius*inputs(2))/wheel_rad;
            inputs_wheel(2,1) =  (inputs(1) + robot_radius*inputs(2))/wheel_rad;
            U_l(i+1) = inputs_wheel(1);
            U_r(i+1) = inputs_wheel(2);

            next_pose = unicycle_integrator( ...
                robot_pose, ...
                inputs, ...
                integration_step, ...
                1/f_int);

            % Update of robot pose
            robot_pose = next_pose;
        end

        %% Update Environment
        % Update of robot position
        robot_state(:,i+1) = robot_pose([1,2]);
        Robot_orientation(:, i+1) = next_pose(3);

        target_state(:,i+1) = target_state(:,i)  + target_velocity*dt;
        target_pose([1,2],:) = target_state(:,i+1);
%         target_pose(3) = atan2(target_velocity(2),target_velocity(1));

        % Check if the robot hit the obstacles
        for j = 1:n_obs
           if checkInsideEllipse(obs{j}.rx + robot_radius,obs{j}.ry + robot_radius,obs{j}.c,obs{j}.theta,robot_state(:,i+1))
                 fprintf('Hit the obstacle\n') 
           end
        end

        % Update the obstacles
        for j = 1:n_obs    
            %%%%%%%%%%%%%%%%
            % Maybe here you can modify the eps of each obstacle
            % obs{j}.eps = ...
            %%%%%%%%%%%%%%%%
            obs{j}.c = obs{j}.c + obs{j}.linear_vel*dt;
            obs{j}.ref = obs{j}.c;
            obs{j}.theta = obs{j}.theta + obs{j}.angular_vel*dt;

            [obs{j}.x_el,obs{j}.y_el] = getEllipse(obs{j}.rx,obs{j}.ry,obs{j}.c,obs{j}.theta,n_points);
            [obs{j}.enlarged_x_el,obs{j}.enlarged_y_el] = getEllipse(obs{j}.rx+robot_radius + obs{j}.eps,obs{j}.ry+robot_radius + obs{j}.eps,obs{j}.c,obs{j}.theta,n_points);
            
            obstacles(j).x = obs{j}.c(1);
            obstacles(j).y = obs{j}.c(2);
            ghost_obstacles = ghost_obstacles.step(dt);
        end

        %% Plot update  
        cla;
        hold on;
        drawObstacles();
        drawRobot();      
                
        % s = streamslice(X,Y,U1_MDS,U2_MDS, 2);
        s = streamslice(X, Y, U1, U2, 2);
        set(s, 'Color', COLORS.MDS,'LineWidth', 0.02 );    
        
        axis(axis_corner)
        hold off

        if movieflag
            % capture current figure
            frame = getframe(gcf);
            writeVideo(vidobj,frame)
            if Time(i+1)>=(Tsim/2) && ~flagSnap
                set(gcf,'PaperPositionMode', 'auto');   % Required for exporting graphs
                saveas(gcf,'snapshot_SDS','epsc');
                flagSnap = true;
            end
        else
            pause(0.001);
        end
    end

    if movieflag
        close(vidobj);
    end

    %% End plots
%     AxisPlots = [0 2000 0 1];
    PicSize = [10 10 480 780];
    Fonts = 'Times New Roman';
%     FaceALphas = 0.18;
%     colorAlpha = 0.85;
    FontSizes = 24;
%     MarkersSizes = 14;
    LinesWidths = 2.8;
    
    presults = figure();
%     subplot(211);
%     plot(Time, Penetration);
%     title('Penetration of Ghost Obstacle')
%     xlabel('t [s]')
%     ylabel('Penetration [m]')
%     grid on
    subplot(211);
    plot(Time, velocity_vec,...
        'linewidth',LinesWidths);
%     title('Velocity Modulation During Contact')
    xlabel('time [s]')
    ylabel('Velocity [m/s]')
    grid off
    set(gcf, 'Position', PicSize);
	set(gcf, 'PaperPositionMode', 'auto');
	hold on;
	hXLabel=xlabel('time [s]');
	hYLabel=ylabel('Velocity [m/s]');
	set(gca, ...
        'Box'         , 'off'       , ...
        'TickDir'     , 'out'       , ...
        'TickLength'  , [.02 .02]   , ...
        'XMinorTick'  , 'off'       , ...
        'YMinorTick'  , 'off'       , ...
        'YGrid'       , 'off'       , ...
        'XColor'      , 'k'         , ...
        'YColor'      , 'k'         , ...%           'XTick'       , 0:round(m/10):(m+1), ... %           'YTick'       , Ymin:round((Ymax-Ymin)/10):Ymax, ...
        'LineWidth'   , 1.5         );

	subplot(212);
	plot(Time, Collision_force,...
        'linewidth',LinesWidths);
%     title('Collision force from ghost obstacle')
    xlabel('t [s]')
    ylabel('Collision force [N]')
    grid off
    
    set(gcf, 'Position', PicSize);
    set(gcf,'PaperPositionMode', 'auto');
    hold on;
    hXLabel=xlabel('time [s]');
    hYLabel=ylabel('Collision force [N]');
    set(gca, ...
        'Box'         , 'off'     , ...
        'TickDir'     , 'out'     , ... % 
        'TickLength'  , [.02 .02] , ...
        'XMinorTick'  , 'off'      , ...
        'YMinorTick'  , 'off'      , ...
        'YGrid'       , 'off'      , ...
        'XColor'      , 'k', ...
        'YColor'      , 'k', ...%           'XTick'       , 0:round(m/10):(m+1), ... %           'YTick'       , Ymin:round((Ymax-Ymin)/10):Ymax, ...
        'LineWidth'   , 1.5         );

    set([hXLabel, hYLabel]  , ...
        'FontName',  Fonts,...
        'FontSize',  FontSizes,...
        'color',     [0 0 0]);
    if SAVE_PLOTS
        set(gcf,'PaperPositionMode', 'auto');   % Required for exporting graphs
        saveas(presults,'Results_force_contact','epsc');
    end
                    
    %% Helper Functions
    function drawObstacles()
        for kk = 1:n_obs
            plot(obs{kk}.enlarged_x_el, obs{kk}.enlarged_y_el, ...
                'LineStyle', '--', ...
                'Color', COLORS.obs, ...
                'LineWidth', 2);
            plot(obs{kk}.x_el, obs{kk}.y_el, ...
                'Color', COLORS.obs,...
                'LineWidth', 2);
            plot(obs{kk}.c(1), obs{kk}.c(2), ...
                'Marker', '*', ... ...
                'Color', COLORS.obs);
            plot(obs{kk}.ref(1), obs{kk}.ref(2), ...
                'Marker', '*', ... ...
                'Color', COLORS.obs);
        end
        ghost_obstacles.draw(F_nominal);
        
        ghost_obstacles.draw_force(X, Y, 2);
        obstacles.draw_force(X, Y, 2);
    end

    function drawRobot()
        [x_rob, y_rob] = getEllipse(...
            robot_radius, robot_radius, ...
            robot_state(1:2,end), 0);
        plot(x_rob, y_rob, ...
            'Color', COLORS.robot, ...
            'LineStyle', '--')
        wheel_points = wheelchair_draw(robot_radius, robot_pose(1:2), robot_pose(3));
        plot(wheel_points(1,:), wheel_points(2,:), ...
            'Color', COLORS.robot, ...
            'LineStyle', '-', ...
            'LineWidth', 2);
        plot(robot_state(1,end), robot_state(2,end), ...
            'Marker', 'o', ...
            'MarkerFaceColor', COLORS.robot);
        arrow(gca, robot_state(1,end), robot_state(2,end), ...
            cos(next_pose(3,end)), sin(next_pose(3,end)), ...
            COLORS.robot);
        
        [x_rob_goal, y_rob_goal] = getEllipse(...
            robot_radius, robot_radius, ...
            target_pose(1:2), 0);
        plot(x_rob_goal, y_rob_goal, ...
            'Color', COLORS.goal, ...
            'LineStyle', '--');        
        wheel_points = wheelchair_draw(robot_radius, target_pose(1:2), target_pose(3));
        plot(wheel_points(1,:), wheel_points(2,:), ...
            'Color', COLORS.goal, ...
            'LineStyle', '-', ...
            'LineWidth', 2);
        plot(goal(1), goal(2), ... 
            'Marker', 'p', ...
            'MarkerFaceColor', COLORS.goal, ...
            'MarkerSize',10);
        arrow(gca, target_pose(1,end), target_pose(2,end), ...
            cos(target_pose(3,end)), sin(target_pose(3,end)), ...
            COLORS.goal);
    end
    
    function [axis_corner, mesh_points] = updateMesh()
        [axis_corner, X, Y] = map_mesh(obs,robot_state,target_state,goal,robot_radius);
        mesh_points = [X(:), Y(:)];
    end

    function Target = updateTarget(i)
        % The target has a DS to avoid the obstacles
        target_velocity = modulation(i, dt, target_state(:,i), Goal, obs);
        % target_velocity = [0; 0];

        % Velocity saturation: the maximum target velocity should be smaller
        % wrt the maximum velocity of the robot 
        coeff = i/100;
        if coeff > 0.6
            coeff = 0.6;
        end
        if norm(target_velocity) > HL_max_vel*coeff
            target_velocity = HL_max_vel*coeff*target_velocity/norm(target_velocity);
        end

        Target.state = target_state(:,i);
        Target.vel = target_velocity;
    end

    function [U1_MDS, U2_MDS, U1, U2] = getModulationFlow(i)
        % Compute the modulation for each point of the mesh
        V = zeros(2, length(mesh_points));
        for jj = 1:length(mesh_points)
            V(:,jj) = modulation(i, dt, mesh_points(jj,:)', Target, obs);
            % If the mesh_point is inside the obstacle, set the modulation to 0 
            for kk = 1:length(obs)
            	if checkInsideEllipse(...
                        obs{kk}.rx + obs{kk}.eps + robot_radius, ...
                        obs{kk}.ry + obs{kk}.eps + robot_radius, ...
                        obs{kk}.c, obs{kk}.theta, mesh_points(jj,:)')
                    V(:,jj) = [0; 0];
                end
            end
            % Velocity saturation (ensure reasonable MDS velocities)
            if norm(V(:,jj)) > HL_max_vel
                V(:,jj) = HL_max_vel*V(:,jj)/norm(V(:,jj));
            end
        end

        U1_MDS = reshape(V(1,:),size(X,1),size(X,2));
        U2_MDS = reshape(V(2,:),size(X,1),size(X,2));

        % Passive DS (Known Obstacles)
        [Fx, Fy] = obstacles.getForce(X, Y);
        [U1, U2] = passive_ds.getVelocityField(X, Y, Fx, Fy, U1_MDS, U2_MDS);
        
        % Passive DS (Ghost Obstacles)
        [Fx, Fy] = ghost_obstacles.getForce(X, Y);
        [U1, U2] = passive_ds.getVelocityField(X, Y, Fx, Fy, U1, U2);
        
    end

    function h = arrow(ax, x0, y0, dx, dy, color, scale)
        if nargin < 7
            scale = 0.8;
        end

        h = annotation('arrow');
        h.Parent = ax;
        h.Position = [x0, y0, scale*dx, scale*dy];
        h.Color = color;
        h.LineWidth = 2;
        h.HeadStyle = 'plain';
        h.HeadLength = scale*10;
        h.HeadWidth = scale*8;
    end
    
end