classdef Obstacle
    
properties
    x = 0
    y = 0
    r = 1
    vx = 0
    vy = 0
    k = 100
    robot_radius = 1
    r_apparent
%     COLORS = struct(...
%         'force_field', '#D45544', ...
%         'contact_zone', '#95E37F', ...
%         'flex_obs', '#DB8945', ...
%         'rigid_obs', '#C4C4C4');
    COLORS = struct(...
        'force_field', '#D45544', ...
        'contact_zone', '#D4D4D4', ...
        'flex_obs', '#B0B0B0', ...
        'rigid_obs', '#777777');
end

methods
    %% Constructor
    function obj = Obstacle(x, y, r, k, robot_radius)
        if nargin == 5
            obj.x = x;
            obj.y = y;
            obj.r = r;
            obj.k = k;
            obj.robot_radius = robot_radius;
        elseif nargin == 4
            obj.x = x;
            obj.y = y;
            obj.r = r;
            obj.k = k;
        elseif nargin == 3
            obj.x = x;
            obj.y = y;
            obj.r = r;
        elseif nargin == 2
            obj.x = x;
            obj.y = y;
        end
        
        obj.r_apparent = obj.r + obj.robot_radius;
    end
    
    %% Step
    function obj = step(obj, dt)
        obj.x = obj.x + obj.vx * dt;
        obj.y = obj.y + obj.vy * dt;
    end
    
    %% Get Force
    function [FX, FY] = getForce(obj, X, Y)
        n_obj = length(obj);

        if any(size(X) ~= size(Y))
            error("Size of x and y are not same")
        end
        size_x = size(X);
        X = X(:);
        Y = Y(:);
        n_x = length(X);

        force_x = zeros(n_x, n_obj);
        force_y = zeros(n_x, n_obj);
        for i = 1:n_obj
            dx = X(:) - obj(i).x;
            dy = Y(:) - obj(i).y;
            dr = vecnorm([dx, dy], 2, 2);

            penetration = obj(i).r_apparent - dr;
            force_mag = obj(i).k .* penetration;

            inactive = penetration < 0;     % No penetration of obstacle
            force_mag(inactive) = 0;

            force_x(:, i) = force_mag .* dx ./dr;
            force_y(:, i) = force_mag .* dy ./dr;
        end

        force_x = sum(force_x, 2);
        force_y = sum(force_y, 2);

        FX = reshape(force_x, size_x);
        FY = reshape(force_y, size_x);
    end
    
    function [FX, FY] = getRealForce(obj, X, Y)
        n_obj = length(obj);

        if any(size(X) ~= size(Y))
            error("Size of x and y are not same")
        end
        size_x = size(X);
        X = X(:);
        Y = Y(:);
        n_x = length(X);

        force_x = zeros(n_x, n_obj);
        force_y = zeros(n_x, n_obj);
        for i = 1:n_obj
            dx = X(:) - obj(i).x;
            dy = Y(:) - obj(i).y;
            dr = vecnorm([dx, dy], 2, 2);

            penetration = obj(i).r - dr;
            force_mag = obj(i).k .* penetration;

            inactive = penetration < 0;     % No penetration of obstacle
            force_mag(inactive) = 0;

            force_x(:, i) = force_mag .* dx ./dr;
            force_y(:, i) = force_mag .* dy ./dr;
        end

        force_x = sum(force_x, 2);
        force_y = sum(force_y, 2);

        FX = reshape(force_x, size_x);
        FY = reshape(force_y, size_x);
    end
    
    %% Draw Obstacle
    function draw(obj, F_nominal)
        n_obj = length(obj);
        hold on;
        plot([obj.x], [obj.y], 'k.', 'MarkerSize', 10);
        for i = 1:n_obj
            sliding_r = obj(i).r_apparent - F_nominal ./ obj(i).k;
            pos = [obj(i).x - sliding_r, ...
                   obj(i).y - sliding_r, ...
                   2 * sliding_r, ...
                   2 * sliding_r];
            rectangle(...
                'Position', pos, ...
                'Curvature', [1, 1], ...
                'FaceColor', obj(i).COLORS.contact_zone, ...
                'LineStyle', 'none');
            
            pos = [obj(i).x - obj(i).r, ...
                   obj(i).y - obj(i).r, ...
                   2 * obj(i).r, ...
                   2 * obj(i).r];
            rectangle(...
                'Position', pos, ...
                'Curvature', [1, 1], ...
                'LineWidth', 2, ...
                'EdgeColor', '#000000', ...
                'FaceColor', obj(i).COLORS.flex_obs, ...
                'LineStyle', '--');
            
            collision_r = obj(i).r - F_nominal ./ obj(i).k;
            pos = [obj(i).x - collision_r, ...
                   obj(i).y - collision_r, ...
                   2 * collision_r, ...
                   2 * collision_r];
            rectangle(...
                'Position', pos, ...
                'Curvature', [1, 1], ...
                'FaceColor', obj(i).COLORS.rigid_obs, ...
                'LineStyle', 'none');
        end
    end
    
    function draw_force(obj, X, Y, nstep)
        if nargin < 4
            nstep = 4;            
        end
        n_obj = length(obj);
        for i = 1:n_obj
            X = X(1:nstep:end, 1:nstep:end);
            Y = Y(1:nstep:end, 1:nstep:end);
            [Fx, Fy] = obj.getRealForce(X, Y);
            quiver(X, Y, Fx, Fy, 'Color', obj(i).COLORS.force_field); 
            % h = streamslice(X, Y, Fx, Fy, 10);
            % h.set('Color', obj(i).COLORS.force_field);
        end
    end
end
    
end