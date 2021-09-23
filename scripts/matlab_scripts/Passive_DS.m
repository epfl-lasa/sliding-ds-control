classdef Passive_DS
    
properties
    Lambda = diag([0, 0.5]);
    timestep = 0.01;
    robot_mass = 5;
    F_nominal = 30;
end

methods
    function obj = Passive_DS(F_nominal)
        obj.F_nominal = F_nominal;
    end
    
    
    function [Vx, Vy] = getVelocityField(obj, X, Y, Fx, Fy, Vx_MDS, Vy_MDS, Vx_prev, Vy_prev)
        if any(size(X) ~= size(Y))
            error("Size of x and y are not same")
        end
        
        if nargin < 9
            Vx_prev = 0;
            Vy_prev = 0;
        end
        if nargin < 7
            Vx_MDS = zeros(size(X));
            Vy_MDS = zeros(size(X));
        end
        
        size_x = size(X);
        X = X(:);
        Y = Y(:);
        n_x = length(X);
        

        Fx = Fx(:);
        Fy = Fy(:);
        F = [Fx, Fy];
        F_mag = vecnorm(F, 2, 2);
        n_hat = - F ./ F_mag;
        t_hat = [n_hat(:, 2), -n_hat(:, 1)];
        
        Vx_MDS = Vx_MDS(:);
        Vy_MDS = Vy_MDS(:);
        V_cmd = [Vx_MDS, Vy_MDS];
        
        V_prev = [Vx_prev; Vy_prev];
        
        V = zeros(n_x, 2);
        for i = 1:n_x
            Q = [t_hat(i,:); n_hat(i,:)].';
            D = Q * obj.Lambda * Q.';

            V(i,:) = obj.timestep / obj.robot_mass * ( ...
                - D * V_prev ...
                + obj.F_nominal * n_hat(i,:).' ...
                - F_mag(i) * n_hat(i,:).' ...
            ) + (t_hat(i,:) *  V_cmd(i,:).') * t_hat(i,:).';
        
            if (n_hat(i,:) *  V_cmd(i,:).') < 0
                V(i,:) = V(i,:) + (n_hat(i,:) *  V_cmd(i,:).') * n_hat(i,:);
            end
        end
        
        Vx = V(:, 1);
        Vx(F_mag == 0) = Vx_MDS(F_mag == 0);
        
        Vy = V(:, 2);
        Vy(F_mag == 0) = Vy_MDS(F_mag == 0);
        
        Vx = reshape(Vx, size_x);
        Vy = reshape(Vy, size_x);

    end
end

end