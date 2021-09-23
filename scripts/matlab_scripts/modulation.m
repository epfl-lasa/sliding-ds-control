function velocity = modulation(i,dt,robot_state,target,obs)

    n_obs = length(obs);
        
    Gamma = [];
    if n_obs == 0
        velocity = dx(i*dt,robot_state,target.state,target.vel);
    else
        nf = (dx(i*dt,robot_state,target.state,target.vel));
        for j = 1:n_obs
            d = robot_state - obs{j}.c;
            obs_vel = obs{j}.linear_vel + obs{j}.angular_vel*[-d(2);d(1)];
            % Compute the modulation matrix M, Gamma is the value of the Gamma function
            % computed in the position of the robot
            C = [obs{j}.enlarged_x_el;obs{j}.enlarged_y_el];
            [Mod{j},Gamma(j)] = M(robot_state,C,obs{j}.ref);

            % Compute the modulated DS
            Mod_DS(:,j) = Mod{j}*(nf - obs_vel) + obs_vel;
            % Compute the direction fo the modulated DS
            Magn_mod_DS(j) = norm(Mod_DS(:,j));
            
            if  Magn_mod_DS(j) == [0;0]
                n_mod_DS(:,j) = Mod_DS(:,j);
            else
                n_mod_DS(:,j) = Mod_DS(:,j)/Magn_mod_DS(j);
            end
        end 

        % Here we implement the algorithm used to compute the weighs w 
        G = [];
        w = [];
        for j = 1:length(Gamma)
            G = Gamma;
            G(j) = [];
            G = G-1;

            num = prod(G);
            den = 0;
            for k = 1:length(Gamma)
                G = Gamma;
                G(k) = [];
                G = G-1;

                den = den + prod(G);
            end

            w(j) = num/den;
        end

        % Compute the magnitude of the final velocity
        Magn_DS = sum(w.*Magn_mod_DS);

        % Direction of the original DS
        if norm(nf) ~= 0
           nf = nf/norm(nf); 
        else
           nf; 
        end
               
        % Vector orthogonal to nf
        %ef = null(nf(:).');
        ef = [-nf(2);nf(1)];

        % Orthonormal matrix Rf
        Rf = [nf ef];

        % n_hat
        n_hat = Rf'*n_mod_DS;

        % kappa function 
        for j = 1:length(Gamma)
            if n_hat(2,j) >= 0
                k(j) = real(acos(n_hat(1,j)));
            else
                k(j) = -real(acos(n_hat(1,j)));
            end
        end

        % weighted mean k_bar
        k_bar = sum(w.*k);

        % direction vector of the modulated DS
        n_bar = Rf*[cos(norm(k_bar)) sin(norm(k_bar))*sign(k_bar)]';

        velocity = n_bar*Magn_DS;
        
    end

end