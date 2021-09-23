% Modulation function
function [M,Gamma] = M(x,c,ref)

if x == ref
    M = eye(2);
    Gamma = 0;
else

    dir_vect = c-ref;
    dir_vect = dir_vect./vecnorm(dir_vect);

    % r is the vector that represents the direction from ref and x
    r = x - ref;
    r = r/norm(r);

    [~,idx] = min(vecnorm(r - dir_vect));

    edge_point = c(:,idx);

    if idx == length(c)
        e = c(:,idx) - c(:,idx-1);
    else
        e = c(:,idx) - c(:,idx+1);
    end

    e = e/norm(e);

    % Gamma funcion: d is the denominator and p is the exponential 
    d = norm(edge_point-ref);
    p = 1;    

    Gamma = ((((norm(x-ref)/d).^(2*p)) - 1)/1 +1);

    % E is the matrix formed by r and e
    E = [r e];
    % lam_r and lam_e are the eigenvalues of the matrix D
    lam_r = 1 - 1/Gamma;
    lam_e = 1 + 1/Gamma;

    D = diag([lam_r lam_e]);
    M = E*D*inv(E);
end

end