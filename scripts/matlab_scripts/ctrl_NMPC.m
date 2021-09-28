function [ctrl, traj] = ctrl_NMPC(Ts,limits,Gains,prediction_steps)
import casadi.*
opti = casadi.Opti(); % Optimization problem
N = prediction_steps; % MPC horizon [SET THIS VARIABLE]
% ???? decision variables ?????????
X = opti.variable(3,N+1); % state trajectory variables
U = opti.variable(2, N); % control trajectory (throttle, brake)
X0 = opti.parameter(3,1); % initial state
REF = opti.parameter(5,1); % reference position [x,y,z,yaw]
%%%%%%%%%%%%%%%%%%%%%%%%
%%%% YOUR CODE HERE %%%%
% Non linear dynamic of the system 
f_not_discrete = @(x,u) unicycle(x,u);
% Runge-Kutta 4 integration
f_discrete = @(x,u) rk4(x,u,Ts,f_not_discrete);

% Gain vectors for states and inputs
Gx = zeros(3,1);
Gu = zeros(2,1);

% Tuned values for state gains
Gx = Gains([1:3]);

% Tuned values for input gains
Gu= Gains([4:5]);              

% Cost function to be minimized
opti.minimize(Gx(1)*(X(1,:)-REF(1))*(X(1,:)-REF(1)).'+...     % x
    Gx(2)*((X(2,:)-REF(2))*(X(2,:)-REF(2)).')+...             % y
    Gx(3)*((X(3,:)-REF(3))*(X(3,:)-REF(3)).')+...             % z
    Gu(1)*(U(1,:)*U(1,:).')+...                               % u1
    Gu(2)*(U(2,:)*U(2,:).'));                                 % u2
                          
opti.subject_to(-limits([1:3]) <= X <= limits([1:3]));
% Input contraints
opti.subject_to(-limits([4:5]) <= U <= limits([4:5]));

opti.subject_to(-limits([6:7])*Ts <= (U(:,1)-REF([4:5])) <= limits([6:7])*Ts);

%System dynamics
opti.subject_to(X(:,1)==X0);
for k = 1:N
   opti.subject_to(X(:,k+1) == f_discrete(X(:,k),U(:,k))); 
end

for k = 1:N-1 
   opti.subject_to(-limits([6:7])*Ts <= (U(:,k+1) - U(:,k)) <= limits([6:7])*Ts);
end

%%%%%%%%%%%%%%%%%%%%%%%%
ctrl = @(x,ref) eval_ctrl(x, ref, opti, X0, REF, X, U);
end

function u = eval_ctrl(x, ref, opti, X0, REF, X, U)
% ???? Set the initial state and reference ????
opti.set_value(X0, x);
opti.set_value(REF, ref);
% ???? Setup solver NLP ??????
ops = struct('ipopt', struct('print_level',0, 'tol', 1e-3), 'print_time', false);
opti.solver('ipopt', ops);
% ???? Solve the optimization problem ????
sol = opti.solve();
assert(sol.stats.success == 1, 'Error computing optimal input');
u = opti.value(U);
% Use the current solution to speed up the next optimization
opti.set_initial(sol.value_variables());
opti.set_initial(opti.lam_g, sol.value(opti.lam_g));
end
