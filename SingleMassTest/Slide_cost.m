function [Mayer, Lagrange] = Slide_cost(solcost)
global robot;
i_phase = solcost.phase;
t0 = solcost.initial.time;
x0 = solcost.initial.state;
tf = solcost.terminal.time;
if i_phase == 1
    xf = [5; 0];
else
    xf = [8; 0];
end
t = solcost.time;
x = solcost.state;
u = solcost.control;
p = solcost.parameter;
N = size(x,1);

% weight
S = [1 1; 1 1];
% the sum of tau
Mayer = dot(xf, S*xf);
Lagrange = dot(u,u,2);
end