clear;
clc;
close all;

%% Robot Configuration
global robot;
robot = LoadRobot();
GUESS_DT = 0.01;
robot.t_guess = [0; 2; 4];

%% The required fields
% a string containing the name of the problem.
model.name = 'Slide';
% a structure whose elements contain the names of the user-defined function in the problem.
model.funcs.cost = 'Slide_cost';
model.funcs.dae = 'Slide_dae';
model.funcs.link = 'Slide_link';
% an array of structures that contains the information about the lower and upper limits on the variables and constraints in each phase of the problem
%% Stage 1
i_phase = 1;
model.limits(i_phase).meshPoints = [];
model.limits(i_phase).nodesPerInterval = [];
model.limits(i_phase).time.min = [0; 0];
model.limits(i_phase).time.max = [5; 5];
model.limits(i_phase).state.min = [0, -10, 4.9; 0, -10, -0.1];
model.limits(i_phase).state.max = [0, 10, 5.1; 0, 10, 0.1];
model.limits(i_phase).control.min = [-20];
model.limits(i_phase).control.max = [20];
model.limits(i_phase).parameter.min = [];
model.limits(i_phase).parameter.max = [];
model.limits(i_phase).path.min = [];
model.limits(i_phase).path.max = [];
model.limits(i_phase).event.min = [];
model.limits(i_phase).event.max = [];
model.limits(i_phase).duration.min = [5];
model.limits(i_phase).duration.max = [5];
model.guess(i_phase).time = (robot.t_guess(i_phase):GUESS_DT:robot.t_guess(i_phase+1))';
point_num = length(model.guess(i_phase).time);
model.guess(i_phase).state = zeros(point_num,2);
model.guess(i_phase).control = zeros(point_num,1);
for i=1:point_num
    model.guess(i_phase).state(i,:) = [0; 0];
    model.guess(i_phase).control(i,:) = 1.0;
end
model.guess(i_phase).parameter = [];
model.limits(i_phase).dependencies = [];

%% two leg push
i_phase = 2;
model.limits(i_phase).meshPoints = [];
model.limits(i_phase).nodesPerInterval = [];
model.limits(i_phase).time.min = [5; 5];
model.limits(i_phase).time.max = [10; 10];
model.limits(i_phase).state.min = [4.8, -10, 7.9; -0.1, -10, -0.1];
model.limits(i_phase).state.max = [5.1, 10, 8.1; 0.1, 10, 0.1];
model.limits(i_phase).control.min = [-20];
model.limits(i_phase).control.max = [20];
model.limits(i_phase).parameter.min = [];
model.limits(i_phase).parameter.max = [];
model.limits(i_phase).path.min = [];
model.limits(i_phase).path.max = [];
model.limits(i_phase).event.min = [];
model.limits(i_phase).event.max = [];
model.limits(i_phase).duration.min = [5];
model.limits(i_phase).duration.max = [5];
model.guess(i_phase).time = (robot.t_guess(i_phase):GUESS_DT:robot.t_guess(i_phase+1))';
point_num = length(model.guess(i_phase).time);
model.guess(i_phase).state = zeros(point_num,2);
model.guess(i_phase).control = zeros(point_num,1);
for i=1:point_num
    model.guess(i_phase).state(i,:) = [0; 0];
    model.guess(i_phase).control(i,:) = 1.0;
end
model.guess(i_phase).parameter = [];
model.limits(i_phase).dependencies = [];

%% The optional fields
% an array of structures that contains the information about the lower and upper limits of the linkage constraints
i_link = 1;
model.linkages(i_link).min = [0; 0];
model.linkages(i_link).max = [0; 0];
model.linkages(i_link).left.phase = 1;
model.linkages(i_link).right.phase = 2;

%%
% Specifies the parameters to be used by the hp{adaptive method refinement algorithm
% model.mesh =[];
% a string that indicates whether or not the user would like the optimal control problem to be scaled automatically before it is solved.
model.autoscale = 'on';     % off, on
% a string indicating differentiation method to be used.
model.derivatives = 'finite-difference';     % finite-difference, complex, automatic, automatic-INLAB��analytic,
% a flag to check user defined analytic derivatives
% model.checkDerivatives = 0; % 0, 1
model.maxIteration = 1e6;   %
model.printoff = 'true';   % 0, 1
% two element array specifiying the NLP solver Optimality and Feasibility Tolerances
model.tolerance = [1e-6, 2e-6];       % [1e-6, 2e-6]
addpath(genpath('./gpops'));
ts = cputime;
[output,gpopsHistory]=gpops(model);

solution = output.solution;
solutionPlot = output.solutionPlot;
te = cputime;
robot.solve_time = te - ts;

figure
subplot(2,3,1)
hold on
plot(solutionPlot(1).time,solutionPlot(1).state(:,1))
plot(solutionPlot(2).time,solutionPlot(2).state(:,1))
xlabel('Time (s)')
ylabel('Position (m)')
legend('Stage 1','Stage 2')
title('Position')

subplot(2,3,2)
hold on
plot(solutionPlot(1).time,solutionPlot(1).state(:,2))
plot(solutionPlot(2).time,solutionPlot(2).state(:,2))
xlabel('Time (s)')
ylabel('Velocity (m/s)')
legend('Stage 1','Stage 2')
title('Velocity')

subplot(2,3,3)
hold on
plot(solutionPlot(1).time,solutionPlot(1).control(:,1))
plot(solutionPlot(2).time,solutionPlot(2).control(:,1))
xlabel('Time (s)')
ylabel('Force (N)')
legend('Stage 1','Stage 2')
title('Control Force')






