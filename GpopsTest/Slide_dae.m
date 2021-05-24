function [dae] = Slide_dae(soldae)
global robot;

iphase = soldae.phase;
t = soldae.time;
x = soldae.state;
u = soldae.control;
p = soldae.parameter;

if iphase==1
    x1dot = x(:,2);
    x2dot = u(:,1)/robot.m;
    path = [];
elseif iphase==2
    x1dot = x(:,2);
    x2dot = u(:,1)/robot.m;
    path = [];
end

dae = [x1dot x2dot path];

end

