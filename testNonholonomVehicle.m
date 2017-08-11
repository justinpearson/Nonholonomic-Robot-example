%% Test Nonholonomic Vehicle class
% * Justin Pearson
% * Aug 11, 2017

clear all; fclose all; clc


%% Example 1: Create robot & let it drive

dt = .1; % timestep
x0 = 2;
y0 = 3;
th0 = pi/4;
vl = .2;
vr = .5;
robot = NonholonomicVehicle(dt,x0,y0,th0,vl,vr);

figure(1); clf
for i=0:9
    plot(robot.X,robot.Y,'ko','markersize',20); hold on;
    text(robot.X,robot.Y,num2str(i));
    robot.timestep();
end

axis square
title('Roomba position at each timestep')
drawnow



%% Example 2: Circle-tracing controller
% Controller: A line-detecting sensor says whether the line is to the right
% (0) or left (1) of the robot. The controller adjusts the wheel velocities
% a litle in response to this, in order to turn the robot a little.



figure(1); clf;
DIAM = 4;
rectangle('Position',DIAM*[-1/2,-1/2,1,1],'Curvature',[1 1]); hold on;

dt = .1;
x0 = DIAM/2 - .1;
y0 = 0;
th0 = pi/2;
vl = .4;
vr = .4;
robot = NonholonomicVehicle(dt,x0,y0,th0,vl,vr);

sensor = @(x,y) double(hypot(x,y) > DIAM/2);

TMAX = 13; % sec, len of sim.
n_iters = TMAX / dt;
xs = zeros(n_iters,1);
ys = zeros(size(xs));

for i=1:n_iters
    
    xs(i) = robot.X;
    ys(i) = robot.Y;
    
    robot.timestep();  % run dynamics
    s = sensor(robot.X,robot.Y);  % sense
        
    % Control:
    Vnom = .3;
    if s==0 % line is to the right; need to turn right a bit
        Vdiff = -.1;
    else
        % line is to the left; need to turn left a little
        Vdiff = .1;
    end
    robot.setWheelVelocities(Vnom-Vdiff,Vnom+Vdiff);
    
end

plot(xs,ys,'.'); hold on;
text(xs(1),ys(1),'START','color','r')
text(xs(end),ys(end),['END (t=' num2str(TMAX) ')'],'color','g')
axis square
axis(DIAM/2*1.1*[-1,1,-1,1])
title('Robot trying to follow a circle.')
