clear all
close all
clc

% Define the end-effector position and orientation
x = 0.5;
y = 0.5;
yaw = pi/4;

% Define the manipulator parameters
l1 = 1;
l2 = 1;
l3 = 0.1;
branch = 0;

% Compute the joint angles using the inverse kinematics function
[q1, q2, q3] = inverseKinematics3R(x, y, yaw,l1,l2,l3,branch);
%[q1_2, q2_2, q3_2] = inverseKinematics3R(x, y, yaw,l1,l2,l3,0);

% Compute the joint positions
x1 = 0;
y1 = 0;
x2 = l1*cos(q1);
y2 = l1*sin(q1);
x3 = x2 + l2*cos(q1+q2);
y3 = y2 + l2*sin(q1+q2);
x4 = x3 + l3*cos(q1+q2+q3);
y4 = y3 + l3*sin(q1+q2+q3);

% Compute the joint positions
%x1_2 = 0;
%y1_2 = 0;
%x2_2 = l1*cos(q1_2);
%y2_2 = l1*sin(q1_2);
%x3_2 = x2_2 + l2*cos(q1_2+q2_2);
%y3_2 = y2_2 + l2*sin(q1_2+q2_2);
%x4_2 = x3_2 + l3*cos(q1_2+q2_2+q3_2);
%y4_2 = y3_2 + l3*sin(q1_2+q2_2+q3_2);

%disp("x1,y1");disp(x1);disp(y1);
%disp("x2,y2");disp(x2);disp(y2);
%disp("x3,y3");disp(x3);disp(y3);
%disp("x4,y4");disp(x4);disp(y4);

% Plot the manipulator
plot([x1,x2,x3,x4],[y1,y2,y3,y4],'o-','LineWidth',2,'MarkerSize',3,'MarkerFaceColor','b');
%hold on
%plot([x1_2,x2_2,x3_2,x4_2],[y1_2,y2_2,y3_2,y4_2],'o-','LineWidth',2,'MarkerSize',3,'MarkerFaceColor','r');
axis equal;
xlim([-2,2]);
ylim([-2,2]);
xlabel('x');
ylabel('y');
title('3R Manipulator');
