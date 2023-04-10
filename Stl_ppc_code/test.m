clear all
close all
clc

pid = 0;
stl_ppc = 1;

p_x = -1;
p_y = 1.2;
yaw = pi/4;

v_x = 0;
v_y = 0;
w = 0;

x_obj_i = [p_x;p_y;yaw];
v_obj_i = [v_x;v_y;w];

x_des = [0.5;0.5;pi/4];
v_des = [0;0;0];

%% STL Specifications
stl(1).type = 'G';
stl(1).time_range = [0 10];
stl(1).des_config = [0;1;0];
stl(1).tolr_err = 0.01;

stl(2).type = 'G';
stl(2).time_range = [10 20];
stl(2).des_config = [1;1.2;-pi/4];
stl(2).tolr_err = 1;

%% solving diff equation
tspan = [0 20];
y0 = [x_obj_i;v_obj_i];
y_des = [x_des;v_des];

%% Controller
if (pid == 1)
   [t,y] = ode45(@(t,y) ode_pd_control(t,y,y_des), tspan, y0);
else
   [t,y] = ode23tb(@(t,y) ode_stl_ppc_control(t,y,stl,y0), tspan, y0);
end

%% Plots
err_plot = 1;
if (err_plot == 1)
   x_axis = y(:,1)-y(:,1);
   a = y(:,1) - stl(1).des_config(1);
   b = y(:,2) - stl(1).des_config(2);
   c = y(:,3) - stl(1).des_config(3);
   a2 = y(:,1) - stl(2).des_config(1);
   b2 = y(:,2) - stl(2).des_config(2);
   c2 = y(:,3) - stl(2).des_config(3);
   %plot(t,x_axis,'-',t,a,'-o',t,b,'-.',t,c,'-x')
   plot(t,x_axis,'-',t,a2,'-o',t,b2,'-.',t,c2,'-x')
   pause(5);

   sz_stl_array = size(stl);
   sz_stl = sz_stl_array(2);

   for i=1:size(t)
      [gamma_h,~] = gamma_h_calc(stl,t(i),x_obj_i);
      [h,~] = stl2ppc(stl,t(i),[y(i,1);y(i,2);y(i,3)]);
      gamma_h_vec(:,i) = diag(gamma_h);
      h_vec(:,i) = h;
      if t(i)<=10
         t1(i) = t(i);
         h_plot1(i) = h_vec(1,i);
         gamma_h_plot1(i) = gamma_h_vec(1,i);
      elseif t(i) > 10
        t2(i-size(t1)) = t(i);
        h_plot2(i-size(t1)) = h_vec(2,i);
        gamma_h_plot2(i-size(t1)) = gamma_h_vec(2,i);
      end
   end
   gamma_h_plot_neg1 = -1*gamma_h_plot1;
   gamma_h_plot_neg2 = -1*gamma_h_plot2;

plot(t1,gamma_h_plot1,'r',t1,gamma_h_plot_neg1,'r',t1,h_plot1,'b');hold on
plot(t2,gamma_h_plot2,'r',t2,gamma_h_plot_neg2,'r',t2,h_plot2,'b');
pause(5);

% Plotting the manipulators
x_obj = y(:,1);
y_obj = y(:,2);
yaw_obj = y(:,3);
v_x_obj = y(:,4);
v_y_obj = y(:,5);
w_obj = y(:,6);
d_o = 0.5;
l1 = 1.5;
l2 = 1.5;
l3 = 0.1;
branch = 0;
for i=1:size(t)
     %[J_o1,J_o2,dJ_o1,dJ_o2] = coup_dyn([  x_obj(i);  y_obj(i);yaw_obj(i)],...
     %                                   [v_x_obj(i);v_y_obj(i);  w_obj(i)],...
     %                                   d_o);

     x_ee_1_grd = [x_obj(i);y_obj(i);yaw_obj(i)] - [(d_o/2)*cos(yaw_obj(i));(d_o/2)*sin(yaw_obj(i));0];
     x_ee_2_grd = [x_obj(i);y_obj(i);pi+yaw_obj(i)] + [(d_o/2)*cos(yaw_obj(i));(d_o/2)*sin(yaw_obj(i));0];

     x_ee_1 = x_ee_1_grd + [1;0;0];
     x_ee_2 = x_ee_2_grd - [1;0;0];

     [q1_1, q1_2, q1_3] = inverseKinematics3R(x_ee_1(1), x_ee_1(2), x_ee_1(3),...
                                        l1,l2,l3,branch);
     [q2_1, q2_2, q2_3] = inverseKinematics3R(x_ee_2(1), x_ee_2(2), x_ee_2(3),...
                                        l1,l2,l3,1);
     M1_j0(i,:) = [-1 0];
     M1_j1(i,:) = M1_j0(i,:) + [l1*cos(q1_1) l1*sin(q1_1)];
     M1_j2(i,:) = M1_j1(i,:) + [l2*cos(q1_1 + q1_2) l2*sin(q1_1 + q1_2)];
     M1_j3(i,:) = M1_j2(i,:) + [l3*cos(q1_1 + q1_2 + q1_3) l3*sin(q1_1 + q1_2 + q1_3)];

     M2_j0(i,:) = [1 0];
     M2_j1(i,:) = M2_j0(i,:) + [l1*cos(q2_1) l1*sin(q2_1)];
     M2_j2(i,:) = M2_j1(i,:) + [l2*cos(q2_1 + q2_2) l2*sin(q2_1 + q2_2)];
     M2_j3(i,:) = M2_j2(i,:) + [l3*cos(q2_1 + q2_2 + q2_3) l3*sin(q2_1 + q2_2 + q2_3)];
end
end

animate = 1;
if (animate == 1)
    % Define the x and y positions of the square's center
    x_p = transpose(y(:,1)); % 100 points from 0 to 10
    y_p = transpose(y(:,2)); % 100 points from 0 to 5

    % Define the orientation of the square wrt x-axis
    theta = transpose(y(:,3)); % 100 points from 0 to pi

    % Define the length of the side of the square
    a = 0.5;

    % Create a figure
    figure;

    % Set the x and y limits of the plot
    xlim([-2, 6]);
    ylim([-2, 3.5]);

    % Loop through the positions and orientations and plot the square at each position
    i = 1;
    while t(i) < 20
        % Define the four vertices of the square in the local coordinate frame
        square_x = [-a/2 a/2 a/2 -a/2 -a/2];
        square_y = [-a/2 -a/2 a/2 a/2 -a/2];
    
        % Define the rotation matrix for the square
        R = [cos(theta(i)) -sin(theta(i)); sin(theta(i)) cos(theta(i))];
        R1 = [cos(stl(1).des_config(3)) -sin(stl(1).des_config(3)); sin(stl(1).des_config(3)) cos(stl(1).des_config(3))];
        R2 = [cos(stl(2).des_config(3)) -sin(stl(2).des_config(3)); sin(stl(2).des_config(3)) cos(stl(2).des_config(3))];
    
        % Transform the square vertices to the global coordinate frame
        square_xy = R * [square_x; square_y] + [x_p(i); y_p(i)];        
        square_xy_des1 = R1 * [square_x; square_y] + [stl(1).des_config(1);stl(1).des_config(2)];
        square_xy_des2 = R2 * [square_x; square_y] + [stl(2).des_config(1);stl(2).des_config(2)];
        
        square_x = square_xy(1,:);
        square_y = square_xy(2,:);
        square_x_des1 = square_xy_des1(1,:);
        square_y_des1 = square_xy_des1(2,:);
        square_x_des2 = square_xy_des2(1,:);
        square_y_des2 = square_xy_des2(2,:);
        % Manipulator arm
        M1_x  = [M1_j0(i,1) M1_j1(i,1) M1_j2(i,1) M1_j3(i,1)]; 
        M2_x =  [M2_j3(i,1) M2_j2(i,1) M2_j1(i,1) M2_j0(i,1)]; 
        M1_y = [M1_j0(i,2) M1_j1(i,2) M1_j2(i,2) M1_j3(i,2)];
        M2_y = [M2_j3(i,2) M2_j2(i,2) M2_j1(i,2) M2_j0(i,2)];

        % Plot the square
        plot(square_x, square_y, 'b', 'LineWidth', 2); hold on
        plot(square_x_des1, square_y_des1, 'r', 'LineWidth', 1);
        plot(square_x_des2, square_y_des2, 'r', 'LineWidth', 1);
        plot(M1_x, M1_y,'r','LineWidth', 3); 
        plot(M2_x, M2_y,'r', 'LineWidth', 3);
        plot(M1_x, M1_y,'o','MarkerSize', 7, 'MarkerFaceColor', 'r'); 
        plot(M2_x, M2_y, 'o','MarkerSize', 7, 'MarkerFaceColor', 'r');hold off
        %plot(); % plot the square in blue
        text(0, 2.5, ['time = ' num2str(t(i))]); % add a text annotation to the plot
        xlabel('x');
        ylabel('y');
        title('Animating a Square Moving and Rotating');
        xlim([-4, 4]);
        ylim([-2, 3.5]);
        drawnow; % update the plot
        pause(0.05); % pause for 50 milliseconds
        disp("code is running; iter");
        disp(i)
        if (t(i) > 9.5) && (t(i) < 9.8)
            i = i + 25;
        elseif (t(i) >= 9.8) && (t(i) < 10.14)
            i = i + 500;
        elseif (t(i) >= 10.14) && (t(i) < 10.18)
            i = i + 100;
        elseif (t(i) >= 10.18) && (t(i) < 10.40)
            i = i + 25;
        else
            i=i+1;
        end
    end   
end
