function dydt = ode_pd_control(t,y,y_des)
    x_o = [y(1); y(2); y(3)]; 
    v_o = [y(4); y(5); y(6)];
    
    x_des = [y_des(1); y_des(2); y_des(3)]; 
    v_des = [y_des(4); y_des(5); y_des(6)];
    
    err = x_des - x_o;
    err_rate = v_des - v_o;

    Kp1_vec = [50,20,20];
    Kp2_vec = [50,20,20];
    Kd1_vec = [25,7,7];
    Kd2_vec = [25,7,7];

    Kp1 = diag(Kp1_vec);
    Kp2 = diag(Kp2_vec);
    Kd1 = diag(Kd1_vec);
    Kd2 = diag(Kd2_vec);

    Kp = [Kp1;Kp2];
    Kd = [Kd1;Kd2];

    u = Kp*err + Kd*err_rate;
    [~,~,~,v_o,dv_o] = coop_manip_dyn(x_o,v_o,u);
    dydt = [v_o;dv_o];
    %dydt = transpose(dydt_T);
end