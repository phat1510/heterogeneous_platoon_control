function [K_engine, CL, gamma] = engine_controller_synthesis(G_engine, We, Wu, Wd, Wn)
    % Name the plant input/output
    G_engine.u = 'u_car'; G_engine.y = 'y_car';
    
    %% Connect the blocks
    sum_e = sumblk('e=r-yn');
    sum_d = sumblk('u_car=d_out+u');
    sum_n = sumblk('yn=y_car+nf');
    
    %% Name the input/output of weighting functions
    
    % For tracking error (S)
    We.u = 'e';We.y='z1';
    
    % For control input (KS)
    Wu.u = 'u'; Wu.y = 'z2';
    
    % For input disturbance
    Wd.u = 'd'; Wd.y = 'd_out';
    
    % Noise filter
    Wn.u = 'n'; Wn.y = 'nf';
    
    % Build interconnected system
    T0 = connect(G_engine,We,Wu,Wd,Wn,sum_e,sum_d,sum_n, ...
        {'r','d','n','u'},{'z1','z2','e'});
    
    %% Controller
    nmeas=1; ncon=1;
    [K_inf,T,gam] = hinfsyn(T0,nmeas,ncon);
    
    K_engine = K_inf;
    
    CL=T; gamma = gam;
end