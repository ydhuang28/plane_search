inputs = [1.14593 0.000111287 0.0454121 0.000189623 -0.300000 -0.00126975].*1e4;

for ic_num = 1:length(inputs(:,1))
    watden = 1000;  %kg m-3
    %csa = 0.08; %m2
    watflow = [-1 -2 0];%m s-1  direction: [x y z] where x=east,y=north,z=up
    wata = watflow(1);
    watb = watflow(2);
    watc = watflow(3);
    watvis = 1.308e-3; %Pa s
    objden = 2700; %kg m-3
    objvol = 0.012; %m3
    objm = objden*objvol; %should be kg
    grav = 9.8; %m s-2
    watdep = 3000; %local depth of ocean, m
    param2 = [watden csa wata watb watc cd objden objvol objm grav watdep];%for quadratic v-dependence
    options2 = odeset('Events',@eventoutwater,'RelTol',1e-8);
    icb2 = inputs(ic_num,:).*[1 -1 1 -1 1 -1];
    % ic2 = ye1.*[1 objden/(objden+watden) 1 objden/(objden+watden) 1 objden/(objden+watden)];%modify initial speed upon hitting water
    gb = @(t,y,par)[y(2);...
        -(1/2)*par(1)*par(2)*(-y(2)-par(3))*abs(y(2)-par(3))*par(6)/par(9);...
        y(4);...
        -(1/2)*par(1)*par(2)*(-y(4)-par(4))*abs(y(4)-par(4))*par(6)/par(9);...
        y(6);...
        (-(par(7)-par(1))*par(8)*par(10) - (1/2)*par(1)*par(2)*(-y(6)-par(5))*abs(y(6)-par(5))*par(6))/par(9)];
    [tb2,yb2,teb2,yeb2] = ode45(gb,[0 2000],icb2,options2,param2);
    
    %kg,m,s,
    airden = 1.29;  %kg m-3
    csa = 0.08; %m2
    airflow = [-10 0 0];%m s-1  direction: [x y z] where x=east,y=north,z=up
    aira = airflow(1);
    airb = airflow(2);
    airc = airflow(3);
    cd = 0.031; %for Boeing 747, streamlined
    %cd = 0.47 % for sphere
    %cd = 1.05 % for cube
    objden = 2700; %kg m-3
    objvol = 0.012; %m3
    objm = objden*objvol; %should be kg
    grav = 9.8; %m s-2
    param1 = [airden csa aira airb airc cd objden objvol objm grav];
    options1 = odeset('Events',@eventbcksht,'RelTol',1e-8);
    
    % modify initial speed upon leaving water
    icb1 = yeb2.*[1 -objden/(objden+watden) 1 -objden/(objden+watden) 1 -objden/(objden+watden)];
    fb = @(t,y,par)[y(2);...
        -(1/2)*par(1)*par(2)*(-y(2)-par(3))*abs(y(2)-par(3))*par(6)/par(9);...
        y(4);...
        -(1/2)*par(1)*par(2)*(-y(4)-par(4))*abs(y(4)-par(4))*par(6)/par(9);...
        y(6);...
        (-(par(7)-par(1))*par(8)*par(10) - (1/2)*par(1)*par(2)*(-y(6)-par(5))*abs(y(6)-par(5))*par(6))/par(9)];
    [tb1,yb1,teb1,yeb1] = ode45(fb,[teb2 teb2+200],icb1,options1,param1);

end
