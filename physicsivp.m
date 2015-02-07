%kg,m,s,
airden = 1.29;  %kg m-3
csa = 0.08; %m2
airflow = [10 0 0];%m s-1  direction: [x y z] where x=east,y=north,z=up
aira = airflow(1);
airb = airflow(2);
airc = airflow(3);
cd = 1.05; %for Boeing 747
objden = 2700; %kg m-3
objvol = 0.012; %m3
objm = objden*objvol; %should be kg
grav = 10; %m s-2
param1 = [airden csa aira airb airc cd objden objvol objm grav];
options1 = odeset('Events',@eventzer0,'RelTol',1e-8);
f = @(t,y,par)[y(2);...
    -(1/2)*par(1)*par(2)*(y(2)-par(3))*abs(y(2)-par(3))*par(6)/par(9);...
    y(4);...
    -(1/2)*par(1)*par(2)*(y(4)-par(4))*abs(y(4)-par(4))*par(6)/par(9);...
    y(6);...
    (-(par(7)-par(1))*par(8)*par(10) - (1/2)*par(1)*par(2)*(y(6)-par(5))*abs(y(6)-par(5))*par(6))/par(9)];
[t1,y1,te1,ye1] = ode45(f,[0 200],[0 250 0 1 13000 0],options1,param1);

watden = 1000;  %kg m-3
%csa = 0.08; %m2
watflow = [1 2 0];%m s-1  direction: [x y z] where x=east,y=north,z=up
wata = watflow(1);
watb = watflow(2);
watc = watflow(3);
watvis = 1.308e-3; %Pa s
%objden = 2700; %kg m-3
%objvol = 0.012; %m3
%objm = objden*objvol; %should be kg
%grav = 10; %m s-2
watdep = 3000; %local depth of ocean, m 
param2 = [watden csa wata watb watc cd objden objvol objm grav watdep];%for quadratic v-dependence
%param2 = [watden csa wata watb watc watvis objden objvol objm grav
%watdep]; %for Stoke
options2 = odeset('Events',@eventbot,'RelTol',1e-8);
ic2 = ye1.*[1 objden/(objden+watden) 1 objden/(objden+watden) 1 objden/(objden+watden)];%modify initial speed upon hitting water
%g = @(t,y,par)[y(2);...
%    -6*pi*par(6)*par(2)*(y(2)-par(3))/par(9);...
%    y(4);...
%    -6*pi*par(6)*par(2)*(y(4)-par(4))/par(9);...
%    y(6);...
%    (-(par(7)-par(1))*par(8)*par(10) - 6*pi*par(6)*par(2)*(y(6)-par(5)))/par(9)];
g = @(t,y,par)[y(2);...
    -(1/2)*par(1)*par(2)*(y(2)-par(3))*abs(y(2)-par(3))*par(6)/par(9);...
    y(4);...
    -(1/2)*par(1)*par(2)*(y(4)-par(4))*abs(y(4)-par(4))*par(6)/par(9);...
    y(6);...
    (-(par(7)-par(1))*par(8)*par(10) - (1/2)*par(1)*par(2)*(y(6)-par(5))*abs(y(6)-par(5))*par(6))/par(9)];
[t2,y2,te2,ye2] = ode45(g,[te1 te1+2000],ic2,options2,param2);

