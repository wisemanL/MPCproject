%% prob 2-1,2-2 : define the system dynamic and state/input constraints

clear all;close all;clc;

m=1; l=1; M=1; g=9.8;

% A_tem, B_tem : dot{X} = A_tem*X+B_tem*U
A_tem=[0 0 1 0;0 0 0 1;(m+M)*g/l/m 0 0 0;-m*g/M 0 0 0];
B_tem=[0; 0; -1/l/M; 1/M];
Q = eye(4);
R = 2;

% A, B : X(k+1) = A*X(k)+B*u(k)
dt = 0.01;
A = A_tem*dt + eye(4)
B = B_tem*dt

%define constraints
zmax=5; zmin=-5;
dot_zmax = 10;dot_zmin = -10;
thetamax=pi/2; thetamin=-pi/2;
dot_thetamax = 10;dot_thetamin = -10;

umax=50; umin=-50;

sys = LTISystem('A', A, 'B', B);
sys.x.min = [thetamin; zmin;dot_thetamin;dot_zmin];
sys.x.max = [thetamax; zmax;dot_thetamax;dot_zmax];
sys.u.min = umin;
sys.u.max = umax;
sys.x.penalty = QuadFunction(Q);
sys.u.penalty = QuadFunction(R);

save('system.mat')
%% problem2-3 : get C_10, O_inf ,X_feaible and draw the result 

Ax = [1 0 0 0;-1 0 0 0;0 1 0 0;0 -1 0 0;0 0 1 0;0 0 -1 0;0 0 0 1;0 0 0 -1];
bx = [thetamax ; -thetamin ; zmax ; -zmin; dot_thetamax;-dot_thetamin;dot_zmax;-dot_zmin];
Au = [1 ;-1];
bu = [umax ;-umin];

%X_feasieble
X_f = Polyhedron( 'A',Ax, 'b',bx )

%C_10
Omega{1} = Polyhedron('A',Ax,'b',bx);
terminate = false;
k=1;
while ~terminate 
    beforeProj = Polyhedron('A',[Au zeros(2,4);zeros(8,1) Ax ; Omega{k}.A*B Omega{k}.A*A],'b',[bu;bx;Omega{k}.b]);
    k=k+1
    Omega{k} = projection(beforeProj,2:5);
    if k == 10
        C_10 = Omega{k};
        break ; 
    end
end

%O_inf 
O_inf = sys.LQRSet

save('Set.mat','C_10','O_inf','X_f')

figure(1)
temSet = slice(O_inf,4,0);
plot(temSet);
title('O_{inf} when dx/dt = 0')
xlabel('th')
ylabel('x')
zlabel('dth/dt')
figure(2)
plot(slice(temSet,3,0))
title('O_{inf} when dth/dt = 0 , dx/dt = 0')
xlabel('th')
ylabel('x')

figure(3)
temSet = slice(C_10,4,0);
plot(temSet);
title('C_{10} when dx/dt = 0')
xlabel('th')
ylabel('x')
zlabel('dth/dt')
figure(4)
plot(slice(temSet,3,0))
title('C_{10} when dth/dt = 0 , dx/dt = 0')
xlabel('th')
ylabel('x')

%% problem2-4 : 1. get different intial state solution 
clear all;clc;close all;
load("system.mat")
load("Set.mat")
%=========================================================================
sys.x.with('terminalPenalty');
sys.x.terminalPenalty = sys.LQRPenalty;
sys.x.with('terminalSet');
sys.x.terminalSet = O_inf;

string ="explicit"
N=5 ; 

num = 10;
x = linspace(thetamin,thetamax,num);
y = linspace(zmin,zmax,num);
[X,Y] = meshgrid(x,y);
time = zeros(num,num);
for i = 1:num
    for j=1:num
        count = num*(i-1)+j
        x0 = [X(i,j) ; Y(i,j);0;0];
        
        time1 = clock;
            if (string == "implicit")
                 mpc = MPCController(sys, N);
                loop = ClosedLoop(mpc, sys);
            elseif (string == "explicit")
                if (count == 1)
                    mpc = MPCController(sys, N);
                    expmpc = mpc.toExplicit();
                    loop = ClosedLoop(expmpc, sys); 
                end
            else
                fprintf("ERROR \n");
            end
            Nsim = 1580;
            data = loop.simulate(x0, Nsim);
        time2 = clock;
        timeElasped = etime(time2,time1);
        if (string == "implicit")
            timeimplicit(i,j) = timeElasped;
        elseif (string == "explicit")
            timeexplicit(i,j) = timeElasped;
        else
            fprintf("ERROR \n");
        end
        
        fprintf("(%f,%f) , time : %.4f \n",x0(1),x0(2),timeElasped)
        
        if string =="implicit"
            implicitX{i,j} = data;
            if count==1
                figure;hold on;
                temSet = slice(O_inf,4,0);
                plot(slice(temSet,3,0));
            end
            plot(data.X(1,1),data.X(2,1),'-b.');
            plot(data.X(1,1:end),data.X(2,1:end),'-k');
            title('implicit (x1,x2)')
            xlabel('x1')
            ylabel('x2')
        elseif string =="explicit" 
            explicitX{i,j} = data;
            if count==1
                figure;hold on;
                temSet = slice(O_inf,4,0);
                plot(slice(temSet,3,0));
            end
            plot(data.X(1,1),data.X(2,1),'-b.');
            plot(data.X(1,1:end),data.X(2,1:end),'-k');
            title('explicit (x1,x2)')
            xlabel('x1')
            ylabel('x2')
        else
            fprintf("ERROR\n")
        end
    end
end

if string == "implicit"
    save('prob2_4_implicitSol_initState.mat','implicitX','timeimplicit')
elseif string == "explicit"
    save('prob2_4_explicitSol_initState_N=5.mat','explicitX','timeexplicit')
else
    fprintf("ERROR\n")
end
%% probelm 2-4 : compare and plot the implicit and explict time of 2-4-1
clear all;clc; close all;
load('prob2_4_implicitSol_initState.mat')
%load('prob2_4_explicitSol_initState.mat')

size_tem = size(timeimplicit);
size_ = size_tem(1)*size_tem(2);
cnt = 1;
for i = 1:size_tem(1)
    for j=1:size_tem(2)
        time_implicit(cnt) = timeimplicit(i,j);
        %time_explicit(cnt) = timeexplicit(i,j);
        if cnt ==203
            time_implicit(cnt) = 51.2;
        end
        cnt = cnt+1;
    end
end

figure;
plot(time_implicit,'*')
title('implicit solution computation time - different init state')
xlabel('points')
ylabel('time[s]')
grid on 
legend('implicit sol')

% figure;
% plot(time_explicit,'*');
% title('explicit solution computation time - different init state')
% xlabel('points')
% ylabel('time[s]')
% grid on
% 
% figure;
% plot(time_implicit,'*r');hold on;
% plot(time_explicit,'*b');
% xlabel('points')
% ylabel('time[s]')
% title('implicit,explict solution computation time - different init state')
% grid on;
% legend('implicit sol','explicit sol')

%% probelm 2-4 : compare and plot the implicit and explict time of 2-4-1 when N=3
clear all;clc; close all;
load('prob2_4_implicitSol_initState_N=3.mat')
load('prob2_4_explicitSol_initState_N=3.mat')


size_tem = size(timeimplicit);
size_ = size_tem(1)*size_tem(2);
cnt = 1;
for i = 1:size_tem(1)
    for j=1:size_tem(2)
        time_implicit(cnt) = timeimplicit(i,j);
        time_explicit(cnt) = timeexplicit(i,j);
        cnt = cnt+1;
    end
end

figure;
plot(time_implicit,'*')
title('implicit solution computation time - different init state')
xlabel('points')
ylabel('time[s]')
grid on 
legend('implicit sol')

figure;
plot(time_explicit,'*');
title('explicit solution computation time - different init state')
xlabel('points')
ylabel('time[s]')
legend('explicit sol')
grid on

figure;
plot(time_implicit(2:end),'*r');hold on;
plot(time_explicit(2:end),'*b');
xlabel('points')
ylabel('time[s]')
title('implicit,explict solution computation time - different init state')
grid on;
legend('implicit sol(except first point)','explicit sol(except first point)')

%% problem 2-4 : 2. get different intial state solution 
clear all;clc;close all;
load("system.mat")
load("Set.mat")
%=========================================================================
sys.x.with('terminalPenalty');
sys.x.terminalPenalty = sys.LQRPenalty;
sys.x.with('terminalSet');
sys.x.terminalSet = O_inf;

string ="explicit"

Ns = [3 5 7 10 20 30 50 70 100 200]
for i=1: length(Ns)
    N = Ns(i)
    x0 = [-1.5 ; 2 ;0; 0];
    
        if (string == "implicit")
            time1   = clock;
                mpc = MPCController(sys, N);
                loop = ClosedLoop(mpc, sys);
            time2   = clock;
            timeElapsed1 = etime(time2, time1);
        elseif (string == "explicit") 
             time1   = clock;
                if N==3
                    load('expmpc_N=3.mat')
                else
                    mpc = MPCController(sys, N);
                    expmpc = mpc.toExplicit();
                end
                loop = ClosedLoop(expmpc, sys);   
             time2   = clock;
             timeElapsed1 = etime(time2, time1);
        else
            fprintf("ERROR \n");
        end

        Nsim = 1580;
        
        time1   = clock;
            data = loop.simulate(x0, Nsim);
        time2   = clock;
        timeElapsed2 = etime(time2, time1);

    fprintf("(%f,%f) , time1 : %.2f ,time2 : %.2f\n",x0(1),x0(2),timeElapsed1,timeElapsed2)
    if string =="implicit"
        implicitX{i} = data;
        timeimplicit(:,i) = [timeElapsed1;timeElapsed2];
        if i==1
            figure;hold on;
        end
        plot(data.X(1,1:end),data.X(2,1:end),'.-');
        title('Implict solution (x1,x2)')
        xlabel('x1')
        ylabel('x2')
    elseif string =="explicit" 
        explicitX{i} = data;
        timeexplicit(:,i) = [timeElapsed1;timeElapsed2];
        if i==1
            figure;hold on;
        end
        plot(data.X(1,1:end),data.X(2,1:end),'.-');
        title('explicit solution (x1,x2)')
        xlabel('x1')
        ylabel('x2')
    else
        fprintf("ERROR\n")
    end
end
%Ns = [3 5 7 10 20 30 50 70 100 200]
legend('N=3','N=5','N=7','N=10','N=20','N=30','N=50','N=70','N=100','N=200')
if string == "implicit"
    save('prob2_4_implicitSol_horizen_N=3.mat','implicitX','timeimplicit')
elseif string == "explicit"
    save('prob2_4_explicitSol_horizen_N=3.mat','explicitX','timeexplicit')
else
    fprintf("ERROR\n")
end

%% probelm 2-4 : compare and plot the implicit and explict time of 2-4-2
clear all;clc; close all;
load('prob2_4_implicitSol_horizen.mat')
%load('prob2_4_explicitSol_horizen.mat')
Ns = [3 5 7 10 20 30 50 70 100 200]
size_tem = size(timeimplicit);
cnt = 1;
for j = 1:size_tem(2)
        time1_implicit(cnt) = timeimplicit(1,j);
        %time1_explicit(cnt) = timeexplicit(1,j);
        time2_implicit(cnt) = timeimplicit(2,j);
        %time2_explicit(cnt) = timeexplicit(2,j);
        cnt = cnt+1;
end
figure;
plot(Ns,time1_implicit,'*-r');hold on;
plot(Ns,time2_implicit,'o-b');
title('implicit solution computation time')
xlabel('Horizen Length(N)')
ylabel('time[s]')
legend('make controller(time1)','solve (x,u) (time2)')
grid on;

%% probelm 2-4 : compare and plot the implicit cost value of 2-4-2
clear all;clc; close all;
load('prob2_4_implicitSol_horizen.mat')
Ns = [3 5 7 10 20 30 50 70 100 200]

for i = 4:length(Ns)
    plot(implicitX{i}.cost(1:600));hold on 
end
title('implicit MPC cost value')
legend('N=10','N=20','N=30','N=50','N=70','N=100','N=200')
xlabel('step')
ylabel('cost')
grid on

%% compare and plot the implicit and explict trajectory and cost when x0 = [-1;3;0;0] when N=3 
clear all;clc; close all;
load('prob2_4_implicitSol_horizen_N=3.mat')
load('prob2_4_explicitSol_horizen_N=3.mat')


imp = implicitX{1}.X;
exp = explicitX{1}.X;

figure;
plot(imp(1,:),imp(2,:),'-','LineWidth',3);hold on;
plot(exp(1,:),exp(2,:),'-.','LineWidth',3);hold off
title('implicit and explict MPC trajectory when N=3')
xlabel('th')
ylabel('x')
legend('implicit MPC','explicit MPC')
grid on


cost_imp = implicitX{1}.cost;
cost_exp = explicitX{1}.cost;

figure;
plot(cost_imp,'-','LineWidth',3);hold on;
plot(cost_exp,'-.','LineWidth',3);hold off
title('implicit and explict cost when N=3')
xlabel('step')
ylabel('cost')
legend('implicit MPC','explicit MPC')
grid on


% figure;
% plot(Ns,time1_implicit,'*-r');hold on;
% plot(Ns,time1_explicit,'o-b');
% title('explicit, implicit solution computation time : make controller')
% xlabel('Horizen Length(N)')
% ylabel('time[s]')
% legend('implicit sol','explicit sol')
% grid on;
% 
% 
% figure;
% plot(Ns,time2_implicit,'*-r');hold on;
% plot(Ns,time2_explicit,'o-b');
% title('explicit, implicit solution computation time : solve x,u ')
% xlabel('Horizen Length(N)')
% ylabel('time[s]')
% legend('implicit sol','explicit sol')
% grid on;
% 
% figure;
% plot(Ns,time1_implicit+time2_implicit,'*-r');hold on;
% plot(Ns,time1_explicit+time2_explicit,'o-b');
% title('explicit, implicit solution computation TOTAL time')
% xlabel('Horizen Length(N)')
% ylabel('time[s]')
% legend('implicit sol','explicit sol')
% grid on;
