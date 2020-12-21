% problem 1-1,1-2

% define system(A,b) and constraints cost matrix(R,Q) and length(N)
A = [1.1 -0.2; -1 1];
eig(A)
B = [0.3 ;0.2];
Q = eye(2);
R = 0.05;


sys = LTISystem('A', A, 'B', B);
sys.x.min = [-1; -1];
sys.x.max = [1; 1];
sys.u.min = -0.7;
sys.u.max = 0.7;
sys.x.penalty = QuadFunction(Q);
sys.u.penalty = QuadFunction(R);

%% problem 1-3 

% O_inf 
O_inf = sys.LQRSet;
% C_inf
C_inf = sys.invariantSet();
% P_inf
[F_inf,P_inf] = dlqr(A,B,Q,R);
save("set.mat",'O_inf','C_inf')
%% problem 1-4 : 1. get different intial state solution 

sys.x.with('terminalPenalty');
sys.x.terminalPenalty = sys.LQRPenalty;
sys.x.with('terminalSet');
sys.x.terminalSet = O_inf;

string ="implicit"

N=10;

num = 15;
x = linspace(-1.0,1.0,num);
y = linspace(-1.0,1.0,num);
[X,Y] = meshgrid(x,y);
time = zeros(num,num);
for i = 1:num
    for j=1:num
        count = num*(i-1)+j
        x0 = [X(i,j) ; Y(i,j)];
        
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

            Nsim = 1000;
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
                plot(O_inf);
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
                plot(O_inf);
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
    save('prob1_4_implicitSol_initState.mat','implicitX','timeimplicit')
elseif string == "explicit"
    save('prob1_4_explicitSol_initState.mat','explicitX','timeexplicit')
else
    fprintf("ERROR\n")
end
%% plot implict and explict solution computation time of 1-4:1. get different intial state solution 
clear all;clc; close all;
load('prob1_4_implicitSol_initState.mat')
load('prob1_4_explicitSol_initState.mat')

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
plot(time_implicit,'*');
title('implicit solution computation time - different init state')
xlabel('points')
ylabel('time[s]')
grid on 

figure;
plot(time_explicit,'*');
title('explicit solution computation time - different init state')
xlabel('points')
ylabel('time[s]')
grid on

figure;
plot(time_implicit,'*r');hold on;
plot(time_explicit,'*b');
xlabel('points')
ylabel('time[s]')
title('implicit,explict solution computation time - different init state')
grid on;
legend('implicit sol','explicit sol')

%% problem 1-4 : 2. get different horizen solution for fixed initial condition 


clear all;clc;close all;
load('set.mat')
A = [1.1 -0.2; -1 1];
eig(A)
B = [0.3 ;0.2];
Q = eye(2);
R = 0.05;

sys = LTISystem('A', A, 'B', B);
sys.x.min = [-1; -1];
sys.x.max = [1; 1];
sys.u.min = -0.7;
sys.u.max = 0.7;
sys.x.penalty = QuadFunction(Q);
sys.u.penalty = QuadFunction(R);
sys.x.with('terminalPenalty');
sys.x.terminalPenalty = sys.LQRPenalty;
sys.x.with('terminalSet');
sys.x.terminalSet = O_inf;

string = "explicit"
Ns = [3 5 7 8 9 10 11 13 15 30 40]
for i=1: length(Ns)
    N = Ns(i)
    x0 = [-0.66 ;-0.91];
        if (string == "implicit")
            time1   = clock;
                mpc = MPCController(sys, N);
                loop = ClosedLoop(mpc, sys);
            time2   = clock;
            timeElapsed1 = etime(time2, time1);
        elseif (string == "explicit") 
             time1   = clock;
                mpc = MPCController(sys, N);
                expmpc = mpc.toExplicit();
                loop = ClosedLoop(expmpc, sys);   
             time2   = clock;
             timeElapsed1 = etime(time2, time1);
        else
            fprintf("ERROR \n");
        end

        Nsim = 1000;
        
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
%Ns = [3 5 7 8 9 10 11 13 15 30 40]
legend('N=3','N=5','N=7','N=8','N=9','N=10','N=11','N=13','N=15','N=30','N=40')
if string == "implicit"
    save('prob1_4_implicitSol_horizen.mat','implicitX','timeimplicit')
elseif string == "explicit"
    save('prob1_4_explicitSol_horizen.mat','explicitX','timeexplicit')
else
    fprintf("ERROR\n")
end

%% plot implicit and explicit solution cost value of 1-4 : 2. get different horizen solution for fixed initial condition
clear all;clc; close all;
load('prob1_4_implicitSol_horizen.mat')
load('prob1_4_explicitSol_horizen.mat')
Ns = [3 5 7 8 9 10 11 13 15 30 40]

figure(1)
for j = 1:length(Ns)
    a = implicitX{j}.cost;
    len = size(a);
    if len<20
        plot(a);hold on
    else
        plot(a(1:20));hold on
    end
end
title('implicit cost value')
xlabel('step')
ylabel('cost')
legend('N=3','N=5','N=7','N=8','N=9','N=10','N=11','N=13','N=15','N=30','N=40')



%% plot implict and explict solution computation time of 1-4 : 2. get different horizen solution for fixed initial condition
clear all;clc; close all;
load('prob1_4_implicitSol_horizen.mat')
load('prob1_4_explicitSol_horizen.mat')
Ns = [3 5 7 8 9 10 11 13 15 30 40]
size_tem = size(timeimplicit);
cnt = 1;
for j = 1:size_tem(2)
        time1_implicit(cnt) = timeimplicit(1,j);
        time1_explicit(cnt) = timeexplicit(1,j);
        time2_implicit(cnt) = timeimplicit(2,j);
        time2_explicit(cnt) = timeexplicit(2,j);
        cnt = cnt+1;
end

figure;
plot(Ns,time1_implicit,'*-r');hold on;
plot(Ns,time1_explicit,'o-b');
title('explicit, implicit solution computation time : make controller')
xlabel('Horizen Length(N)')
ylabel('time[s]')
legend('implicit sol','explicit sol')
grid on;


figure;
plot(Ns,time2_implicit,'*-r');hold on;
plot(Ns,time2_explicit,'o-b');
title('explicit, implicit solution computation time : solve x,u ')
xlabel('Horizen Length(N)')
ylabel('time[s]')
legend('implicit sol','explicit sol')
grid on;

figure;
plot(Ns,time1_implicit+time2_implicit,'*-r');hold on;
plot(Ns,time1_explicit+time2_explicit,'o-b');
title('explicit, implicit solution computation TOTAL time')
xlabel('Horizen Length(N)')
ylabel('time[s]')
legend('implicit sol','explicit sol')
grid on;