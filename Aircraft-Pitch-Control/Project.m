clear;
clc;
close all;

A = [-0.313 56.7 0;-0.0139 -0.426 0;0 56.7 0];
B=[0.232;0.0203;0];
C=[1 0 1];
D=[0];               
%% Task 1- Checking Stability of System

%Method 1: poles of tf
[n,d]=ss2tf(A,B,C,D);
poles_of_tf=roots(d);
disp('The poles of transfer function are');
poles_of_tf


%Method 2: eigen values
eigen_values=eig(A);
disp('The eigen values of matrix A are');
eigen_values

%Method 3 - step response
t = 0:0.01:10;
sys=ss(A,B,C,D)
figure;
step(sys,t);
title('Step response of the system');
xlabel('Time');
ylabel('Amplitude');


%Method 4 - root locus
sys_tf = tf(ss(A,B,C,D));
figure;
rlocus(sys_tf)
grid on
title('Root Locus of Open-Loop System')

%Method 5 Pole-Zero Map
figure;
pzmap(sys);
title('Pole-Zero Map');
xlabel('Real Axis');
ylabel('Imaginary Axis');
grid on;              

% Method 5: Characteristic Equation for Routh Table
char_poly = poly(A)

%% part c: Checking Controllability & Observability

P = ctrb(A,B);
rank_of_ctrb_matrix=rank(P);
disp('The rank of controllability matrix is');
rank_of_ctrb_matrix

order_of_system=size(A,1);
disp('The order of the system is');
order_of_system

if(rank_of_ctrb_matrix==order_of_system)
    display('Controllability Test Pass');
else
    display('Controllability Test Fail');
end

%Observability Test 
Q = obsv(A, C);
rank_Ob = rank(Q);
disp('Rank of Observability Matrix:');
rank_Ob

order_of_system=size(A,1);
disp('The order of the system is');
order_of_system

if(rank_Ob==order_of_system)
    display('Observability Test Pass');
else
    display('Observability Test Fail');
end

% Controller Design
desired_egnvalues = [-4 -2 -14];
K = place(A,B,desired_egnvalues);
disp('Matrix K:');
disp(K);
% Observer
observer_egnvalues = [-20 -10 -70];
L = place(A',C',observer_egnvalues)';    
disp('Matrix L:');
disp(L);

%% Closed-loop system
A_cl = A - B*K;
sys_cl = ss(A_cl, B, C, D);
% Step Response of Closed Loop system
t = 0:0.01:10;
figure;
step(sys_cl, t);
title('Step response of the Close Loop system');
xlabel('Time');
ylabel('Amplitude');

%%Steady State Errors

% Steady State Error- Open loop system
sys = ss(A,B,C,D);
if isstable(sys)
    y_ol = dcgain(sys);
    e_ol = abs(1 - y_ol);
else
    y_ol = NaN;
    e_ol = Inf;
end
disp('Open-loop steady-state error:')
disp(e_ol)
%Steady State Error- Close loop system
if isstable(sys_cl)
    y_cl = dcgain(sys_cl);
    e_cl = abs(1 - y_cl);
else
    y_cl = NaN;
    e_cl = Inf;
end
disp('Closed-loop steady-state error:')
disp(e_cl)
% Augmented System
A_obs = [A - B*K, B*K; zeros(size(A)), A - L*C];
B_obs = [B; zeros(size(B))];
C_obs = [C, zeros(size(C))];
D_obs = D;

augmented_sys = ss(A_obs, B_obs, C_obs, D_obs);
figure;
step(augmented_sys, t);
title('Augmented System Step Response (Controller + Observer)');
xlabel('Time (s)');
ylabel('Amplitude');

%% Task 5 - Steady-State Error Analysis
open_loop_ss_error = abs(1 - dcgain(open_loop_sys));
disp('Steady-state error (Open-loop):');
disp(open_loop_ss_error);

closed_loop_ss_error = abs(1 - dcgain(closed_loop_sys));
disp('Steady-state error (Closed-loop):');
disp(closed_loop_ss_error);

%% Task 6 - Add a PID Controller

pidTuner(sys_tf, 'PID');

Kp = 20;   
Ki = 1;   
Kd = 20; 
pid_controller = pid(Kp, Ki, Kd);

closed_loop_with_pid = feedback(pid_controller * sys_tf, 1);

% Step Response for PID Controlled System
figure;
step(closed_loop_with_pid, t);
title('Closed-Loop Step Response with PID Controller');
xlabel('Time (s)');
ylabel('Amplitude');

% Steady-State Error for PID Controlled System
pid_ss_error = abs(1 - dcgain(closed_loop_with_pid));
disp('Steady-state error (PID-controlled):');
disp(pid_ss_error);

%% Verification and Comparison
fprintf('\n========== SYSTEM COMPARISON ==========\n');
fprintf('Open-loop SS Error:        Inf (unstable)\n');
fprintf('State Feedback SS Error:   %.4f\n', closed_loop_ss_error);
fprintf('PID Controller SS Error:   %.10f\n', pid_ss_error);

% Check PID system performance
pid_info = stepinfo(closed_loop_with_pid);
fprintf('\n========== PID PERFORMANCE ==========\n');
fprintf('Rise Time:      %.3f s\n', pid_info.RiseTime);
fprintf('Settling Time:  %.3f s\n', pid_info.SettlingTime);
fprintf('Overshoot:      %.2f %%\n', pid_info.Overshoot);
fprintf('Peak:           %.4f\n', pid_info.Peak);

% Plot comparison
figure;
subplot(2,1,1);
step(closed_loop_sys, t);
title('State Feedback Only (SS Error ≈ 0.9984)');
ylabel('Amplitude');
grid on;

subplot(2,1,2);
step(closed_loop_with_pid, t);
title('State Feedback + PID (SS Error = 0)');
xlabel('Time (s)');
ylabel('Amplitude');
grid on;