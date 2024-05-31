clear; close all; clc;
try
    fclose(instrfindall);
catch
end
rosshutdown;

%% Load Class
    % Load Classes
      % Create OptiTrack object and initialize
    OPT = OptiTrack;
    OPT.Initialize;
    
    % Joystick
    J = JoyControl;
        
    % 
    A = ArDrone();
    A.pPos.Z = zeros(12,1);
    A.pPar.T_loop = 1/50;
    A.pSC.Fref = zeros(3,1);
    idA = 1;
    
    dummy_var_pose = ArDrone();
    
    
    rX = 1;           % [m]
    rY = 1;           % [m]
    T = 10;             % [s]
    Tf = T*2;            % [s]
    w = 2*pi/T;         % [rad/s]
    w = sqrt(3);
    

    
%% Network
rosinit('192.168.0.100');
% rosinit('10.10.10.40');
% rosinit('http://10.10.10.40:11311','NodeHost','10.10.10.125')
pub = rospublisher('/cf1/cmd_vel','geometry_msgs/Twist');
pub_motors = rospublisher('/cf1/cmd_vel_motors','geometry_msgs/Twist');
subVel = rossubscriber('/cf1/crazyflieVel','geometry_msgs/Vector3');
pause(.5)
A.pPos.X(7:9) = [subVel.LatestMessage.X;subVel.LatestMessage.Y;subVel.LatestMessage.Z];


% pub = rospublisher('/L1/cmd_vel','geometry_msgs/Twist');
% pub = rospublisher('RosAria/cmd_vel','geometry_msgs/Twist');
msg = rosmessage(pub);
msg_motors = rosmessage(pub_motors);
cmd = zeros(1,4);
cmd_old = zeros(1,4);
    
% timers
t_incB = tic;
t  = tic;
t_control = tic;
T_exp = 480; % tempo de experimento
T_run = 1/50; % período de amostragem do experimento
t_run = tic;
t_total = tic; % tempo que o experimento está rodando (running)
t_plot = tic; % 
T_plot = .3; % período de plotagem
t_task = tic;
T_task = 13;
i_task = 0;
t_exp = tic;
t_gps = -1;
gps_freq = 1/5;


data = [];

% crazyflie 

tilt_max = 20;

psi_ref_max = 100;
g = 9.81;
a_max = 14.35;
conversao_digital = 60000/a_max;
theta_ref = 0;
phi_ref = 0;
psi_ref = 0;
thrust = 11000;
%%%%%%%%%%%%%%%%%%%%%% Botão de Emergencia %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nLandMsg = 3;
btnEmergencia = 0;
ButtonHandle = uicontrol('Style', 'PushButton', ...
    'String', 'land', ...
    'Callback', 'btnEmergencia=1', ...
    'Position', [50 50 400 300]);    

A.pPar.ti = tic;
A.pPar.counter = 0;
A.pPos.Xb(7:8) = zeros(2,1);
A.pPos.dXb(7:8) = zeros(2,1);

zt_delta = 0;

% manter pos inicial
% Obter os dados dos sensores
rb = OPT.RigidBody;
if rb(idA).isTracked
    A = getOptData(rb(idA),A);
    %             A.pPos.X
end

A.pPos.Xd(1:2) = A.pPos.X(1:2);
A.pPos.Xd(3) = .75;

pouso = 0;
centro_sala = 0;

[b,a] = butter(2,1/15);

way_i = 1;

t_estabilizacao = 7; % de preferencia maior que 3
T_interval = 15;
no_noise = true;

A.pPos.Xi = zeros(3,1);
A.pPos.dXi = zeros(3,1);

while toc(t) < (0*T_exp + 12000) 
        
        if toc(t_run) > T_run 
%             toc(t_run)
            
            t_run = tic;
            t_atual = toc(t);
            A.pPar.counter = A.pPar.counter + 1;
                
        
%% Trajetória
% Trajetória 2
            if t_atual > 10
                w = sqrt(2);
            else
                w = sqrt(0.5);
            end

%% Círculo simples
%             if t_atual > t_estabilizacao
%                 wz = 2*w;
%                 A.pPos.Xd([1:3 6]) = [rX*sin(w*t_atual);
%                     rY*cos(w*t_atual);
%                     1 - 1*0.25*sin(wz*t_atual);
%                     0];
% 
% 
%                 A.pPos.Xd([7:9 12]) = [w*rX*cos(w*t_atual);
%                     -w*rY*sin(w*t_atual);
%                     1*-0.25*wz*cos(wz*t_atual);
%                     0];
% 
%                 A.pPos.dXd([7:9 12]) = [w^2*rX*-sin(w*t_atual);
%                     -w^2*rY*cos(w*t_atual);
%                     1*-0.25*wz^2*-sin(wz*t_atual);
%                     0];
%                 
%                 
%                 A.pPos.Xd(6) = atan2(A.pPos.Xd(8),A.pPos.Xd(7));
%                 A.pPos.Xd(12) = w;
%                 
% %                 A.pPos.Xd(6) = 0;
% %                 A.pPos.Xd(12) = 0;
%             end
% 
            
%% Quadrado
%                 if t_atual > t_estabilizacao
%                     
%                     if ~exist('t_interval','var')
%                         t_interval = tic;
%                     end
%                     
%                     X = [-2 1.2 0.7;
%                          2 -1.2 1.3;
%                          -2 -1.2 0.7;
%                          2 1.2 1.3;
%                          -2 1.2 0.7]';
%                      
%                     X = [-0.8 0.71 0.7;
%                          1.13 -0.71 1.3;
%                          -0.8 -0.71 0.7;
%                          1.13 0.71 1.3;
%                          -0.8 0.71 0.7]';
%                      
% %                      X = [-1 1 1;
% %                          1 -1 1;
% %                          -1 -1.0 1;
% %                          1 1.0 1]';
% 
%                      
%                      A.pPos.Xd([1:3 6]) = [X(:,way_i);0];
%                      A.pPos.Xd([7:9 12]) = zeros(4,1);
%                      A.pPos.dXd([7:9 12]) = zeros(4,1);
%                      
%                      if toc(t_interval) > T_interval
%                          way_i = way_i + 1;
%                          if way_i > length(X)
%                              way_i = 5;
%                          end
%                          t_interval = tic;
%                          disp(X(:,way_i))                         
%                     end
%                      
%                 end
                
%% Oito Louco
             if t_atual > t_estabilizacao
                T = 5;
                w = 2*pi/T;
                r = 0.75;

                flag_traj = double(rem(floor(t_atual/T),2));
                A.pPos.Xd(1:3) = [r*cos((1-2*flag_traj)*w*(t_atual - flag_traj*T/2));r*sin((1-2*flag_traj)*w*(t_atual - flag_traj*T/2));1] + [-r + 2*r*flag_traj;0;0];
                A.pPos.Xd(7:9) = [r*(1-2*flag_traj)*w*-sin((1-2*flag_traj)*w*(t_atual - flag_traj*T/2));r*(1-2*flag_traj)*w*cos((1-2*flag_traj)*w*(t_atual - flag_traj*T/2));0];                              
                                
                A.pPos.Xd(6) = 0;
                A.pPos.Xd(12) = 0;
             end
                
%% Centro sala                

                if centro_sala == 1 || t_atual <= t_estabilizacao
                    A.pPos.Xd([1:3 6]) = [0;0;1;0];
                    A.pPos.Xd([7:9 12]) = [0;0;0;0];
                    A.pPos.dXd([7:9 12]) = [0;0;0;0];
                end
                
        %% Obter os dados dos sensores
        rb = OPT.RigidBody;
            if rb(idA).isTracked
            dummy_var_pose = getOptDataNoise(rb(idA),dummy_var_pose);
            
            if t_gps == -1
                A = getOptDataNoise(rb(idA),A);
                t_gps = tic;
            end
            
            if toc(t_gps) >= gps_freq
                A = getOptDataNoise(rb(idA),A);
                t_gps = tic;
            end
            
            A.pPos.X(7:9) = [subVel.LatestMessage.X;subVel.LatestMessage.Y;subVel.LatestMessage.Z];    
            A = cLuenbergerEstimator(A);
%             A.pPos.X            
            end
            
        % Para modelo        
        thrust_feito = (A.pPos.dX(9)+g)/(cos(A.pPos.X(4))*cos(A.pPos.X(5)));
        A.pSC.thrust_feito = thrust_feito;
        %%
   
        %% Automático
        % Controlador 
        Pxi = diag([0.1 0.1 0.1])*3/10;
        Pdxi = diag([0.1 0.1 0.1])*3/10;
        
        A.pPos.Xi = A.pPos.Xi + (A.pPos.Xd(1:3) - A.pPos.X_sujo)*A.pPar.T_loop;
        A.pPos.dXi = A.pPos.dXi + (A.pPos.Xd(7:9) - A.pPos.X(7:9))*A.pPar.T_loop;
        
        Rwb = [cos(A.pPos.X(6)) sin(A.pPos.X(6)); -sin(A.pPos.X(6)) cos(A.pPos.X(6))];
        
        Int = (Pxi*A.pPos.Xi + Pdxi*A.pPos.dXi)*1;
        
        if  t_atual < (t_estabilizacao -1) | no_noise
            xref =  Rwb*(A.pPos.dXd(7:8) + 6/2*(A.pPos.Xd(7:8) - dummy_var_pose.pPos.X(7:8)) + 8/2*(A.pPos.Xd(1:2) - dummy_var_pose.pPos.X(1:2)) + Int(1:2)); % No noise
            zref = A.pPos.dXd(9) + 2*(A.pPos.Xd(9) - dummy_var_pose.pPos.X(9)) + 4*(A.pPos.Xd(3) - dummy_var_pose.pPos.X(3) + Int(3));
        else
            xref =  Rwb*(A.pPos.dXd(7:8) + 6/2*(A.pPos.Xd(7:8) - A.pPos.X(7:8)) + 8/2*(A.pPos.Xd(1:2) - A.pPos.X_sujo(1:2)) + Int(1:2)); % With noise
            zref = A.pPos.dXd(9) + 2*(A.pPos.Xd(9) - A.pPos.X(9)) + 4*(A.pPos.Xd(3) - A.pPos.X_sujo(3) + Int(3));
        end
        
        vel_ref = [(A.pPos.Xd(7:8) + 12/10*(A.pPos.Xd(1:2) - A.pPos.X(1:2))); (A.pPos.Xd(9) + 4/2*(A.pPos.Xd(3) - A.pPos.X(3)))];
        
        A.pPos.Z([2 5 8]) = vel_ref;
        
        if pouso == 1
            zref = 3*-.4;
        end
        
        A.pSC.Fref(1:2) = A.pPos.dXd(7:8) + 10*(A.pPos.Xd(7:8) - A.pPos.X(7:8)) + 12*(A.pPos.Xd(1:2) - A.pPos.X(1:2));
        A.pSC.Fref(3) = A.pPos.dXd(9) + 2*(A.pPos.Xd(9) - A.pPos.X(9)) + 4*(A.pPos.Xd(3) - A.pPos.X(3)) + 9.81;
        
        thrust = (zref + g)/(cos(A.pPos.X(5))*cos(A.pPos.X(4)))*conversao_digital;
        theta_ref = conversao_digital/thrust*xref(1);
        phi_ref = conversao_digital/thrust*xref(2);
             
        psi_til = A.pPos.Xd(6) - A.pPos.X(6);
        if abs(psi_til) > pi
            if psi_til > 0
                psi_til = psi_til - 2*pi;
            else
                psi_til = psi_til + 2*pi;
            end
        end
               
        psi_ref = A.pPos.Xd(12) + 3*psi_til;
       
        
        if t_atual > 50
            disp('land')
        end
                                 
%% Manual (joystick)
        mRead(J);

        if norm(J.pAnalog(4:5)) > .1
            theta_ref = -J.pAnalog(5);
            phi_ref = -J.pAnalog(4);
        end

        if abs(J.pAnalog(3)) > .1
            psi_ref = J.pAnalog(3);
            psi_ref = min(max(psi_ref,-1),1);
            psi_ref = psi_ref*psi_ref_max;
        end

        if J.pDigital(1) == 1
            btnEmergencia = 1;
        end
        
        if J.pDigital(2) == 1
            pouso = 1;
        end
        
        if J.pDigital(4) == 1
            centro_sala = 1;
        end
        
        if abs(J.pAnalog(2)) > .1
            thrust = (.5 + -J.pAnalog(2)*.5)*60000;
            thrust = min(max(thrust,30000),60000);
        end
        
        if t_atual > 3
            A.pPos.Xref = [xref;zref];
        else
            A.pPos.Xref = [0;0;zref];
        end
        
%         disp(A.pPos.Xref)

%% Motor control
%         A = acc2motorControl(A);

%% Envio dos sinais de controle
        cmd_raw = [[theta_ref phi_ref psi_ref]*180/pi thrust];

        theta_ref = min(max(cmd_raw(1),-tilt_max),tilt_max);
%         theta_ref = theta_ref*tilt_max;
        phi_ref = min(max(cmd_raw(2),-tilt_max),tilt_max);
%         phi_ref = phi_ref*tilt_max;
        psi_ref = min(max(cmd_raw(3),-psi_ref_max),psi_ref_max);
%         psi_ref = psi_ref*psi_ref_max;
        thrust = min(max(thrust,11000),60000);

        cmd = [theta_ref phi_ref psi_ref thrust];
        
        msg.Linear.X = cmd(1); msg.Linear.Y = cmd(2); msg.Linear.Z = cmd(4); msg.Angular.Z = cmd(3);

        disp([msg.Linear.X msg.Linear.Y msg.Angular.Z msg.Linear.Z/60000])


%         cmd_motors = [A.pSC.cmd_motors(1:4)' 1];
%         disp([cmd_motors(1:4)/65550 cmd_motors(5)])
%         msg_motors.Linear.X = cmd_motors(1); msg_motors.Linear.Y = cmd_motors(2); msg_motors.Linear.Z = cmd_motors(3); msg_motors.Angular.X = cmd_motors(4); msg_motors.Angular.Y = 1;
%         send(pub_motors,msg_motors);
%%

        %%%%% SEND %%%%%%%%%%%
        send(pub,msg)
       
        %% If push Emergency or ROS Emergency Stop On or Not Rigid Body tracked Stop loop
            drawnow
            if btnEmergencia ~= 0 
                 msg.Linear.X = 0;
                 msg.Linear.Y = 0;
                 msg.Angular.Z = 0;       
                 msg.Linear.Z = 0;
                 send(pub,msg) %NÃO COMENTAR
                 msg_motors.Linear.X = 0; msg_motors.Linear.Y = 0; msg_motors.Linear.Z = 0; msg_motors.Angular.X = 0; msg_motors.Angular.Y = 1;
                 send(pub_motors,msg_motors) %NÃO COMENTAR

                 
                disp('Bebop Landing through Emergency Command ');

                % Send 3 times Commands 1 second delay to Drone Land
                for i=1:nLandMsg
                    disp("End Land Command");
                    
                end
                break;
            end         

%         A.pPos.Xb(7:8) = Rwb*A.pPos.X(7:8);
%         A.pPos.dXb(7:8) = Rwb*A.pPos.dX(7:8);
        
        % Coleta de dados
        data = [  data  ; A.pPos.Xd' A.pPos.X' thrust' thrust_feito' A.pPos.dX' A.pPos.Z([2 5 8])' cmd cmd_raw A.pPos.X_sujo' A.pPos.Xi' A.pPos.dXi' dummy_var_pose.pPos.X' t_atual];
%                          1 -- 12    13 -- 24   25     26           27 -- 38     39 40 41        42-45 46-49     50 51 52    53 54 55    56 57 58         59 -- 70            71

        end
end


for i = 1:100
                 msg.Linear.X = 0;
                 msg.Linear.Y = 0;
                 msg.Angular.Z = 0;       
                 msg.Linear.Z = 0;
                 send(pub,msg)
                 msg.Linear.X = 0; msg.Linear.Y = 0; msg.Linear.Z = 0; msg.Angular.X = 0; msg.Angular.Y = 0;
                 send(pub_motors,msg_motors)
                 msg_motors.Linear.X = 0; msg_motors.Linear.Y = 0; msg_motors.Linear.Z = 0; msg_motors.Angular.X = 0; msg_motors.Angular.Y = 1;
                 send(pub_motors,msg_motors)
end

%% Plot results
drone = A;
Xtil = data(:,1:12) - data(:,13:24);
dXtil = data(:,39:41) - data(:,19:21);

figure();
hold on;
grid on;
plot(data(:,end),Xtil(:,1));
plot(data(:,end),Xtil(:,2));
plot(data(:,end),Xtil(:,3));
% plot(data(:,end),Xtil(:,6));
axis([0 70 -.2 .2])
title('Erro de Posição');
legend('Pos X','Pos Y','Pos Z', 'Ori Z');
xlabel('Tempo[s]');
ylabel('Erro [m]');
% %%
% figure();
% hold on;
% grid on;
% plot(data(:,end),filtfilt(b,a,dXtil(:,1)));
% plot(data(:,end),filtfilt(b,a,dXtil(:,2)));
% plot(data(:,end),filtfilt(b,a,dXtil(:,3)));
% % plot(data(:,end),Xtil(:,6));
% axis([0 70 -.2 .2])
% title('Erro de seguimento de velocidade');
% legend('Vel X','Vel Y','Vel Z', 'Ori Z');
% xlabel('Tempo[s]');
% ylabel('Erro [m]');

%%
figure();
subplot(311)
hold on;
grid on;
plot(data(:,end),data(:,1));
plot(data(:,end),data(:,13));

dim = [.2 .4 .3 .3];
str = ['RMSE: ', num2str(sqrt(mean((data(:,1) - data(:,13)).^2)))];
annotation('textbox', dim, 'String', str, 'FitBoxToText', 'on', 'BackgroundColor', 'white');

xlabel('Tempo[s]');
ylabel('x [m]');
legend('Xd', 'X');

subplot(312)
hold on;
grid on;
plot(data(:,end),data(:,2));
plot(data(:,end),data(:,14));

dim = [.2 .4 .3 .3];
str = ['RMSE: ', num2str(sqrt(mean((data(:,2) - data(:,14)).^2)))];
annotation('textbox', dim, 'String', str, 'FitBoxToText', 'on', 'BackgroundColor', 'white');

xlabel('Tempo[s]');
ylabel('y [m]');
legend('Yd', 'Y');

subplot(313)
hold on;
grid on;
plot(data(:,end),data(:,3));
plot(data(:,end),data(:,15));

dim = [.2 .4 .3 .3];
str = ['RMSE: ', num2str(sqrt(mean((data(:,3) - data(:,15)).^2)))];
annotation('textbox', dim, 'String', str, 'FitBoxToText', 'on', 'BackgroundColor', 'white');

xlabel('Tempo[s]');
ylabel('z [m]');
legend('Zd', 'Z');
%%
% subplot(414)
% hold on;
% grid on;
% plot(data(:,end),data(:,6));
% plot(data(:,end),data(:,18));
% xlabel('Tempo[s]');
% ylabel('psi [rad]');
% legend('phid', 'phi');

%% vel
figure();
subplot(311)
hold on;
grid on;
plot(data(:,end),data(:,7)); 
plot(data(:,end),data(:,19));
% plot(data(:,end),data(:,39));
% plot(data(:,end),filtfilt(b,a,data(:,19)));
xlabel('Tempo[s]');
ylabel('Velocidade [m/s]');
legend('$\dot{x}_{des}$', '$\dot{x}$','interpreter','latex');

subplot(312)
hold on;
grid on;
plot(data(:,end),data(:,8));
plot(data(:,end),data(:,20));
% plot(data(:,end),data(:,40));
% plot(data(:,end),filtfilt(b,a,data(:,20)));
xlabel('Tempo[s]');
ylabel('Velocidade [m/s]');
legend('$\dot{y}_{des}$', '$\dot{y}$','interpreter','latex');

subplot(313)
hold on;
grid on;
plot(data(:,end),data(:,9));
plot(data(:,end),data(:,21));
% plot(data(:,end),data(:,41));
% plot(data(:,end),filtfilt(b,a,data(:,21)));
xlabel('Tempo[s]');
ylabel('Velocidade [m/s]');
legend('$\dot{z}_{des}$', '$\dot{z}$','interpreter','latex');

%% acc
figure();
subplot(311)
hold on;
grid on;
% plot(data(:,end),data(:,7));
% plot(data(:,end),data(:,19));
plot(data(:,end),filtfilt(b,a,data(:,34)));
axis([0 data(end,end) -5 5])
xlabel('Tempo[s]');
ylabel('acc [m/s2]');
legend('Xdot_des', 'Xdot');

subplot(312)
hold on;
grid on;
% plot(data(:,end),data(:,8));
% plot(data(:,end),data(:,20));
plot(data(:,end),filtfilt(b,a,data(:,35)));
axis([0 data(end,end) -5 5])
xlabel('Tempo[s]');
ylabel('acc [m/s2]');
legend('Ydot_des', 'Ydot');

subplot(313)
hold on;
grid on;
% plot(data(:,end),data(:,9));
% plot(data(:,end),data(:,21));
plot(data(:,end),filtfilt(b,a,data(:,36)));
axis([0 data(end,end) -5 5])
xlabel('Tempo[s]');
ylabel('acc [m/s2]');
legend('Zdot_des', 'Zdot');
%%

figure();
hold on;
grid on;
plot(data(:,1),data(:,2));
plot(data(:,13),data(:,14));
title('XY');
xlabel('X [m]');
ylabel('Y [m]');

%%
figure();
subplot(311)
hold on;
grid on;
plot(data(:,end),data(:,46));
plot(data(:,end),data(:,42));
plot(data(:,end),rad2deg(data(:,17)));

axis([0 data(end,end) -30 30])
title('roll');
legend('$\phi ref_{raw}$', '$\phi ref$','$\phi$','interpreter','latex');
xlabel('time [s]');
ylabel('roll [degress]');

subplot(312)
hold on;
grid on;
plot(data(:,end),data(:,47));
plot(data(:,end),data(:,43));
plot(data(:,end),-rad2deg(data(:,16)));
axis([0 data(end,end) -30 30])
title('pitch');
legend( '$\theta ref_{raw}$', '$\theta ref $','$\theta$','interpreter','latex');
xlabel('time [s]');
ylabel('pitch [degress]');

subplot(313)
hold on;
grid on;
% plot(data(:,end),data(:,49));
plot(data(:,end),filtfilt(b,a,data(:,26)));
plot(data(:,end),data(:,25)/conversao_digital);
axis([0 data(end,end) 5 15])
title('thrust');
legend('$thrust ref$','$thrust_{exec}$','interpreter','latex');
% legend('$thrust ref_{raw}$', '$thrust ref$','$thrust_{exec}$','interpreter','latex');
xlabel('time [s]');
ylabel('pitch [degress]');

%% 
figure();
subplot(321)
hold on;
grid on;
plot(data(:,end),data(:,end-6));
axis([0 data(end,end) -10 10])
xlabel('time [s]');
ylabel('x');

subplot(323)
hold on;
grid on;
plot(data(:,end),data(:,end-5));
axis([0 data(end,end) -10 10])
xlabel('time [s]');
ylabel('y');

subplot(325)
hold on;
grid on;
plot(data(:,end),data(:,end-4));
axis([0 data(end,end) -10 10])
xlabel('time [s]');
ylabel('z');

subplot(322)
hold on;
grid on;
plot(data(:,end),data(:,end-3));
axis([0 data(end,end) -10 10])
xlabel('time [s]');
ylabel('dx');

subplot(324)
hold on;
grid on;
plot(data(:,end),data(:,end-2));
axis([0 data(end,end) -10 10])
xlabel('time [s]');
ylabel('dy');

subplot(326)
hold on;
grid on;
plot(data(:,end),data(:,end-1));
axis([0 data(end,end) -10 10])
xlabel('time [s]');
ylabel('dz');