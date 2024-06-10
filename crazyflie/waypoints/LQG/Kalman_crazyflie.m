function drone = Kalman_crazyflie(drone,flaggps)
dt = drone.pPar.T_loop;
m = 0.035; g = 9.81; psi = drone.pPos.X(6);
kvx = 0.03; kvy = 0.03; kvz = 0;

% covariancia do modelo
Rw = diag([0.01 0.01 0.01 0.2 0.2 0.2]);
% covariancia das medidas
% Rv = diag([0.8 0.8 0.8 0.01 0.01 0.01]);
% Rv = diag([0.1 0.1 0.01 0.01 0.01 0.01]);

J = [1, 0, 0,    dt*cos(psi),   -dt*sin(psi),              0
     0, 1, 0,    dt*sin(psi),    dt*cos(psi),              0
     0, 0, 1,              0,              0,             dt
     0, 0, 0, 1 - (dt*kvx)/m,              0,              0
     0, 0, 0,              0, 1 - (dt*kvy)/m,              0
     0, 0, 0,              0,              0, 1 - (dt*kvz)/m];
 
 Rwb = [cos(psi) sin(psi) 0; -sin(psi) cos(psi) 0;0 0 1];

 M = [drone.pPos.X_sujo(1:3);Rwb*drone.pPos.X(7:9)];
%  x = M(1); y = M(2); z = M(3);
 dxb = M(4); dyb = M(5); dzb = M(6);
if drone.pPar.counter < 2
     u = [0;0;0];
 else
     u = drone.pSC.U_LQR;
 end
 c = u(3); theta = drone.pPos.X(5); phi = drone.pPos.X(4);
 

 
 
% estimated state 
dxw = cos(psi)*dxb - sin(psi)*dyb;
dyw = sin(psi)*dxb + cos(psi)*dyb;
dzw = dzb;
ddxb = -kvx/m*dxb + c/m*sin(theta);
ddyb = -kvy/m*dyb + c/m*sin(phi);
ddzb = -kvz/m*dzb + c/m*cos(theta)*cos(phi) -g;

% matriz transição (modelo) - estado estimado
if drone.pPar.counter < 2
    drone.pPos.T = M; %aprox
else
    drone.pPos.T = [drone.pPos.T(1) + dxw*dt;
                    drone.pPos.T(2) + dyw*dt;
                    drone.pPos.T(3) + dzw*dt;
                    drone.pPos.T(4) + ddxb*dt;
                    drone.pPos.T(5) + ddyb*dt;
                    drone.pPos.T(6) + ddzb*dt];
end

 % error covariance
 if drone.pPar.counter < 2
     drone.pPos.L = diag([0 0 0 0 0 0]);
 else
     drone.pPos.L =  J*drone.pPos.L*J' + Rw;
 end
 % flaggps=1;
 % flaggps
 if ~flaggps
    M = M(4:6); % Medições
    Rv = diag([0.01 0.01 0.01]); % Covariancia das medidas
    H = [zeros(3) eye(3)]; % Matriz de saída
 else
    Rv = diag([0.8 0.8 0.8 0.01 0.01 0.01]); % Covariancia das medidas
    H = eye(6); % Matriz de saída
    disp('entrou')
 end
 
 % inovação
 I = M - H*drone.pPos.T;
 
 % covariancia da inovacão
 LL = H*drone.pPos.L*H' + Rv;
 
 % Kalman gain
 KK = drone.pPos.L*H'*pinv(LL);
 
 % estimated state final
 
 drone.pPos.T = drone.pPos.T + KK*I;
 
 drone.pPos.L =  (eye(6) - KK*H)*drone.pPos.L;

% % 
% syms x y z psi dxw dyw dzw dxb dyb dzb kvx kvy kvz m c theta phi g dt
% % states x y z dxb dyb dzb
% 
% % dinâmica
% dxw = cos(psi)*dxb - sin(psi)*dyb;
% dyw = sin(psi)*dxb + cos(psi)*dyb;
% dzw = dzb;
% ddxb = -kvx/m*dxb + c/m*sin(theta);
% ddyb = -kvy/m*dyb + c/m*sin(phi);
% ddzb = -kvz/m*dzb + c/m*cos(theta)*cos(phi) -g;
% 
% % matriz transição (modelo)
% T = [x + dxw*dt;
%      y + dyw*dt;
%      z + dzw*dt;
%      dxb + ddxb*dt;
%      dyb + ddyb*dt;
%      dzb + ddzb*dt];
%  
% J = jacobian(T,[x y z dxb dyb dzb]);
% 
%