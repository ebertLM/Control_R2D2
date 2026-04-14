close all;
%____________ Graficas ______________________
T1_max = ROBOT_DATA.T1_max;
T2_max = ROBOT_DATA.T2_max;
T3_max = ROBOT_DATA.T3_max;
Vc1_max = ROBOT_DATA.Vc1_max;
Vc2_max = ROBOT_DATA.Vc2_max;
Vc3_max = ROBOT_DATA.Vc3_max;
% q1_min = ROBOT_DATA.q1_min;
% q2_min = ROBOT_DATA.q2_min;
% q3_min = ROBOT_DATA.q3_min;
% q1_max = ROBOT_DATA.q1_max;
% q2_max = ROBOT_DATA.q2_max;
% q3_max = ROBOT_DATA.q3_max;
dq1_max = ROBOT_DATA.dq1_max;
dq2_max = ROBOT_DATA.dq2_max;
dq3_max = ROBOT_DATA.dq3_max;
ddq1_max = ROBOT_DATA.ddq1_max;
ddq2_max = ROBOT_DATA.ddq2_max;
ddq3_max = ROBOT_DATA.ddq3_max;
Kt1 = ROBOT_DATA.Kt1;
Kt2 = ROBOT_DATA.Kt2;
Kt3 = ROBOT_DATA.Kt3;

%------------------------------------------------------------%
% 7. PLOTEO DE RESULTADOS
%------------------------------------------------------------%
% _________________7.1 Singularidades ________________________
Plotter.drawSingularity(Q(2,:),Q(3,:))

% ___________________7.2 Eje temporal_________________________
time = linspace(0,(MM-1)*Ts,MM);


simin1 = [time',Q(1,:)'];
simin2 = [time',Q(2,:)'];
simin3 = [time',Q(3,:)'];
% ___________________7.3 Perfil trapezoidal___________________
% Posición de s(t)
figure(Name='Perfil trapezoidal',Position=[100 100 800 700])
subplot(311)
plot(time, S(:),'-r', 'Linewidth',2)
legend('s(t)')
xlabel('[s]')
ylabel('s(t)')
title('Ley temporal del perfil trapezoidal s(t)')
grid on
axis([time(1)  time(end) min(S(1,:)') max(S(1,:)')])
set(gca,'box', 'on')

% Velocidad de s(t)
subplot(312)
plot(time, dS(:),'-r', 'Linewidth',2)
legend('ds(t)')
xlabel('[s]')
ylabel('ds(t)')
title('Velocidad de s(t)')
grid on
axis([time(1)  time(end) min(dS(1,:)') max(dS(1,:)')])

% Aceleracion de s(t)
subplot(313)
plot(time, ddS(:),'-r', 'Linewidth',2)
legend('dds(t)')
xlabel('[s]')
ylabel('dds(t)')
title('Aceleracion de s(t)')
grid on
axis([time(1)  time(end) min(ddS(1,:)') max(ddS(1,:)')])

% ___________________7.4 Posicion cartesiana__________________
% Posición en el eje "X"
figure(Name='Posiciones',Position=[100 100 800 700])
subplot(311)
plot(time,XYZ(1,:)*1e+3,'-r', time,PD(1,:)*1e+3,'-b', 'Linewidth',2)
legend('x', 'xd')
xlabel('[s]')
ylabel('x[mm]')
title('Posición X[mm]')
grid on
axis([time(1)  time(end) min([XYZ(1,:)';PD(1,:)'])*1000-10 ...
                         max([XYZ(1,:)';PD(1,:)'])*1000+10])
set(gca,'box', 'on')

% Posición en el eje "Y"
subplot(312)
plot(time,XYZ(2,:)*1e+3,'-r', time,PD(2,:)*1e+3,'-b', 'Linewidth',2)
legend('y', 'yd')
xlabel('[s]')
ylabel('y[mm]')
title('Posición Y[mm]')
grid on
axis([time(1)  time(end) min([XYZ(2,:)';PD(2,:)'])*1000-10 ...
                         max([XYZ(2,:)';PD(2,:)'])*1000+10])

% Posición en el eje "Z"
subplot(313)
plot(time,XYZ(3,:)*1e+3,'-r', time,PD(3,:)*1e+3,'-b', 'Linewidth',2)
legend('z', 'zd')
xlabel('[s]')
ylabel('z[mm]')
title('Posición Z[mm]')
grid on
axis([time(1)  time(end) min([XYZ(3,:)';PD(3,:)'])*1000-10 ...
                         max([XYZ(3,:)';PD(3,:)'])*1000+10])
% ___________________7.4 Errores de posición__________________
% Error de posición en cada eje 
figure(Name='Error in Position en cada eje', Position=[100 100 800 600])
plot(time,errXYZ(1,:)*1000,'-r', time,errXYZ(2,:)*1000,'-b', time,errXYZ(3,:)*1000,'-m', 'Linewidth',2)
grid on
set(gca,'box', 'on')
legend('ex', 'ey', 'ez')
ylabel('error[mm]')
xlabel('t[s]')
title('Error X-Y-Z')
axis([time(1) time(end) min([errXYZ(1,:)';errXYZ(2,:)';errXYZ(3,:)'])*1000-2.5...
                        max([errXYZ(1,:)';errXYZ(2,:)';errXYZ(3,:)'])*1000+2.5])

% 3. Error distancia euclideana del punto de trabajo XYZ
errXYZrms = sqrt(sum(errXYZ.^2));
figure(Name='Error in Position total', Position=[100 100 800 600])
plot(time,errXYZrms*1000,'-k', 'Linewidth',2)
grid on
set(gca,'box', 'on')
legend('error X-Y-Z')
ylabel('error[mm]')
xlabel('t[s]')
title('Error total XYZ')
axis([time(1) time(end) min(errXYZrms')*1000-2.5...
                        max(errXYZrms')*1000+2.5])
% __________________7.5 Posicion angular_____________________
%  7.2. POSICIÓN ANGULAR
figure(Name='Joints Position', Position=[100, 100, 1100, 800])
% Q1
subplot(311)
plot(time, Q(1,:)*180/pi,'-r', time,QD(1,:)*180/pi,'-b', "LineWidth",2);
legend('q1', 'q1 deseado')
xlabel('[s]');
ylabel('[sexag]');
title('Posición angular Q1');
grid on
% axis([0 time(end) q1_min q1_max])
axis([0 time(end) min([Q(1,:)'*180/pi; QD(1,:)'*180/pi])-10 max([Q(1,:)'*180/pi; QD(1,:)'*180/pi])+10])
% Q2
subplot(312)
plot(time, Q(2,:)*180/pi,'-r', time,QD(2,:)*180/pi,'-b', "LineWidth",2);
legend('q2', 'q2 deseado')
xlabel('[s]');
ylabel('[sexag]');
title('Posición angular Q2');
grid on
axis([0 time(end) min([Q(2,:)'*180/pi; QD(2,:)'*180/pi])-5 max([Q(2,:)'*180/pi; QD(2,:)'*180/pi])+5])
% Q3
subplot(313)
plot(time, Q(3,:)*180/pi,'-r', time,QD(3,:)*180/pi,'-b', "LineWidth",2);
legend('q3', 'q3 deseado')
xlabel('[s]');
ylabel('[sexag]');
title('Posición angular Q3');
grid on
axis([0 time(end) min([Q(3,:)'*180/pi; QD(3,:)'*180/pi])-10 max([Q(3,:)'*180/pi; QD(3,:)'*180/pi])+10])

% VELOCIDAD ANGULAR 
figure(Name='Joints Velocity and Acceleration', Position=[100, 100, 1100, 800])
subplot(211)
plot(time, dQ(1,:), time, dQ(2,:), time, dQ(3,:), "LineWidth",2);
legend('dq1', 'dq2','dq3')
xlabel('[s]');
ylabel('dq[rad/s]');
title('Velocidad angular');
grid on
axis([0 time(end) -max([dq1_max;dq2_max;dq3_max]) max([dq1_max;dq2_max;dq3_max])])

% ACELERACIÓN ANGULAR
subplot(212)
plot(time, ddQ(1,:), time, ddQ(2,:), time, ddQ(3,:), "LineWidth",2);
legend('ddq1', 'ddq2','ddq3')
xlabel('[s]');
ylabel('ddq[rad/s^2]');
title('Aceleración angular');
grid on
axis([0 time(end) -max([ddq1_max;ddq2_max;ddq3_max]) max([ddq1_max;ddq2_max;ddq3_max])])


%  7.3. VOLTAJE DE ARMADURA, TORQUES Y CORRIENTES
figure(Name='Voltage', Position=[100, 100, 1100, 800])
% Voltaje de armadura(Va)
subplot(2,3,1)
plot(time, VA(1,:), '-m', "LineWidth",2);
title('Voltaje de armadura motor 1');
grid on
axis([0 time(end) -15 15])
subplot(2,3,2)
plot(time, VA(2,:), '-b',"LineWidth",2);
title('Voltaje de armadura motor 2');
grid on
axis([0 time(end) -15 15])
subplot(2,3,3)
plot(time, VA(3,:), '-r', "LineWidth",2);
title('Voltaje de armadura motor 3');
grid on
axis([0 time(end) -15 15])
% Voltaje de control(Vc)
subplot(2,3,4)
plot(time, VC(1,:), '-m', "LineWidth",2);
title('Voltaje de control motor 1');
grid on
axis([0 time(end) -Vc1_max Vc1_max])
subplot(2,3,5)
plot(time, VC(2,:),'-b', "LineWidth",2);
title('Voltaje de control motor 2');
grid on
axis([0 time(end) -Vc2_max Vc2_max])
subplot(2,3,6)
plot(time, VC(3,:), '-r', "LineWidth",2);
title('Voltaje de control motor 3');
grid on
axis([0 time(end) -Vc3_max Vc3_max])

% Torques de control
figure(Name='TorqueAndCurrent', Position=[100, 100, 1100, 800])
subplot(2,3,1)
plot(time, TAU_L(1,:),'-m',"LineWidth",2);
xlabel('[s]');
ylabel('[N/m]');
title('Torque de control motor 1');
grid on
axis([0 time(end) -T1_max T1_max])
subplot(2,3,2)
plot(time, TAU_L(2,:),'-b',"LineWidth",2);
xlabel('[s]');
ylabel('[N/m]');
title('Torque de control motor 2');
grid on
axis([0 time(end) -T2_max T2_max])
subplot(2,3,3)
plot(time, TAU_L(3,:),'-r',"LineWidth",2);
xlabel('[s]');
ylabel('[N/m]');
title('Torque de control motor 3');
grid on
axis([0 time(end) -T3_max T3_max])
% Corrientes de control
subplot(2,3,4)
plot(time, TM(1,:)/Kt1,'-m',"LineWidth",2);
xlabel('[s]');
ylabel('[A]');
title('Corriente motor 1');
grid on
axis([0 time(end) -2 2])
subplot(2,3,5)
plot(time, TM(2,:)/Kt2,'-b',"LineWidth",2);
xlabel('[s]');
ylabel('[A]');
title('Corriente motor 2');
grid on
axis([0 time(end) -2 2])
subplot(2,3,6)
plot(time, TM(3,:)/Kt3,'-r',"LineWidth",2);
xlabel('[s]');
ylabel('[A]');
title('Corriente motor 3');
grid on
axis([0 time(end) -1 1])