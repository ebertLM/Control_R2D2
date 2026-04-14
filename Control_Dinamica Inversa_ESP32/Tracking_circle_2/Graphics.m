%___________ Parámetros constantes _____________
T1_max = ROBOT_DATA.T1_max;
T2_max = ROBOT_DATA.T2_max;
T3_max = ROBOT_DATA.T3_max;
Vc1_max = ROBOT_DATA.Vc1_max;
Vc2_max = ROBOT_DATA.Vc2_max;
Vc3_max = ROBOT_DATA.Vc3_max;
dq1_max = ROBOT_DATA.dq1_max;
dq2_max = ROBOT_DATA.dq2_max;
dq3_max = ROBOT_DATA.dq3_max;
ddq1_max = ROBOT_DATA.ddq1_max;
ddq2_max = ROBOT_DATA.ddq2_max;
ddq3_max = ROBOT_DATA.ddq3_max;

%____________ Graficas ______________________
%  1. Eje de tiempo
time = linspace(0,(lengthColumn-1)*Ts,lengthColumn);

%  2.1. Posición en el eje "X"
figure(Name='Posiciones',Position=[100 100 800 700])
subplot(311)
plot(time,XYZ(1,:)*1e+3,'-r', time,PD(1,:)*1e+3,'-b', 'Linewidth',2)
legend('x', 'xd')
xlabel('[s]')
ylabel('x[mm]')
title('Posición X[mm]')
grid on
axis([time(1)  time(end) min([XYZ(1,:)';PD(1,:)'])*1000-2 ...
                         max([XYZ(1,:)';PD(1,:)'])*1000+2])
set(gca,'box', 'on')

%  2.2. Posición en el eje "Y"
subplot(312)
plot(time,XYZ(2,:)*1e+3,'-r', time,PD(2,:)*1e+3,'-b', 'Linewidth',2)
legend('y', 'yd')
xlabel('[s]')
ylabel('y[mm]')
title('Posición Y[mm]')
grid on
axis([time(1)  time(end) min([XYZ(2,:)';PD(2,:)'])*1000-10 ...
                         max([XYZ(2,:)';PD(2,:)'])*1000+10])

%  2.3. Posición en el eje "Z"
subplot(313)
plot(time,XYZ(3,:)*1e+3,'-r', time,PD(3,:)*1e+3,'-b', 'Linewidth',2)
legend('z', 'zd')
xlabel('[s]')
ylabel('z[mm]')
title('Posición Z[mm]')
grid on
axis([time(1)  time(end) min([XYZ(3,:)';PD(3,:)'])*1000-10 ...
                         max([XYZ(3,:)';PD(3,:)'])*1000+10])

%  3.1. Posición angular Q1
figure(Name='Joints Position', Position=[100, 100, 1100, 800])
subplot(311)
plot(time, Q(1,:)*180/pi,'-r', time,QD(1,:)*180/pi,'-b', "LineWidth",2);
legend('q1', 'q1 deseado')
xlabel('[s]');
ylabel('[sexag]');
title('Posición angular Q1');
grid on
axis([0 time(end) min([Q(1,:)'*180/pi; QD(1,:)'*180/pi])-10 max([Q(1,:)'*180/pi; QD(1,:)'*180/pi])+10])

%  3.2. Posición angular Q2
subplot(312)
plot(time, Q(2,:)*180/pi,'-r', time,QD(2,:)*180/pi,'-b', "LineWidth",2);
legend('q2', 'q2 deseado')
xlabel('[s]');
ylabel('[sexag]');
title('Posición angular Q2');
grid on
axis([0 time(end) min([Q(2,:)'*180/pi; QD(2,:)'*180/pi])-5 max([Q(2,:)'*180/pi; QD(2,:)'*180/pi])+5])

%  3.3. Posición angular Q3
subplot(313)
plot(time, Q(3,:)*180/pi,'-r', time,QD(3,:)*180/pi,'-b', "LineWidth",2);
legend('q3', 'q3 deseado')
xlabel('[s]');
ylabel('[sexag]');
title('Posición angular Q3');
grid on
axis([0 time(end) min([Q(3,:)'*180/pi; QD(3,:)'*180/pi])-10 max([Q(3,:)'*180/pi; QD(3,:)'*180/pi])+10])





%  3.1. Velocidad angular Q1
figure(Name='Joints Velocidad angular', Position=[100, 100, 1100, 800])
subplot(311)
plot(time, dQ(1,:),'-r', time,dQD(1,:),'-b', "LineWidth",2);
legend('dq1', 'dq1 deseado')
xlabel('[s]');
ylabel('[º/s]');
title('Velocidad angular-Q1');
grid on
axis([0 time(end) min([dQ(1,:)'; dQD(1,:)'])-10 max([dQ(1,:)'; dQD(1,:)'])+10])

%  3.2. Velocidad angular Q2
subplot(312)
plot(time, dQ(2,:),'-r', time,dQD(2,:),'-b', "LineWidth",2);
legend('dq2', 'dq2 deseado')
xlabel('[s]');
ylabel('[º/s]');
title('Velocidad angular-Q2');
grid on
axis([0 time(end) min([dQ(2,:)'; dQD(2,:)'])-5 max([dQ(2,:)'; dQD(2,:)'])+5])

%  3.3. Velocidad angular Q3
subplot(313)
plot(time, dQ(3,:),'-r', time,dQD(3,:),'-b', "LineWidth",2);
legend('dq3', 'dq3 deseado')
xlabel('[s]');
ylabel('[º/s]');
title('Velocidad angular-Q3');
grid on
axis([0 time(end) min([dQ(3,:)'; dQD(3,:)'])-10 max([dQ(3,:)'; dQD(3,:)'])+10])




%  3.1. Aceleracion angular Q1
figure(Name='Joints Aceleracion angular', Position=[100, 100, 1100, 800])
subplot(311)
plot(time, ddQ(1,:),'-r', time,ddQD(1,:),'-b', "LineWidth",2);
legend('ddq1', 'ddq1 deseado')
xlabel('[s]');
ylabel('[º/s^2]');
title('Aceleracion angular-Q1');
grid on
axis([0 time(end) min([ddQ(1,:)'; ddQD(1,:)'])-10 max([ddQ(1,:)'; ddQD(1,:)'])+10])

%  3.2. Aceleracion angular Q2
subplot(312)
plot(time, ddQ(2,:),'-r', time,ddQD(2,:),'-b', "LineWidth",2);
legend('ddq2', 'ddq2 deseado')
xlabel('[s]');
ylabel('[º/s^2]');
title('Aceleracion angular-Q2');
grid on
axis([0 time(end) min([ddQ(2,:)'; ddQD(2,:)'])-10 max([ddQ(2,:)'; ddQD(2,:)'])+10])

%  3.3. Aceleracion angular Q3
subplot(313)
plot(time, ddQ(3,:),'-r', time,ddQD(3,:),'-b', "LineWidth",2);
legend('ddq3', 'ddq3 deseado')
xlabel('[s]');
ylabel('[º/s^2]');
title('Aceleracion angular-Q3');
grid on
axis([0 time(end) min([ddQ(3,:)'; ddQD(3,:)'])-10 max([ddQ(3,:)'; ddQD(3,:)'])+10])


%  6. Error de la posición angular
figure(Name='Error de la posicion angular', Position=[100 100 800 600])
plot(time,errQ(1,:)*180/pi,'-r', time,errQ(2,:)*180/pi,'-b', time,errQ(3,:)*180/pi,'-m', 'Linewidth',2)
grid on
set(gca,'box', 'on')
legend('err-q1', 'err-q2', 'err-q3')
ylabel('error[grados]')
xlabel('t[s]')
title('Error q1-q2-q3')
axis([time(1) time(end) min([errQ(1,:)';errQ(2,:)';errQ(3,:)'])*180/pi-2.5 ...
                        max([errQ(1,:)';errQ(2,:)';errQ(3,:)'])*180/pi+2.5])

%  6. Error de posición en XYZ
figure(Name='Error in Position', Position=[100 100 800 600])
plot(time,errXYZ(1,:)*1000,'-r', time,errXYZ(2,:)*1000,'-b', time,errXYZ(3,:)*1000,'-m', 'Linewidth',2)
grid on
set(gca,'box', 'on')
legend('ex', 'ey', 'ez')
ylabel('error[mm]')
xlabel('t[s]')
title('Error X-Y-Z')
axis([time(1) time(end) min([errXYZ(1,:)';errXYZ(2,:)';errXYZ(3,:)'])*1000-2.5 ...
                        max([errXYZ(1,:)';errXYZ(2,:)';errXYZ(3,:)'])*1000+2.5])


%  7. Voltaje de armadura
figure(Name='Voltage', Position=[100, 100, 1100, 800])

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

%  8. Voltaje de control
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

%  9. Torque de control
figure(Name='TorqueAndCurrent', Position=[100, 100, 1100, 800])
subplot(2,3,1)
plot(time, TT(1,:),'-m',"LineWidth",2);
xlabel('[s]');
ylabel('[N/m]');
title('Torque de control motor 1');
grid on
axis([0 time(end) -T1_max T1_max])
subplot(2,3,2)
plot(time, TT(2,:),'-b',"LineWidth",2);
xlabel('[s]');
ylabel('[N/m]');
title('Torque de control motor 2');
grid on
axis([0 time(end) -T2_max T2_max])
subplot(2,3,3)
plot(time, TT(3,:),'-r',"LineWidth",2);
xlabel('[s]');
ylabel('[N/m]');
title('Torque de control motor 3');
grid on
axis([0 time(end) -T3_max T3_max])

%  10. Corriente de control
subplot(2,3,4)
plot(time, IC(1,:),'-m',"LineWidth",2);
xlabel('[s]');
ylabel('[A]');
title('Corriente armadura-Motor 1');
grid on
axis([0 time(end) -2 2])
subplot(2,3,5)
plot(time, IC(2,:),'-b',"LineWidth",2);
xlabel('[s]');
ylabel('[A]');
title('Corriente armadura-Motor 2');
grid on
axis([0 time(end) -2 2])
subplot(2,3,6)
plot(time, IC(3,:),'-r',"LineWidth",2);
xlabel('[s]');
ylabel('[A]');
title('Corriente armadura-Motor 3');
grid on
axis([0 time(end) -1 1])