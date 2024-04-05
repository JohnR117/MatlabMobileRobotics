%Control P de un robot tomando punto por fuera
close all; clear; clc;
a=sim('ModCin');
P=a.get('P');
Pd=a.get('Pd');
t=a.get('t');
v=a.get('v');
w=a.get('w');

figure(1)
plot(P(:,1),P(:,2),'--r',Pd(:,1),Pd(:,2),'b');
grid on, hold on;
plot(P(1,1),P(1,2),'*r');
xlabel('X[m]');
ylabel('Y[m]'), title('Posicion en el plano');
legend('P','Pd');

figure(2)
plot(t,Pd(:,1)-P(:,1),'r',t,Pd(:,2)-P(:,2),'b');
grid on;
xlabel('t[s]');
ylabel('e[m]');
title('Errores de seguimiento');
legend('e_{x}','e_{y}');

figure(3)
plot(t,v,'r',t,w,'b')
grid on;
xlabel('t[s]');
ylabel('U[m/s,rad/s]');
title('Entradas de control');
legend('v','w');