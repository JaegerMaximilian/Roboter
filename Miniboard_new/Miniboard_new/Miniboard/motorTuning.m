%% Programm um Daten vom Regler zu visualisieren
clear all; clc; close all;
M = csvread('motor.dat',1,0,[1,0,1000,5]);
l= length(M);
t = linspace(1,100,l);

%% Geschwindigkeiten Soll Vs Ist
close all; clc;
figure
plot(t,M(1:end,1));grid on; hold on;
plot(t,M(1:end,2));
plot(t,M(1:end,4));
plot(t,M(1:end,5));
title('Geschwindigkeit');
legend('Motor Rechts Soll','Motor Rechts Ist','Motor Links Soll','Motor Links Ist');
ylim([-0.2,0.8]);
hold off;
%% Stellgrößen
 clc;
iN =5;
offset =20;
y_glatt_rechts = filter(ones(1,iN)/iN, 1, M(1:end,3));% Glätten der Daten
y_glatt_links= filter(ones(1,iN)/iN, 1, M(1:end,6));% Glätten der Daten

figure
subplot(2,1,1)
plot(t(offset:end),M(offset:end,3)); hold on; grid on;
plot(t(offset:end),y_glatt_rechts(offset:end));
title('Stellgröße Motor Rechts');
subplot(2,1,2)
plot(t(offset:end),M(offset:end,6));hold on; grid on;
plot(t(offset:end),y_glatt_links(offset:end));
title('Stellgröße Motor Links')
hold off;




