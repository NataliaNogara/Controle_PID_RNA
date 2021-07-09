clear all
close all

%Inicializa��o dos par�metros
T=0.1; % tempo de amostragem
N=100; % n�mero de itera��es
Amax=5;
y=zeros(1,N); % sa�da da planta
u=zeros(1,N); % sinal de controle
e=zeros(1,N); % erro
w1=ones(1,N); % peso proporcional
w2=ones(1,N); % peso integral
w3=ones(1,N); % peso derivativo
I=zeros(1,N);
phiI=zeros(1,N);
deltaw1=zeros(1,N);
deltaw2=zeros(1,N);
deltaw3=zeros(1,N);
w1linha=zeros(1,N);
w2linha=zeros(1,N);
w3linha=zeros(1,N);

% Par�metros de ajuste da parte P
eta1=0.1; 
w1=w1*1.5;

% Par�metros de ajuste da parte I
eta2=0.1; 
w2=w2*0.1;

% Par�metros de ajuste a parte D
eta3=0.01;
w3=w3*0.0;

% Fun��o de transfer�ncia do circuito RC
R=10e3;
C=47e-6;
tau=R*C;
num=1;
den=[tau 1];
Gp=tf(num,den);
Gpd=c2d(Gp,T,'zoh'); %A partir da Gpd obtenho a eq. de dif.

% PID baseado em uma Rede Neural Artificial 
for k=3:N
r(k)=1.0; % sinal de refer�ncia
y(k)=0.1917*u(k-1)+0.8083*y(k-1); % eq. de dif. da planta
e(k)=r(k)-y(k); % sinal do erro

x1(k)=e(k)-e(k-1); 
x2(k)=e(k);
x3(k)=e(k)-2*e(k-1)+e(k-2);

I(k)=x1(k)*w1(k)+x2(k)*w2(k)+x3(k)*w3(k);
phiI(k)=(Amax*(1-exp(-I(k))))/(1+exp(-I(k)));
u(k)=u(k-1)+phiI(k);    

deltaw1(k)=eta1*x1(k)*e(k)*u(k); 
deltaw2(k)=eta2*x2(k)*e(k)*u(k);
deltaw3(k)=eta3*x3(k)*e(k)*u(k);

w1linha(k)=w1(k-1)+deltaw1(k); 
w2linha(k)=w2(k-1)+deltaw2(k); 
w3linha(k)=w3(k-1)+deltaw3(k); 

w1(k)=w1linha(k)/sqrt(w1linha(k)^2+w2linha(k)^2+w3linha(k)^2); % Kp
w2(k)=w2linha(k)/sqrt(w1linha(k)^2+w2linha(k)^2+w3linha(k)^2); % Ki
w3(k)=w3linha(k)/sqrt(w1linha(k)^2+w2linha(k)^2+w3linha(k)^2); % Kd 

I(k)=x1(k)*w1(k)+x2(k)*w2(k)+x3(k)*w3(k);
phiI(k)=(Amax*(1-exp(-I(k))))/(1+exp(-I(k)));
u(k)=u(k-1)+phiI(k);
end

% Plotar gr�ficos
k=0:N-1;

figure(1)
plot(k,r,'-.',k,y,'-')
legend('r(k)','y(k)');
title ('Sa�da do sistema com PID neural');
xlabel('kT');
ylabel('Sa�da y(k)');
grid on;

figure(2)
plot(k,u,'-');
legend('u(k)');
title ('A��o de controle do sistema com PID neural');
xlabel('kT');
ylabel('Sinal de controle u(k)');
grid on;

figure(3)
plot(k,w1,k,w2,k,w3)
legend('w1','w2','w3');
title ('Converg�ncia dos pesos')
xlabel('kT');
ylabel('Amplitude');
grid on;

u=[zeros(1,2) ones(1,N-2)];
r=u;

y(1)=0.0;
y(2)=0.0;

for k=3:N
y(k)=0.1917*u(k-1)+0.8083*y(k-1);
end

k=0:N-1;
figure(4)
plot(k,r,'-.',k,y,'-');
legend('r(k)','y(k)');
title ('Sa�da do sistema em malha aberta');
xlabel('kT');
ylabel('y(k)');
grid on;