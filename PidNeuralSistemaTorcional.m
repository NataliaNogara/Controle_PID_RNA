clear all
close all

%Inicialização dos parâmetros
T=0.009; % tempo de amostragem
N=250; % número de iterações
Amax=5;
y=zeros(1,N); % saída da planta
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

% Parâmetros de ajuste da parte P
eta1=0.009;
w1=w1*0.0009;

% Parâmetros de ajuste da parte I
eta2=0.00001;
w2=w2*0.0001;

% Parâmetros de ajuste a parte D
eta3=0.09;
w3=w3*0.01;

% Função de transferência do sistema torcional
num=1600;
den=[1 0.83 0];
Gp=tf(num,den);
Gpd=c2d(Gp,T,'zoh'); % A partir de Gpd obtenho a equação de diferenças

% PID baseado em uma Rede Neural Artificial 
for k=3:N
r(k)=1.0; % sinal de referência
% eq. de dif. da planta
y(k)=0.06464*u(k-1)+0.06448*u(k-2)+1.993*y(k-1)-0.9926*y(k-2); 
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

% Plotar gráficos
k=0:N-1;

figure(1)
plot(k,r,'-.',k,y,'-')
legend('r(k)','y(k)');
title ('Saída do sistema com PID neural');
xlabel('kT');
ylabel('Saída y(k)');
grid on;
 
figure(2)
plot(k,u,'-');
legend('u(k)');
title ('Ação de controle do sistema com PID neural');
xlabel('kT');
ylabel('Sinal de controle u(k)');
grid on;

figure(3)
plot(k,w1,k,w2,k,w3)
legend('w1','w2','w3');
title ('Convergência dos pesos')
xlabel('kT');
ylabel('Amplitude');
grid on;

u=[zeros(1,2) ones(1,N-2)];
r=u;

y(1)=0.0;
y(2)=0.0;

for k=3:N
y(k)=0.06464*u(k-1)+0.06448*u(k-2)+1.993*y(k-1)-0.9926*y(k-2); 
end

k=0:N-1;
figure(4)
plot(k,r,'-.',k,y,'-');
legend('r(k)','y(k)');
title ('Saída do sistema em malha aberta');
xlabel('kT');
ylabel('y(k)');
grid on;