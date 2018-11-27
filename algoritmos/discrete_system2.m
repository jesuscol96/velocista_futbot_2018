close all
clear;

%system
K=4.2;
tau=1.79;

%allow saturation
sat=true;

%time vector
ts=6/1000;  %fc=580 Hz
tmax=0.5;
t=[0:ts:tmax];
n=length(t)

%PID controller
Kp=60.5;
Ki=1816;
Kd=0;
sp=5;
A=Kp+Ki*ts+(Kd/ts);		
B=-Kp-(2*Kd/ts);
C=Kd/ts;

%continuous system
s=tf('s');
G=K/(tau*s+1);
Gc=Kp+(Ki/s)+Kd*s;
Gf=G*Gc/(1+G*Gc);

%input
x=1; %step
%salida
y=[0];
err=[0 0];
y_c=[0]; %controller output
y_c_max=13;

for k=1:n-1
    
  err_new=sp-y(k);
  y_c_next=A*err_new+B*err(1)+C*err(2)+y_c(k);
  
  if y_c_next > y_c_max && sat
    y_c_next=y_c_max;
   end
  y_c=[y_c y_c_next];
  err=[err_new err(1)];
  
  y_next=(ts/(ts+tau))*K*y_c_next+(tau/(ts+tau))*y(k);
  y=[y y_next];     
end

figure;
scatter(t,y,'r');
hold on;
step(sp*Gf,tmax);
title('Discrete/Continuous system response');
xlabel('Time (s)');
xlabel('Output');
legend('Discrete response','Ideal response');

figure;
scatter(t,y_c,'r');
hold on;
plot(t,y_c);
title('PID controller output');
xlabel('Time (s)');
xlabel('Output');
legend('Discrete response','Ideal response');
figure;
w=logspace(-2,3,10000);
bode(Gf,w); %aprox ts







