clear; close all; clc;
%% ************************************************************************
x_0 = 0;
v1 = 100;
time1 = 200;
v2 = -100;
time2 = 300;
sigma_v = 1000;
T = 1;
H = [1 0];
F = [1 T;
     0 1];
G = [0.5*T^2; T]; 
x_k = [x_0; v1];
L_k =  0;
p_k = [0 0;
       0 0];
t = 1:T:time1+time2+1;
%%
figure
sub1 = subplot(4,1,1)
v = cat(2,v1*ones(1,200),v2*ones(1,300));
x = 0;
x_m = 0;
for i = 1:T:500
    x(end+1) = x(end) + T*v(i);
    x_m(end+1) = x(end) + normrnd(0,sigma_v);
end
plot(t, x,'b.', 'DisplayName', 'true path');
ylabel('x');
xlabel('time');
legend();

sub2 = subplot(4,1,2)
ax2 = plot(t, x_m,'b.', 'DisplayName', 'measurements');
ylabel('x');
xlabel('time');
legend();

for i = 1:T:500
    x_k(:,end+1) = F*x_k(:,end)+G*normrnd(0,1);
    p_k = F*p_k*F'+G*1*G';
    L_k = p_k*H'/(H*p_k*H'+sigma_v^2);
    x_k(:,end) = x_k(:,end)+L_k*(x_m(i)-H*x_k(:,end));
    p_k = p_k-L_k*H*p_k;
end

sub3 = subplot(4,1,3)
ax3 = plot(t, x_k(1,:),'b.', 'DisplayName', 'estimated path');
ylabel('x');
xlabel('time');
legend();

linkaxes([sub1,sub2,sub3],'x')

subplot(4,1,4)
hold on;
grid minor;
legend();
plot(abs(x_k(1,:)-x),'r', 'DisplayName', 'estimation error');
plot(abs(x_m-x),'g', 'DisplayName', 'measurement error');
ylabel('time');
xlabel('|\Delta x|');

% {
figure
subplot(2,1,1)
hold on
grid minor
ax1 = plot(1,x_k(1,1),'k.', 'DisplayName', 'estimation');
ax2 = plot(1, x(1),'r.', 'DisplayName', 'true path');
ax3 = plot(1, x_m(1),'b.', 'DisplayName', 'measurements');
legend([ax1, ax2, ax3], 'location', 'southeast');
subplot(2,1,2)
ax4 = plot(1, abs(x_k(1,1) - x(1)), 'k.', 'DisplayName', 'estimation error');
legend([ax4], 'location', 'southeast');
hold on
grid minor
% legend()
for i = 1:T:500
    subplot(2,1,1)
    ax1 = plot(i, x_k(1,i),'k*','HandleVisibility','off');
    ax2 = plot(i ,x(i),'r+','HandleVisibility','off');
    ax3 = plot(i ,x_m(i),'bo','HandleVisibility','off');
    subplot(2,1,2)
    ax4 = plot(i, abs(x_k(1,i) - x(i)), 'k*','HandleVisibility','off');
    pause(0.1);
    subplot(2,1,1)
    delete([ax1,ax2,ax3])
    plot(i, x_k(1,i),'k.','HandleVisibility','off');
    plot(i, x(i),'r.','HandleVisibility','off');
    plot(i, x_m(i),'b.','HandleVisibility','off');
    subplot(2,1,2)
    delete([ax4])
    plot(i, abs(x_k(1,i) - x(i)), 'k.','HandleVisibility','off');
end
%}