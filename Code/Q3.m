clear;clc
startup_rvc
mdl_twolink


xdot0=[-2,0,0];
x0=[sqrt(3),0,1];
qi=inverseKin(twolink,sqrt(3),1,1);
dq=[0.5 0];

xdotf=[0,0,0];
xf=[-0.25,0,0.25];
qf=inverseKin(twolink,xf(1),xf(3),2);
qf=wrapToPi(qf(1,:));

A=[];
b=[];
Aeq=[];
beq=[];
lb=ones(20,2)*(-pi);
ub=ones(20,2)*pi;


% q0=[wrapToPi(linspace(qi(1),wrapTo2Pi(qf(1)),20))',wrapToPi(linspace(qi(2),wrapTo2Pi(qf(2)),20))'];
% q0=[qi;rand(18,2)*2*pi-pi;qf];
% q0=unifrnd(-pi,pi,[20,2]);
q0=[ones(20,1)*qi(1),ones(20,1)*qi(2)];

options = optimoptions(@fmincon,'Algorithm','interior-point','MaxFunEvals',...
    10000,'TolCon',1.0000e-05,'TolFun',1.0000e-05,'Display','iter');
q_result=fmincon(@obj,q0,A,b,Aeq,beq,lb,ub,@const,options);

%%
function J=obj(q);

qd=diff(q)/(2/19);
qdd=diff(qd)/(2/19);
x=[];
for i=1:length(q)
    x=[x;fkine(q(i,1),q(i,2))];
end
dx=diff(x)/(2/19);
ddx=diff(dx)/(2/19);

qddsum=[];
ddxsum=[];
for i=1:length(qdd)
    qddsum=[qddsum;(norm(qdd(i,:)))^2];
    ddxsum=[ddxsum;(norm(ddx(i,:)))^2];
end
J=sum(0.5*qddsum+ddxsum)*(2/19);
disp(J)
end


%% 
function [c,ceq]=const(q);
c=[];
x=[];
for i=1:length(q)
    x=[x;fkine(q(i,1),q(i,2))];
end
dx=diff(x)/(2/19);
ceq=zeros(4,2);
ceq(1,1:2)=x(1,1:2)-[sqrt(3),1];
ceq(2,1:2)=x(end,1:2)-[-0.25,0.25];
% ceq(1,1:2)=q(1,1:2)-[0.5236, 0.0000];
% ceq(2,1:2)=q(end,1:2)-[0.9631,  2.7862];
% ceq(2,1:2)=q(end,1:2)-[-2.5339, -2.7862];

ceq(3,1:2)=dx(1,1:2)-[-2,0];
ceq(4,1:2)=dx(end,1:2)-[0,0];
end




%% fmincon
% % first attempt theta1 and theta2 is a list of 20 number
% dt=2/20;
% x=[sqrt(3),1];
% dx=[-2,0];
% q0=[0.5236, 0];
% dq=[0.5 0];
% xq_know=[x,dx,q0,dq];
% A=[];
% b=[];
% Aeq=[];
% beq=[];
% lb=[-pi,-pi];
% ub=[pi,pi];
% J=0;
% for i=1:20
%     fun=@(q)((norm(((fkine(q(1),q(2))-xq_know(i,1:2))./dt-xq_know(i,3:4))./dt))^2 ... 
%     +0.5*(norm((([q(1),q(2)]-xq_know(i,5:6))./dt-xq_know(i,7:8))./dt))^2)*dt;
%     q0=xq_know(i,5:6);
%     dx=(fkine(q(1),q(2))-xq_know(i,1:2))./dt;
%     dq=([q(1),q(2)]-xq_know(i,5:6))./dt;
%     xq_know = [xq_know; fkine(q(1),q(2)),dx,q,dq]; 
%     J=J+((norm(((fkine(q(1),q(2))-xq_know(i,1:2))./dt-xq_know(i,3:4))./dt))^2 ... 
%     +0.5*(norm((([q(1),q(2)]-xq_know(i,5:6))./dt-xq_know(i,7:8))./dt))^2)*dt;
% end
% 
% q=fmincon(fun,q0,A,b,Aeq,beq,lb,ub);

% 
% twolink.plot(q_result)

figure
anim = Animate('catch ball');
for i=1:length(q_result)
    twolink.plot(q_result(i,:))
    pause(0.5)
    anim.add();
end

x_z=twolink.fkine(q_result);
x=[];
z=[];
for i=1:20
    x=[x,x_z(1,4,i)];
    z=[z,x_z(3,4,i)];
end

figure
plot(x,z)
xlabel('x')
ylabel('z')
ylim([0,1.5])
title('end-effector position')

dx=diff(x)/(2/19);
dz=diff(z)/(2/19);
figure
plot(1:length(dx),dx)
hold on
plot(1:length(dx),dz)
legend('dx','dz')
xlabel('time')
title('end-effector velocity')

figure
plot(1:length(q_result),q_result')
legend('q1','q2')
xlabel('time')
title('joint angle')

qd=diff(q_result)/(2/19);
figure
plot(1:length(qd),qd')
xlabel('time')
legend('dq1','dq2')
title('joint angle velocity')





