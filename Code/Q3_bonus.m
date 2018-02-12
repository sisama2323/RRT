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
lb=ones(30,2)*(-pi);
ub=ones(30,2)*pi;


% q0=[wrapToPi(linspace(qi(1),wrapTo2Pi(qf(1)),20))',wrapToPi(linspace(qi(2),wrapTo2Pi(qf(2)),20))'];
% q0=[qi;rand(18,2)*2*pi-pi;qf];
% q0=unifrnd(-pi,pi,[20,2]);
q0=[ones(30,1)*qf(1),ones(30,1)*qf(2)];

options = optimoptions(@fmincon,'Algorithm','interior-point','MaxFunEvals',...
    10000,'TolCon',1.0000e-05,'TolFun',1.0000e-05,'Display','iter');
q_result=fmincon(@obj,q0,A,b,Aeq,beq,lb,ub,@const,options);

%%
function J=obj(q);

qdd=diff(q,2)/(3/28);
x=[];
for i=1:length(q)
    x=[x;fkine(q(i,1),q(i,2))];
end

ddx=diff(x,2)/(3/28);

qddsum=[];
ddxsum=[];
for i=1:length(qdd)
    qddsum=[qddsum;(norm(qdd(i,:)))^2];
    ddxsum=[ddxsum;(norm(ddx(i,:)))^2];
end
J=sum(0.5*qddsum+ddxsum)*(3/28);

end


%% 
function [c,ceq]=const(q);
c=[];
x=[];
for i=1:length(q)
    x=[x;fkine(q(i,1),q(i,2))];
end
dx=diff(x)/(3/28);
ceq=zeros(6,2);
ceq(1,1:2)=x(1,1:2)-[-0.25,0.25];
ceq(2,1:2)=x(21,1:2)-[sqrt(3),1];
ceq(3,1:2)=x(end,1:2)-[-0.25,0.25];
ceq(4,1:2)=dx(1,1:2)-[0,0];
ceq(5,1:2)=dx(21,1:2)-[-2,0];
ceq(6,1:2)=dx(end,1:2)-[0,0];
end



%%
% twolink.plot(q_result)

% figure
% anim = Animate('catch ball');
% for i=1:length(q_result)
%     twolink.plot(q_result(i,:))
%     pause(0.5)
%     anim.add();
% end
% 
% x_z=twolink.fkine(q_result);
% x=[];
% z=[];
% for i=1:20
%     x=[x,x_z(1,4,i)];
%     z=[z,x_z(3,4,i)];
% end
% 
% figure
% plot(x,z)
% xlabel('x')
% ylabel('z')
% ylim([0,1.5])
% title('end-effector position')
% 
% dx=diff(x)/(2/19);
% dz=diff(z)/(2/19);
% figure
% plot(1:length(dx),dx)
% hold on
% plot(1:length(dx),dz)
% legend('dx','dz')
% xlabel('time')
% title('end-effector velocity')
% 
% figure
% plot(1:length(q_result),q_result')
% legend('q1','q2')
% xlabel('time')
% title('joint angle')
% 
% qd=diff(q_result)/(2/19);
% figure
% plot(1:length(qd),qd')
% xlabel('time')
% legend('dq1','dq2')
% title('joint angle velocity')
% 
% 
% 
% 

