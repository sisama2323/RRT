clear
q1=EulToQuat([0,0,0]);
q1=q1';
q2=EulToQuat([0,0,pi]);
q2=q2';
q3=EulToQuat([0,0,pi/2]);
q3=q3';
q4=EulToQuat([pi/2,pi/2,0]);
q4=q4';
theta1=acos(q1*q2')/(norm(q1)*norm(q2));
theta2=acos(q2*q3')/(norm(q2)*norm(q3));
theta3=acos(q3*q4')/(norm(q3)*norm(q4));
theta4=acos(q4*q1')/(norm(q4)*norm(q1));

SLERP1=[];
SLERP2=[];
SLERP3=[];
SLERP4=[];
for t=0:0.01:1
        SLERP1=[SLERP1;(q1*sin((1-t)*theta1/2)+q2*sin(t*theta1/2))/sin(theta1/2)];
        SLERP2=[SLERP2;(q2*sin((1-t)*theta2/2)+q3*sin(t*theta2/2))/sin(theta2/2)];
        SLERP3=[SLERP3;(q3*sin((1-t)*theta3/2)+q4*sin(t*theta3/2))/sin(theta3/2)];
        SLERP4=[SLERP4;(q4*sin((1-t)*theta4/2)+q1*sin(t*theta4/2))/sin(theta4/2)];
end

x=[1,0,0]';
y=[0,1,0]';
z=[0,0,1]';
for i=1:length(SLERP1)
    rm=quat2rot(SLERP1(i,:));  
    x1=rm*x;
    y1=rm*y;
    z1=rm*z;
    plot3([0,x1(1)],[0,x1(2)],[0,x1(3)],'r',[0,y1(1)],[0,y1(2)],[0,y1(3)],'g',[0,z1(1)],[0,z1(2)],[0,z1(3)],'b');
    grid on
    axis equal
    xlim([-1,1])
    ylim([-1,1])
    zlim([-1,1])
    drawnow
end

pause(1)

for i=1:length(SLERP2)
    rm=quat2rot(SLERP2(i,:));  
    x1=rm*x;
    y1=rm*y;
    z1=rm*z;
    plot3([0,x1(1)],[0,x1(2)],[0,x1(3)],'r',[0,y1(1)],[0,y1(2)],[0,y1(3)],'g',[0,z1(1)],[0,z1(2)],[0,z1(3)],'b');
    grid on
    axis equal
    xlim([-1,1])
    ylim([-1,1])
    zlim([-1,1])
    drawnow
end

pause(1)

for i=1:length(SLERP3)
    rm=quat2rot(SLERP3(i,:));  
    x1=rm*x;
    y1=rm*y;
    z1=rm*z;
    plot3([0,x1(1)],[0,x1(2)],[0,x1(3)],'r',[0,y1(1)],[0,y1(2)],[0,y1(3)],'g',[0,z1(1)],[0,z1(2)],[0,z1(3)],'b');
    grid on
    axis equal
    xlim([-1,1])
    ylim([-1,1])
    zlim([-1,1])
    drawnow
end

pause(1)

for i=1:length(SLERP4)
    rm=quat2rot(SLERP4(i,:));  
    x1=rm*x;
    y1=rm*y;
    z1=rm*z;
    plot3([0,x1(1)],[0,x1(2)],[0,x1(3)],'r',[0,y1(1)],[0,y1(2)],[0,y1(3)],'g',[0,z1(1)],[0,z1(2)],[0,z1(3)],'b');
    grid on
    axis equal
    xlim([-1,1])
    ylim([-1,1])
    zlim([-1,1])
    drawnow
end
















