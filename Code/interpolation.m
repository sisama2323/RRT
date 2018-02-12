function angle=interpolation(eul1,eul2);
q1=EulToQuat([0 eul1 0]);
q1=q1';
q2=EulToQuat([0 eul2 0]);
q2=q2';

theta1=acos(q1*q2')/(norm(q1)*norm(q2));
angle=[];
for t=0:0.1:1
        SLERP=(q1*sin((1-t)*theta1/2)+q2*sin(t*theta1/2))/sin(theta1/2);
        Rot=quat2rot(SLERP);
        angle=[angle;acos((Rot(1,1)+Rot(2,2)+Rot(3,3)-1)/2)];
end
