function R1=rotation(q1,q2,q3);
Rz1=[cos(q1),sin(q1),0;-sin(q1),cos(q1),0;0,0,1];
Ry1=[cos(q2),0,sin(q2);0,1,0;-sin(q2),0,cos(q2)];
Rx1=[1,0,0;0,cos(q3),-sin(q3);0,sin(q3),cos(q3)];
R1=Rx1*Ry1*Rz1;
end