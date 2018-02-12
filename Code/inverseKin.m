function q=inverseKin(R,x,y,i);
l1=R.a(1);
l2=R.a(2);
theta2=acos((x^2+y^2-l1^2-l2^2)/(2*l1*l2));
theta1=atan2(y,x)-asin(l2*sin(theta2)/sqrt(x^2+y^2));
if i==1
   q=[theta1,theta2];
elseif i==2
    q=[theta1,theta2];
    theta2=2*pi-theta2;
    theta1=atan2(y,x)-asin(l2*sin(theta2)/sqrt(x^2+y^2));
    q=[q;theta1,theta2];
end
