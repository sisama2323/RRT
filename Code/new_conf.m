function A = new_conf(qr, qn, dis, step)
   qnew = [0 0];
   
   % Steer towards qn with maximum step size of eps
   if dis >= step
       qnew(1) = qn(1) + ((qr(1)-qn(1))*step)/dis;
       qnew(2) = qn(2) + ((qr(2)-qn(2))*step)/dis;
   else
       qnew(1) = qr(1);
       qnew(2) = qr(2);
   end   
   A = [qnew(1), qnew(2)];
end