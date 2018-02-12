function incollision=collision(q_start,q_end,obs,R);
P1 = generateArmPolygons(R, q_start, 1);
P2 = generateArmPolygons(R, q_end, 1);
if ~gjk2Darray(P1, obs) | ~gjk2Darray(P2, obs)
            incollision=1;
            return
else
            incollision=0;
end