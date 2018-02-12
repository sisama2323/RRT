% THIS SCRIPT CALCULATES THE QUARTERNION GIVEN EULER ANGLES
function quat = EulToQuat(angle)
    z = angle(1);
    y = angle(2);
    x = angle(3);
    quat = [cos(x/2)*cos(y/2)*cos(z/2)+sin(x/2)*sin(y/2)*sin(z/2);
            sin(x/2)*cos(y/2)*cos(z/2)-cos(x/2)*sin(y/2)*sin(z/2);
            cos(x/2)*sin(y/2)*cos(z/2)+sin(x/2)*cos(y/2)*sin(z/2);
            cos(x/2)*cos(y/2)*sin(z/2)-sin(x/2)*sin(y/2)*cos(z/2)];
end