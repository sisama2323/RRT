function a=biasedrandom(max,min,bias1,weigth);
rnd=rand(1)*(max-min)+min;
mix=rand(1)*weigth;
a=rnd*(1-mix)+bias1*mix;
end

