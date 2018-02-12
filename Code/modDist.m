function distance=modDist(x,y)
distance=norm(min(mod(x-y,360),mod(y-x,360)));