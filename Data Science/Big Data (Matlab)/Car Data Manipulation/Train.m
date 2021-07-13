function W = Train(x,y)

dev_x = x - Mean(x);
dev_y = y - Mean(y);
Sxy = dot(dev_x,dev_y);
Sxx = dot(dev_x,dev_x);

W1 = Sxy/Sxx;
W0 = Mean(y) - (W1*Mean(x));
W = [W0;W1];