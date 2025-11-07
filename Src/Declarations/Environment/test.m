[X, Y]=meshgrid(1:1:5,1:1:5);
f=X+Y;
[X Y Z]=xyz2grid('wasserfallenmapdata.xyz');
X=X-2648540;
Y=Y-1195050;

surf(X,Y,Z,'edgecolor','none')
