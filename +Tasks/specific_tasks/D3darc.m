%%
x=linspace(-4,4,1000);
mu=0;
sigma=1.5
p = normpdf(x,mu,sigma);
p=0.15*p./max(p);
x=0.25*x./max(x);
p=0.006+p-min(p);
p=p(x<0);
x=x(x<0);
plot(x,p);
axis equal
f=@(xx) interp1(x,p,abs(xx)-0.25);


fv=@(xx) atan2((f(xx+0.0002)-f(xx-0.0002)),0.0004);

plot(x,f(x))
plot(x,fv(x))
f(0:0.01:0.24)
%%

nozzle=0.008;
ww=nozzle*1.5;
w=0.075;
%%
% pts=[ 0 0 0 ;0 1 0; 1 1 0; 1 0 0];
pts= [ linspace(0,2*pi,300)' sin(linspace(0,2*pi,300))' zeros(300,1)];
path= Tasks.GenericPath(pts);
path.plot()
% path.resample(0.01);
% path.smooth(0.05);
% path.resample(0.01);
% path.plot()
path.scale([0.4 0.1 1])
path.resample(0.01);

path.plot()

%%
xhatch=Tasks.XHatch1Path(ww,nozzle);
xhatch.resample(0.001)
xhatch.plot()

xhatch2=Tasks.XHatch2Path(ww,nozzle);
xhatch2.resample(0.001)
xhatch2.plot()
%%
path2=Tasks.GenericPath(flipud(path.path(1:end-1,:)));
path2.resample(0.0001);
path2.superimpose(xhatch);
path2.resample(0.001);
path2.plot();
path2.path(:,3)=f(path2.path(:,2)-0.125);
path2.resample(0.001);
path2.plot();
axis equal
%%
path3=Tasks.GenericPath((path.path(1:end-1,:)));
path3.resample(0.0001);
path3.superimpose(xhatch2);
path3.resample(0.001);
path3.plot();
path3.path(:,3)=f(path3.path(:,2)-0.125);
path3.resample(0.001);
path3.plot();
path3.path(:,3)=path3.path(:,3)+0.01;
axis equal
%%
ptss=[path2.path; path3.path];
path4=Tasks.GenericPath(ptss);
path4.resample(0.001);
path4.resample(0.0025);
path4.plot()
%%
T=path4.toTForm(path4,"zonly");
% T=TForm.tformX(T,TForm.DOWN);
% 
nn=size(T,3);
TD=TForm.DOWN;
T(1:3,1:3,:)=repmat(TD(1:3,1:3),1,1,nn);
% 
R=eul2rotm([zeros(nn,1) zeros(nn,1) -0.15+fv((path4.path(:,2)+0.125))], "ZYX");
RT=[R zeros(3,1,nn); zeros(1,3,nn) ones(1,1,nn)];
T=TForm.tformX(T,RT);
path4.plotTForm(T,0.01)

xlabel("x")
ylabel("y")
view(90,0)





