%%
x=linspace(0,1,1000);
p=(0.18*((sin(x*4*pi -pi*8/16))+1)/2);
myz=@(x) (0.15*((sin(x*4*pi -pi*8/16))+1)/2);
plot(x,p)
axis equal

%%
x=linspace(0,0.15,1000);
p=(0.25*(pi/2+asin((-2*(0.25*x/0.15)+0.25)*4))/pi);

plot(x,p)
axis equal

isin=@(x) (0.25*(pi/2+asin((-2*(0.25*x/0.15)+0.25)*4))/pi);
% isin(abs(y-0.5))
%%
lh=0.008;

sinptsf=@(n)[
-isin(n*lh) 0 n*lh;
isin(n*lh) 0 n*lh;
isin((n+1)*lh) 0 (n+1)*lh;
-isin((n+1)*lh) 0 (n+1)*lh;
];

sinpts=[
    sinptsf(0);
    sinptsf(2);
    sinptsf(4);
    sinptsf(6);
    sinptsf(8);
    sinptsf(10);
    sinptsf(12);
    sinptsf(14);
    sinptsf(16);
%     sinptsf(18);
%     sinptsf(20);
%     sinptsf(22);
    -isin(18*lh) 0 18*lh;
    isin(18*lh) 0 18*lh;
    ];

%%
path= Tasks.GenericPath(sinpts);
path.resample(0.01);
path.smooth(0.015);
path.resample(0.001);
path.plot()
axis equal
view(0,0)
%%
nozzle=0.01;
ww=nozzle*2;
w=0.075;
%%
xhatch=Tasks.XHatch1Path(ww,nozzle);
xhatch.resample(0.0001)
xhatch.plot()
%%
hill1=Tasks.GenericPath(path.path);
hill1.resample(0.0001);
hill1.superimpose(xhatch);
hill1.resample(0.001);
hill1=Tasks.GenericPath(hill1.path);
hill1.plot();
axis equal

%%
hill2=Tasks.GenericPath(hill1.path+[0.5 0 0]);
hold on
hill2.plot();
hold off
%%
x=linspace(0,1,1000)';

r=(pi/5)*sin((x)*4*pi);
R=eul2rotm([ zeros(1000,1) r zeros(1000,1)],'XYZ');
v=zeros(3,1,1000);
for ii=1:1000
    v(:,:,ii)=R(:,:,ii)*([0 0 0.01]');
end
curved_pts=[
    x zeros(1000,1) myz(x)+lh/2;
    ([flipud(x) zeros(1000,1) myz(flipud(x))]+squeeze(v)');
];

curved_pts=curved_pts+[-0.25 0 0];


clayer=Tasks.GenericPath(curved_pts);
clayer.resample(0.0001);
clayer.superimpose(xhatch);
clayer.resample(0.001);
clayer.plot();
view(0,0)
% axis equal
% xlim([0.4 0.6])
%%
hill1.resample(0.0025);
hill2.resample(0.0025);
clayer.resample(0.0025);
hill1.cumlen()
hill2.cumlen()
clayer.cumlen()
%%
axis equal
hill1.plot();
hold on
hill2.plot();
clayer.plot();
hold off
view(0,0)

hold on

clayer.plotTForm(T,0.2)
xlabel("x(m)")
zlabel("z(m)")
ylabel("y(m)")
ylim([-0.01 0.25])
view(0,0)
axis equal

set(gcf,'Color',[1 1 1])
set(gca,'FontSize',18)


%%



%%

r=-(pi/5)*sin((clayer.path(:,1)+0.25)*4*pi);
plot(clayer.path(:,1),r)

T=clayer.toTForm(clayer,"zonly");
% T=TForm.tformX(T,TForm.DOWN);
% % 
nn=size(T,3);
TD=TForm.DOWN;
T(1:3,1:3,:)=repmat(TD(1:3,1:3),1,1,nn);
% % 
R=eul2rotm([zeros(nn,1)  r zeros(nn,1) ], "ZYX");
RT=rotm2tform(R);
T=TForm.tformX(T,RT);
clayer.plotTForm(T,0.1)
clayer.traj=T;
clayer.s=clayer.gett(1);

xlabel("x")
ylabel("y")
view(0,0)
axis equal

%%
T=hill1.toTForm(hill1,"zonly");
nn=size(T,3);
TD=TForm.DOWN;
T(1:3,1:3,:)=repmat(TD(1:3,1:3),1,1,nn);
hill1.traj=T;
hill1.plotTForm(hill1.traj,0.1);
hill1.s=hill1.gett(1);


T=hill2.toTForm(hill2,"zonly");
nn=size(T,3);
TD=TForm.DOWN;
T(1:3,1:3,:)=repmat(TD(1:3,1:3),1,1,nn);
hill2.traj=T;
hill2.plotTForm(hill2.traj,0.1);
hill2.s=hill2.gett(1);

%%
% task=hill1;
% task=hill2;
% task=clayer;
% name="th_3";
% save(name,"task");
% 
% [file,path] = uigetfile(".yaml");
% task_name=strcat("tasks/",name);
% task_=task;
% clear task
% task.obj=task_;
% task.map_fn=file;
% task.map_path=path;
% task.num_layers=1;
% save(strcat(path,task_name),"task");




