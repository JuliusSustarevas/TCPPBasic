nozzle=0.006;
ww=0.02;
%%
pts=[0 0 0;
    0 -400 0;
    200 -400 0;
    200 (-400 +131.5) 0;
    200 (-400 +131.5) 1;
    200 -400 1;
    200 -400 0;
    400 -400 0;
    400 -160 0;
    196 -160 0;
    196 -160 1;
    (400-160) -160 1;
    (400-160) -160 0;
    (400-160) (-160+37) 0;
    (400-160) (-160+37) 1;
    (400-160) -160 1;
    400 -160 1;
    400 -160 0;
    400 0 0;
    0 0 0;
%     0 0 1;
%     0 -400 1;
%     400 -400 1;
%     400 0 1;
%     0 0 1;
    ];
%%
pts(:,1)=(pts(:,1)./400)*3.5;
pts(:,2)=(pts(:,2)./400)*3.5;
pts(:,3)=pts(:,3)*0.01;
%%
path= Tasks.GenericPath(pts);
path.resample(0.01);
path.smooth(0.005);
path.resample(0.002);
path.plot()
axis equal
%%
upath=Tasks.UPath(0.03);
upath.scale([0.2 0.05 1]);
upath.resample(0.01);
upath.resample(0.001);
upath.plot();

%%
path.superimpose(upath);

% b=abs(path.path(:,3))<0.001 | abs(path.path(:,3)-0.01)<0.001;
% path.path=path.path(b,:);
% path.resample(0.0025);
% path.plot();


b1=path.path(:,3)<0.005;
b2=path.path(:,3)>=0.005;



path.path(b1,3)=0;
path.path(b2,3)=0.01;

path.resample(0.0025);

path.plot();

view(0,90)
axis equal

%%
T=path.toTForm(path,"zonly");
nn=size(T,3);
TD=TForm.DOWN;
T(1:3,1:3,:)=repmat(TD(1:3,1:3),1,1,nn);
path.traj=T;
path.plotTForm(path.traj,0.1);
path.s=path.gett(1);

%%

% task=path;
% name="floorplan";
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
