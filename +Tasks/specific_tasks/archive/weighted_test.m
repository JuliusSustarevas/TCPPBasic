nozzle=0.006;
ww=0.02;
%%
pts=[0 0 0; 1 0 0; 1 1 0; 0 1 0; 
    0 0 0; 1 0 0; 1 1 0; 0 1 0;
    0 0 0; 1 0 0; 1 1 0; 0 1 0;0 0 0;];


%%
path= Tasks.GenericPath(pts);
path.resample(0.01);
path.smooth(0.05);
path.resample(0.002);
path.plot()
axis equal
view(0,90)
axis equal
%%
upath=Tasks.UPath(0.03);
upath.scale([0.19 0.05 1]);
upath.resample(0.01);
upath.resample(0.001);
upath.plot();

%%
path.superimpose(upath);

% b=abs(path.path(:,3))<0.001 | abs(path.path(:,3)-0.01)<0.001;
% path.path=path.path(b,:);
% path.resample(0.0025);
% path.plot();

path.resample(0.005);

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
task=path;
fp=path;
name="weighted_circle";
save(name,"task");


%%
[file,path] = uigetfile(".yaml");
task_name=strcat("tasks/",name);
task_=task;
clear task
task.obj=fp;
task.map_fn=file;
task.map_path=path;
task.num_layers=1;
save(strcat(path,task_name),"task");
