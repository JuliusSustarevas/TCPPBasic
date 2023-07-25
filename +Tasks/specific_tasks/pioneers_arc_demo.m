nozzle=0.007;
ww=0.02;
%%
pts=[ 0 0 0 ;0 -1 0; -0.4 -1 0;0 -1 0; 0 0 0 ];
path1= Tasks.GenericPath(pts);
path1.resample(0.01);
path1.smooth(0.2);
path1.resample(0.01);
path1.plot()
axis equal

%%
upath=Tasks.UPath(0.03);
upath.scale([0.08 0.025 1]);
upath.resample(0.001);
upath.smooth(0.1);
upath.resample(0.001);
upath.plot();

%%
path1.resample(0.0001);
path1.superimpose(upath);
path1.resample(0.001);
path1.plot()
axis equal
path=path1;
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
% fp=path;
% name="pioneer1";
% save(name,"task");


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