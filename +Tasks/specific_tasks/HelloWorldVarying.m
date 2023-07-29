nozzle=0.006;
ww=0.02;
%%
x=linspace(0,4*pi,100);
pts=[x;sin(x);x.*0 + 0.05]';


%%
path= Tasks.GenericPath(pts);
path.scale([5 1 1]);
path.resample(0.01);
path.smooth(0.005);
path.resample(0.002);
path.plot()
axis equal
%%
upath=Tasks.UPath(0.03);
upath.scale([0.15 0.03 1]);
upath.resample(0.01);
upath.resample(0.001);
upath.plot();

%%
path.superimpose(upath);
path.resample(0.005);

path.plot();

view(0,90)
axis equal

%% Create constant orientation
T=path.toTForm(path,"zonly");
nn=size(T,3);
TD=TForm.DOWN;
T(1:3,1:3,:)=repmat(TD(1:3,1:3),1,1,nn);
path.traj=T;
path.plotTForm(path.traj,0.1);
path.s=path.gett(1);

%% Save to file
task=path;
fp=path;
name="hello_world_varying_orientation";
%save(name,"task");


% %%
[file,path] = uigetfile(".yaml","Select Map File m3dp_scenes/testworld/");
task_dir=strcat(path,"tasks");
task_name=strcat(task_dir,filesep, name);
task_=task;
clear task
task.obj=fp;
task.map_fn=file;
task.map_path=path;
task.num_layers=1;
save(task_name,"task");
