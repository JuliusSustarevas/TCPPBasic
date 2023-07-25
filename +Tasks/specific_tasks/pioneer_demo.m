nozzle=0.007;
ww=0.02;
%%
pts=[0.75 0 0; 0.75 -0.7 0;  0.75 0.7 0; 0.75 0 0;];
path1= Tasks.GenericPath(pts);
path1.resample(0.01);
path1.smooth(0.2);
path1.resample(0.01);
path1.plot()
axis equal

%%
upath=Tasks.UPath(0.03);
upath.scale([0.08 0.035 1]);
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

%% Base
pts=[0 0 0; 0 -0.1 -0.3236;0 0.1 0.3236; 0 0 0];
base_path= Tasks.GenericPath(pts);
base_path.resample(0.01);
base_path.path
length(base_path.path)
bx = interp1(linspace(1,length(path.s),length(base_path.path)),base_path.path(:,1),linspace(1,length(path.s),length(path.s)));
by = interp1(linspace(1,length(path.s),length(base_path.path)),base_path.path(:,2),linspace(1,length(path.s),length(path.s)));
bth = interp1(linspace(1,length(path.s),length(base_path.path)),base_path.path(:,3),linspace(1,length(path.s),length(path.s)));

%% Create Scenetask
scene_task.layers=[]
task_positions=squeeze(path.traj(1:3,4,:))';
task_orientations=rotm2quat(path.traj(1:3,1:3,:));
task_s=path.s;

clear layer
layer.task_positions=task_positions
layer.task_orientations=task_orientations
layer.task_s=task_s;
    
%Base 
layer.base_positions=[bx; by; by*0.0]';
layer.base_orientations=eul2quat([bth ;bth*0;bth*0]');
layer.base_elipses=ones(length(path.s),3).*[0.03 0.03 0.1];

scene_task.layers=[scene_task.layers layer];
scene_task.layers=[scene_task.layers layer];

save("pioneers2_solved","scene_task")
