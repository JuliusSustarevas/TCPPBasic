nozzle=0.01;
ww=nozzle*4;
w=0.075;
%%
% pts=[ 0 0 0 ;0 1 0; 1 1 0; 1 0 0];
pts= [ cos(linspace(0,2*pi,300))' sin(linspace(0,2*pi,300))' zeros(300,1)];
path= Tasks.GenericPath(pts);
path.plot()
% path.resample(0.01);
% path.smooth(0.05);
% path.resample(0.01);
% path.plot()
path.scale([0.8 0.8 1])
path.resample(0.01);
path.plot()


%%
xhatch=Tasks.XHatch1Path(ww,nozzle);
xhatch.resample(0.001)
xhatch.plot()
%%
xhatch2=Tasks.XHatch2Path(ww,nozzle);
xhatch2.resample(0.001)
xhatch2.plot()
%%
path2=Tasks.GenericPath(path.path);
path2.resample(0.0001);
path2.superimpose(xhatch2);
path2.resample(0.001);
path2.plot()
%%
path.resample(0.0001);
path.superimpose(xhatch);
path.resample(0.001);
path.plot()
%%
layers=2;
paths=[path; path2];
z=0;
final_path=[];
for ii=1:layers
    final_path=[final_path; paths(mod(ii,2)+1).path  + [0 0 z]];
    z=z+0.005;
end

fp=Tasks.GenericPath(final_path);
fp.resample(0.0025);
fp.plot()
fp.traj=TForm.tformX(fp.toTForm(fp),TForm.DOWN());
td=TForm.DOWN();
for ii=1:size(fp.traj,3)
    if fp.traj(3,3,ii)>-0.99
        fp.traj(1:3,1:3,ii)=td(1:3,1:3); 
    end   
end            
fp.s=fp.gett(1);
fp.plot();
task=fp;
axis equal
est_vol=fp.s(end)*pi*((nozzle/2)^2)/(0.1^3)
%%
name="spiral_r08_28cm_2layers";
save(name,"task");

%%
[file,path] = uigetfile(".yaml");
task_name=strcat("tasks/",name);
task_=task;
clear task
task.obj=fp;
task.map_fn=file;
task.map_path=path;
task.num_layers=layers;
save(strcat(path,task_name),"task");
%% Test:
% [fn,path] = uigetfile();
% task_fn=strcat(path,fn);
% robot=RRT.AsRobot();
% % irm=RM.ASIRM();
% %%
% scene=RRT.Scene(task_fn,irm,robot);
