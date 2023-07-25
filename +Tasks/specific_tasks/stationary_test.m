nozzle=0.007;

ww=0.02;

r2=0.65;
r1=0.45;

%%
% pts=[ 0 0 0 ;0 1 0; 1 1 0; 1 0 0];
pts= [ cos(linspace(-pi/2,pi/2,300))' sin(linspace(-pi/2,pi/2,300))' zeros(300,1)];
path1= Tasks.GenericPath(pts);
path1.plot()
% path.resample(0.01);
% path.smooth(0.05);
% path.resample(0.01);
% path.plot()
path1.scale([r1 r1 1])
path1.resample(0.01);
path1.plot()

%%
xhatch=Tasks.XHatch1Path(ww,nozzle);
xhatch.resample(0.001)
xhatch.plot()
%%
path2=Tasks.GenericPath(flipud(pts));
path2.scale([r2 r2 1])
path2.resample(0.0001);
% path2.superimpose(xhatch);
path2.resample(0.001);
path2.plot()
%%
path1.resample(0.0001);
% path1.superimpose(xhatch);
path1.resample(0.001);
path1.plot()
%%
final_path=[path1.path; path2.path];
final_path(:,1)=final_path(:,1)+0.175;

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
%% generic save
% name="test_circles";
% save(name,"task");
% 

%%
% [file,path] = uigetfile(".yaml");
% task_name=strcat("tasks/",name);
% task_=task;
% clear task
% task.obj=fp;
% task.map_fn=file;
% task.map_path=path;
% task.num_layers=1;
% save(strcat(path,task_name),"task");
%% Test:
% [fn,path] = uigetfile();
% task_fn=strcat(path,fn);
% robot=RRT.AsRobot();
% % irm=RM.ASIRM();
% %%
% scene=RRT.Scene(task_fn,irm,robot);
