nozzle=0.008;
ww=0.03;
w=1;
l=2;
%%
% pts=[ w/2 0 0 ;w 0 0; (ww+w/2) l/2 0; w l 0; 0 l 0 ;  (-ww+w/2) l/2 0 ; 0 0 0;  w/2 0 0;];
% pts=[ 0.25 0 0 ; 0.5 0 0; 0.45 0.55 0; 1 0.5 0; 1 1 0 ;  0.5 1 0; 0.55 0.45 0; 0 0.5 0 ; 0 0 0; 0.25 0 0];
a=0.15;b=-0.086;
pts=[  0.5 (0.5-b) 0; (1-a) 0 0; 1 0 0 ;  1 1 0; (1-a) 1 0 ; 0.5 (0.5+b) 0 ; a 1 0; 0 1 0; 0 0 0; a 0 0; 0.5 (0.5-b) 0];
% pts=(eul2rotm([-pi/4 0 0],"ZYX")*(pts'))';

path= Tasks.GenericPath(pts);
path.resample(0.001);
path.plot()
path.smooth(0.1);
path.resample(0.001);
path.path=[path.path; path.path(1,:)];
path.plot()
path.scale([2 1 1])
path.resample(0.001);




path.plot()
axis equal
%%
xhatch=Tasks.XHatch1Path(ww,nozzle);
xhatch.resample(0.001)
xhatch.plot()
%%
xhatch2=Tasks.XHatch2Path(ww,nozzle);
xhatch2.resample(0.001)
xhatch2.plot()
%%
path1=Tasks.GenericPath(path.path)
path1.resample(0.0001);
path1.superimpose(xhatch);
path1.resample(0.001);
% path1.path=path1.path(1:end-8,:);
path1.plot()
%%
path2=Tasks.GenericPath(path.path)
path2.resample(0.0001);
path2.superimpose(xhatch2);
path2.resample(0.001);
% path2.path=path2.path(1:end-8,:);
path2.plot()
%%
cla
plot(path1.path(:,1),path1.path(:,2))
hold on
plot(path2.path(:,1),path2.path(:,2))
hold off
axis equal


%%
layers=3;
paths=[path1; path2];
z=0;
final_path=[];
for ii=1:layers
    final_path=[final_path; paths(mod(ii,2)+1).path  + [0 0 z]];
    z=z+0.006;
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
sss=path1.cumlen();
est_vol=sss(end)*pi*((0.01/2)^2)/(0.1^3)
est_vol=sss(end)*pi*((0.006/2)^2)/(0.1^3)
est_vol=sss(end)*pi*((0.007/2)^2)/(0.1^3)
layer_time=(sss(end)/0.04)/60
%% generic save
name="eight_2x1_w6_3l";
% save(name,"task");


%%
[file,path] = uigetfile(".yaml");
task_name=strcat("tasks/",name);
task_=task;
clear task
task.obj=fp;
task.map_fn=file;
task.map_path=path;
task.num_layers=layers;
% save(strcat(path,task_name),"task");
%% Test:
% [fn,path] = uigetfile();
% task_fn=strcat(path,fn);
% robot=RRT.AsRobot();
% % irm=RM.ASIRM();
% %%
% scene=RRT.Scene(task_fn,irm,robot);
