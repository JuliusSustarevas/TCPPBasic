% clear all
[file,path] = uigetfile(".yaml");
%%
env_data = Imports.ReadYaml(strcat(path, file));
image = imread(strcat(path, env_data.image));
imageNorm = double(image) / 255;
imageOccupancy = 1 - imageNorm;
imageOccupancy(imageOccupancy < env_data.free_thresh) = 0;
imageOccupancy(imageOccupancy > env_data.free_thresh) = 1;
imageOccupancy=imdilate(imageOccupancy,ones(3));%Dialation

map = occupancyMap(imageOccupancy, (1 / env_data.resolution));
map.FreeThreshold = 0;
map.OccupiedThreshold = 1;
map.GridLocationInWorld=env_data.origin(1:2);
%%
task_name="tasks/1m_10layers_crosshatch";
show(map)
% task.obj=Tasks.MultiLayerDrawnTask(20,0.005);
task.obj=Tasks.XHatchMultiLayerDrawnTask(20,0.005,0.05,0.007);
task.map_fn=file;
task.map_path=path;
% task.obj.plot()
plot(task.obj.path(:,1),task.obj.path(:,2))
hold on
task.obj.plot()
hold off
% save(strcat(path,task_name),"task");
%% Test:
% [fn,path] = uigetfile();
% task_fn=strcat(path,fn);
% robot=RRT.AsRobot();
% irm=RM.ASIRM();
%%
% scene=RRT.Scene(task_fn,irm,robot);