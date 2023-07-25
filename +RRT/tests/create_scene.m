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
task_name="tasks/trapezoid";
show(map)
task.obj=Tasks.DrawnPath();
task.map_fn=file;
task.map_path=path;
save(strcat(path,task_name),"task");
%% Test:
[fn,path] = uigetfile();
task_fn=strcat(path,fn);
robot=RRT.AsRobot();
% irm=RM.ASIRM();
%%
scene=RRT.Scene(task_fn,irm,robot);