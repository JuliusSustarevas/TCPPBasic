% This script is the main entry point to solving TCPP problem. It loads up a task/map, sets up the scene and RRT. Solves. Does some smoothing (You dont need todo that much, this is illustrative.) And then puts the solution into  a data structure that can lateer be sent to the robot via m3dp_tasker
%%
% clear all

% I keep this commented out cuz it takees forever to    
% load. but you need to load it once
if (exist('irm', 'var') == false)
    disp('Loading the IRM file. This will take some time...')
    irm=RM.ASIRM(); 
end
%% select Task/Map
[fn,path] = uigetfile(); % Navigate to m3dp_scenes/test_world/tasks/ and pick something
task_fn=strcat(path,fn);
%%
robot=RRT.AsRobot();

%%
scene=RRT.Scene(task_fn,irm,robot);
rrt=RRT.RRT(scene);
drawnow;
%% set properties
rrt.e_inc = 0.05;
rrt.e_reach = 0.3;
rrt.e_goal = 0.2; %when to start checking if we can reach goal
rrt.start_rad = 2*rrt.e_reach; % rewire radius
rrt.s_var = 0.3;
rrt.num_start_nodes = 200;
% rrt.draw = false; 
rrt.draw = true; % !!!! Set to true if you want to see animation of pathplanning

%%
scene.iriThreshold=0.4;% what IRI counts as good?
path=rrt.rrtstar();
rrt.plot_path(path)
drawnow();
path_after_rrt1=cell2mat(vertcat(arrayfun(@(n) n.toVec4(), path, 'UniformOutput', false)));
rrt.extra_samples(2);%3x more samples to help smooth
path=rrt.trace_path(path(end));
rrt.plot_path(path,[0.5,0.5,0.9])
path_after_rrt2=cell2mat(vertcat(arrayfun(@(n) n.toVec4(), path, 'UniformOutput', false)));
drawnow();
%%
path=RRT.skip_smooth(rrt, path, 20);
rrt.plot_path(path,[0.2,0.5,0.2])
path_after_skip=cell2mat(vertcat(arrayfun(@(n) n.toVec4(), path, 'UniformOutput', false)));
drawnow();
%%
path=RRT.teb_like_smooth(rrt,path,10,15);% will take a while but is worth it. 
rrt.plot_path(path,[0.6,0.2,0.2])
path_after_smooth=cell2mat(vertcat(arrayfun(@(n) n.toVec4(), path, 'UniformOutput', false)));
drawnow();

%% Return here if you're just planning. 
return
%% put to m3dp_tcpp package format
%%
lz=0.105;
ln=scene.num_layers;
get_layer=@(li) squeeze(lz*(li-1.5) <= scene.task.traj(3,4,:)  &   scene.task.traj(3,4,:) <= lz*(li-0.5));


%%
n=size(scene.task.traj,3);
scene_task.layers=[];

    %Task
task_positions=squeeze(scene.task.traj(1:3,4,:))';
task_orientations=rotm2quat(scene.task.traj(1:3,1:3,:));
task_s=scene.task.s;

% base poses
base_poses_=cell2mat(vertcat(arrayfun(@(n) n.toVec4(), path, 'UniformOutput', false)));

% Comp elipses
scene.iriThreshold=0.3;% elipses 0.2
base_elipses=rrt.basepoints2elipses_global(base_poses_);

%Interp base path
base_poses=zeros(n,4);
base_poses(:,1)=interp1(base_poses_(:,4),base_poses_(:,1),task_s);
base_poses(:,2)=interp1(base_poses_(:,4),base_poses_(:,2),task_s);
base_poses(:,3)=wrapToPi(interp1(base_poses_(:,4), unwrap(base_poses_(:,3)),task_s));
base_poses(:,4)=task_s;
%Interp elipses
elipses=zeros(n,3);
elipses(:,1)=interp1(base_poses_(:,4),base_elipses(:,1),task_s);
elipses(:,2)=interp1(base_poses_(:,4),base_elipses(:,2),task_s);
elipses(:,3)=interp1(base_poses_(:,4),base_elipses(:,3),task_s);

%Base 
base_positions=[base_poses(:,1:2) zeros(n,1)];
base_orientations=eul2quat([base_poses(:,3) zeros(n,2)]);
base_elipses=elipses;

for ii=1:ln    

    layer.task_positions=task_positions(get_layer(ii),:);
    layer.task_orientations=task_orientations(get_layer(ii),:);
    layer.task_s=task_s(get_layer(ii),:);
    
    %Base 
    layer.base_positions=base_positions(get_layer(ii),:);
    layer.base_orientations=base_orientations(get_layer(ii),:);
    layer.base_elipses=base_elipses(get_layer(ii),:);
    
    scene_task.layers=[scene_task.layers layer];
end


%% make single laayer fromt ask
% layer.task_positions=task.obj.path;
% layer.task_orientations=rotm2quat(task.obj.traj(1:3,1:3,:));
% layer.task_s=task.obj.s;
% nnn=length(task.obj.s);
% %Base 
% layer.base_positions=zeros(nnn,3);
% layer.base_orientations=repmat([1 0 0 0],nnn,1);
% layer.base_elipses=zeros(nnn,3);
% scene_task.layers=[layer,layer];
