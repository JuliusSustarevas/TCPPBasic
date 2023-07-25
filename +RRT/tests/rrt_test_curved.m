% clear all
% irm=RM.ASIRM();
%% select map
% [fn,path] = uigetfile();
% task_fn=strcat(path,fn);
%%
robot=RRT.AsRobot();

%%
scene=RRT.Scene(task_fn,irm,robot);
rrt=RRT.RRT(scene);
%% set properties
rrt.e_inc = 0.05;
rrt.e_reach = 0.3;
rrt.e_goal = 0.2;%when to start checking goal
rrt.start_rad = 3*rrt.e_reach;
rrt.s_var = 0.3;
rrt.num_start_nodes = 200;
rrt.draw = false;
% rrt.draw = true;
%%
scene.iriThreshold=0.5;%Solve with 0.5
path=rrt.rrtstar();
rrt.plot_path(path)
drawnow();
path_after_rrt1=cell2mat(vertcat(arrayfun(@(n) n.toVec4(), path, 'UniformOutput', false)));
rrt.extra_samples(3);%3x more samples to help smooth
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
path=RRT.teb_like_smooth(rrt,path,15,20);% will take a while but is worth it. 
rrt.plot_path(path,[0.6,0.2,0.2])
path_after_smooth=cell2mat(vertcat(arrayfun(@(n) n.toVec4(), path, 'UniformOutput', false)));
drawnow();
%%
% path_states.r1=path_after_rrt1;
% path_states.r2=path_after_rrt2;
% path_states.r3=path_after_skip;
% path_states.r4=path_after_smooth;
% save("pathstates","path_states");
%% put to m3dp_tcpp package format
% if planning is not finished?
% sm=path(end).pose(5);
% sb=scene.task.s<sm;
% scene.task.s=scene.task.s(sb);
% scene.task.traj=scene.task.traj(:,:,sb);
% scene.task.path=scene.task.path(sb,:);
%%
lz=0.005;
ln=1;
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


    layer.task_positions=task_positions;
    layer.task_orientations=task_orientations;
    layer.task_s=task_s;
    
    %Base 
    layer.base_positions=base_positions;
    layer.base_orientations=base_orientations;
    layer.base_elipses=base_elipses;
    
    scene_task.layers=[scene_task.layers layer];


%%
% assert(all(scene_task.base_elipses(:)>0))%There  should not be non-leeway
save("floorplan_solved","scene_task")

%%
%  Added s distance to  node distance. changed costs.  I think i need 2x in
%  the d cost.  also for these would be interesting to check kd tree  split
%  as function of time through search. also with multi layers.   that
%  should be affected by s being in costs. Do test tomorrow with proper
%  slam from ma_map. after low pass. and before just to checl. Also further
%  calibrate the base following.  also debug layer by layer.  rviz should
%  show now cuz less points?

%  VERY IMPORTANT AFTER IROS:   finding q near should include  costs not
%  just distance.  so do k rad either way.  not just in rewire. and then
%  sor for + cost.
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
