% clear all
% irm=RM.ASIRM();
%% select map
[fn,path] = uigetfile();
task_fn=strcat(path,fn);
%%
robot=RRT.AsRobot();

%%
scene=RRT.Scene(task_fn,irm,robot);

%%¬

s_weights=[0.1 0.25 0.5 0.75 1.0];

scene.iriThreshold=0.2;%Solve with 0.5
all_times=[];

for kl =1:5
global s_w
s_w=s_weights(kl);

    times=[];
%     pathss={};
    for lk=1:25
        rrt=RRT.RRT(scene);
        %% set properties
        rrt.e_inc = 0.05;
        rrt.e_reach = 0.4;
        rrt.e_goal = 0.1;%when to start checking goal
        rrt.start_rad = 1*rrt.e_reach;
        rrt.s_var = 0.15;
        rrt.num_start_nodes = 50;
        rrt.draw = false;
        % rrt.draw = true;
        tic
        path=rrt.rrtstar();
        times=[times toc];
%         pathss{lk}=vertcat(path.pose);
    end
    all_times=[all_times;times];
end

% save("big_nonplanar0","pathss")

% save("small_floor_plan_varying_node_s_irm03","all_times")
save("3circles_varying_ws_irm02","all_times")
% beep
% save("times_small_plan_irm_06","times")
rrt.plot_path(path)
% all_times1=all_times
% all_times2=all_times
% all_times=[all_times1; all_times2];
% 
% [mean(all_times,2) std(all_times,0,2)]
% all_times=[]
% 
% all_times=[all_times; times]
% 
% [mean(all_times,2) std(all_times,0,2)]
% 
% [mean(times,2) std(times,0,2)]

% [min(all_times,[],2) mean(all_times,2) std(all_times,0,2) max(all_times,[],2) ]




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

%%

% set(gcf,"Color",[1 1 1])
% set(gca,"FontSize",18)
