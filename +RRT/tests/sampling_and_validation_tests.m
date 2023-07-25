task_fn="test_scene.yaml";
robot=RRT.AsRobot();
irm=RM.ASIRM();
%%
scene=RRT.Scene(task_fn,irm,robot);
%%
nn=100;
n=1;

for ii=1:nn
    scene.sample_valid_irm(s, n);
    isValid
    
end

%% performance