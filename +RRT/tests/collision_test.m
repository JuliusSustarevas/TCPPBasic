task_fn="test_scene.yaml";
robot=RRT.AsRobot();
irm=RM.ASIRM();
%%
scene=RRT.Scene(task_fn,irm,robot);
%%
c1=[10.07 9.17];%env collision
c2=[12.51 7.93];%task begining
c3=[17.92 11.09];%task end
%% all should be in collision
clear b
n1=RRT.Node(c1(1),c1(2),0,0);
b(1)=scene.isInCollision(n1);
n1=RRT.Node(c1(1),c1(2),0,scene.task_smax);
b(2)=scene.isInCollision(n1);
n1=RRT.Node(c1(1),c1(2),0,0.5);
b(3)=scene.isInCollision(n1);
assert(all(b))
%% should be free
clear b
n2=RRT.Node(c2(1),c2(2),0,0);
b(1)=scene.isInCollision(n2);
n3=RRT.Node(c3(1),c3(2),0,0);
b(2)=scene.isInCollision(n3);
n3=RRT.Node(c3(1),c3(2),0,0.5*scene.task_smax);
b(3)=scene.isInCollision(n3);
assert(all(b==false))
%% should be in collision
clear b
n2=RRT.Node(c2(1),c2(2),0,0.5*scene.task_smax);
b(1)=scene.isInCollision(n2);
n3=RRT.Node(c3(1),c3(2),0,scene.task_smax);
b(2)=scene.isInCollision(n3);
assert(all(b))
%% performance test
nt=100000;
tic
for ii=1:nt
cr=[(scene.map.XLocalLimits(1)+scene.map.XLocalLimits(1)*randn(2))  (scene.map.YLocalLimits(1)+scene.map.YLocalLimits(1)*randn(2))];
sr=randn(1)*scene.task_smax;
nr=RRT.Node(cr(1),cr(2),0,sr);
scene.isInCollision(nr);
end
toc/nt