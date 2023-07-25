load("+RM/data/as_rm.mat")
vec=[0,0,-1];

poses=asrm.poses(1:200,:);

b1=asrm.point2bools([0.8 0 0.3]);
poses1=poses(b1(1:200),:);

R=quat2rotm(poses(:,4:end));
Rvec=squeeze(R(:,3,:))';
b2=acosd(sum(Rvec.*repmat(vec,200,1),2))<45;
poses2=poses(b2,:);

poses3=poses(b2&(b1(1:200)'),:);


%%
cla
[x y z] = sphere(128);
h = surfl(x, y, z); 
set(h, 'FaceAlpha', 0.5)
shading interp
axis equal
set(gcf,"Color",[1 1 1])
hold on
q=Visualization.draw_poses(poses,0.2,[0 0 1],'b')
set(q,"LineWidth",1.5)
view(55,40)
xlabel("x")
ylabel("y")
zlabel("z")
set(gca,"FontSize",16)
%%
cla
[x y z] = sphere(128);
h = surfl(x, y, z); 
set(h, 'FaceAlpha', 0.5)
shading interp
axis equal
set(gcf,"Color",[1 1 1])
hold on
q=Visualization.draw_poses(poses1,0.2,[0 0 1],'b')
set(q,"LineWidth",1.5)
view(55,40)
xlabel("x")
ylabel("y")
zlabel("z")
set(gca,"FontSize",16)
%%
cla
[x y z] = sphere(128);
h = surfl(x, y, z); 
set(h, 'FaceAlpha', 0.5)
shading interp
axis equal
set(gcf,"Color",[1 1 1])
hold on
q=Visualization.draw_poses(poses2,0.2,[0 0 1],'b')
set(q,"LineWidth",1.5)
view(55,40)
xlabel("x")
ylabel("y")
zlabel("z")
set(gca,"FontSize",16)
%%
cla
[x y z] = sphere(128);
h = surfl(x, y, z); 
set(h, 'FaceAlpha', 0.5)
shading interp
axis equal
set(gcf,"Color",[1 1 1])
hold on
q=Visualization.draw_poses(poses3,0.2,[0 0 1],'b')
set(q,"LineWidth",1.5)
view(55,40)
xlabel("x")
ylabel("y")
zlabel("z")
set(gca,"FontSize",16)



