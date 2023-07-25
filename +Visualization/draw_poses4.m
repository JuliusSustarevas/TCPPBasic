function q=draw_poses4(poses,scale,axis,color)
% same as draw poses but takes nx4 no quat

if exist('axis') ~=1
    axis=[1 0 0]';
end
if exist('color') ~=1
    color='b';
end
n=size(poses,1);
r=zeros(n,3);
R=eul2rotm([poses(:,4) zeros(n,2)]);
for ii =1:n
    r(ii,:)=R(:,:,ii)*axis;
end
poses(:,1:3)=round(poses(:,1:3),8);
r=round(r,8);
q=quiver3(poses(:,1),poses(:,2),poses(:,3),r(:,1),r(:,2),r(:,3),scale,color);

end



