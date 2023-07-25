function q=draw_poses(poses,scale,axis,color)

if exist('axis') ~=1
    axis=[1 0 0];
end
if exist('color') ~=1
    color='b';
end

r=quatrotate(quatinv(poses(:,4:end)),axis./norm(axis));
poses(:,1:3)=round(poses(:,1:3),8);
r=round(r,8);
q=quiver3(poses(:,1),poses(:,2),poses(:,3),r(:,1),r(:,2),r(:,3),scale,color);

end



