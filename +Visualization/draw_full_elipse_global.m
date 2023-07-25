function qs=draw_full_elipse_global(pose,elipse)
% same as draw poses but takes nx3 no quat



n=1;
xp=[elipse(1) 0 0]';
xm=[-elipse(1) 0 0]';

yp=[0 elipse(2) 0]';
ym=[0 -elipse(2) 0]';

thp=[0 0 elipse(3) ]';
thm=[0 0 -elipse(3) ]';


vecs=[xp xm yp ym  thp thm]';
colors=[1 0 0;.5 0 0;0 0.5 0;0 0.5 0;.5 .5 0;.5 .5 0];

poses=repmat(pose,6,1);
vecs=round(vecs,6);

qs=cell(6,1);

for ii=1:6
    qs{ii}=quiver3(poses(ii,1),poses(ii,2),poses(ii,3),vecs(ii,1),vecs(ii,2),vecs(ii,3),0,'b','LineWidth',2);    
    qs{ii}.Color=colors(ii,:);    
end

end



