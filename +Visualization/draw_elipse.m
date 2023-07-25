function qs=draw_elipse(pose,elipse)
% same as draw poses but takes nx4 no quat



n=1;
xp=eul2rotm([pose(:,4) zeros(n,2)]) *[elipse(1) 0 0]';
xm=eul2rotm([pose(:,4) zeros(n,2)]) *[-elipse(1) 0 0]';

yp=eul2rotm([pose(:,4) zeros(n,2)]) *[0 elipse(2) 0]';
ym=eul2rotm([pose(:,4) zeros(n,2)]) *[0 -elipse(2) 0]';

zp=eul2rotm([pose(:,4) zeros(n,2)]) *[0 0 elipse(3)]';
zm=eul2rotm([pose(:,4) zeros(n,2)]) *[0 0 -elipse(3)]';

thp=eul2rotm([pose(:,4)+elipse(4) zeros(n,2)]) *[elipse(1) 0 0]';
thm=eul2rotm([pose(:,4)-elipse(4) zeros(n,2)]) *[elipse(1) 0 0]';

vecs=[xp xm yp ym zp zm thp thm]';
colors=[1 0 0;.5 0 0;0 0.5 0;0 0.5 0;0 0 0.5;0 0 0.5;.5 .5 0;.5 .5 0];

poses=repmat(pose,8,1);
vecs=round(vecs,8);

qs=cell(8,1);

for ii=1:8
    qs{ii}=quiver3(poses(ii,1),poses(ii,2),poses(ii,3),vecs(ii,1),vecs(ii,2),vecs(ii,3),0,'b','LineWidth',2);    
    qs{ii}.Color=colors(ii,:);    
end


end



