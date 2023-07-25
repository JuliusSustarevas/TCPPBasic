% asirm=RM.ASIRM()
%% task:
n=100;
l=3;
w=0.5;
x=linspace(0,l,n);
y=0.5*sin(2*pi*x./l);

z=0*x;
% z(:)=0.1;r=pi+(pi/2)*sin(2*pi*x./l)';
q=eul2quat([r, zeros(n,2)],'XYZ');
poses=[x; y; z;q']';


cla
% plot3(x,y,z)
Visualization.draw_poses(poses,0.2,[0 0 1],'b')
axis equal

%%
cla
set(gcf,'Color',[1 1 1])
set(gca,'Color',[1 1 1])
set(gca,'GridColor',[0 0 0])
set(gca,'MinorGridColor',[0 0 0])
set(gca,'XColor',[0.1 0.1 0.1])
set(gca,'YColor',[0 0 0])
set(gca,'ZColor',[0 0 0])

set(gca,'FontSize',18)
xlabel("x (m)")
ylabel("y (m)")
zlabel("z (m)")

view(0,0)
for ii=1:3:100
% cla
% plot3(x,y,z)
Visualization.draw_poses(poses,0.2,[0 0 1],'k')
axis equal
% hold on
% plot(poses(ii,1),poses(ii,2),'.',"MarkerSize",15)
% % ii=25;
% task_point=poses(ii,1:3);
% R=quat2rotm(poses(ii,4:end));
% vec=R(:,3)';
% [points, iris]=asirm.get_irm_at(task_point,vec,30);    
% 
% q=Visualization.draw_poses4(points(iris>0.5,:),0.75)
hold off
view(0,90)
xaxis([-1,4])
yaxis([-1,1])
drawnow;
break
 
end




%%
thres=30;
iri_thres=0.5;
cla
% plot3(x,y,z)
q=Visualization.draw_poses(poses,0.2,[0 0 1],'k')
set(q,"LineWidth",1.5)

hold on
ii=1;

q=Visualization.draw_poses(poses(ii,:),0.06,[0 0 1],'r')
set(q,"LineWidth",3)
task_point=poses(ii,1:3);
R=quat2rotm(poses(ii,4:end));
vec=R(:,3)';
[points, iris]=asirm.get_irm_at(task_point,vec,thres);
q=Visualization.draw_poses4(points(iris>iri_thres,:),0.75)



ii=75;

q=Visualization.draw_poses(poses(ii,:),0.06,[0 0 1],'r')
set(q,"LineWidth",3)
task_point=poses(ii,1:3);
R=quat2rotm(poses(ii,4:end));
vec=R(:,3)';
[points, iris]=asirm.get_irm_at(task_point,vec,thres);
q=Visualization.draw_poses4(points(iris>iri_thres,:),0.75)


axis equal
hold off
hold off
axis equal
xlabel("x(m)")
ylabel("y(m)")
xlim([-1 4])
ylim([-1 1])
set(gca,"FontSize",18)

caxis([min(N(:)) max(N(:))])
view(0,90)
colorbar
%%
xx=linspace(-1,4,100);
yy=linspace(-1,4,100);
[X,Y]=ndgrid(xx,yy);

samples=zeros(n*100,4);
mm=200;
for jj=1:n
    R=quat2rotm(poses(jj,4:end));
    vec=R(:,3)';   
    points=zeros(mm,4);
    for kk=1:mm
        [points4, bools, ~] = asirm.sample_irm_at(poses(jj,1:3), 0, 5);
        iris = asirm.validate_basepoint(points4, bools, vec);
        [~, k] = max(iris);
        points(kk,:) = points4(k(1), :);
    end
    
    samples((jj-1)*mm+1:jj*mm,:)=points;
end
%%
cla
q=Visualization.draw_poses(poses,0.2,[0 0 1],'k')
set(q,"LineWidth",1.5)
% 
hold on
% plot(samples(:,1),samples(:,2),'.')
% hold off

nbins=[round((max(samples(:,1))-min(samples(:,1)))/0.1) round((max(samples(:,2))-min(samples(:,2)))/0.1)];
% nbins=[50 50]

[N,C]=hist3([samples(:,1), samples(:,2)],nbins);
% hist3([samples(:,1), samples(:,2)],'CDataMode','auto')
% contourf(C{1},C{2},N)

[XX,YY]=ndgrid(C{1},C{2});

CC=1+floor(254*N./max(N(:)));
CC=reshape(CC,size(N,1)*size(N,2),1);
c=colormap('summer');
c(1,:)=[1 1 1];
CC=c(CC,:);
CC=reshape(CC,[size(N),3]);
NN=N;
NN(:)=-1;
s=surf(XX,YY,NN,CC,'edgecolor','none');

hold off
axis equal
xlabel("x(m)")
ylabel("y(m)")
xlim([-1 4])
ylim([-1 1])
set(gca,"FontSize",18)
colorbar
caxis([min(N(:)) max(N(:))])
view(0,90)


