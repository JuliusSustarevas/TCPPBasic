% load("+RM/data/as_rm.mat")
% vec=[0,0,-1]';
% r=eul2rotm([0.1,-0.2,0.1]);
% while true
%     vec=r*vec;
% 
%     [points, ris] = asrm.get_rm(vec');
%     b=(abs(points(:,3))<0.025) | (abs(points(:,2))<0.027);
% 
%     subplot(1,3,1)
%     quiver3(0,0,0,vec(1),vec(2),vec(3),0.3,'k','LineWidth',2);
%     hold on
%     quiver3(0,0,0,1,0,0,0.2,'r');
%     quiver3(0,0,0,0,1,0,0.2,'g');
%     quiver3(0,0,0,0,0,1,0.2,'b');
%     hold off
%     xlim([-1, 1]);
%     ylim([-1, 1]);
%     zlim([-1, 1]);
% 
%     subplot(1,3,2:3)
%     Visualization.draw_rm(points(b,:), ris(b,:))
%     xlim([-0.4, 1.2]);
%     ylim([-1, 1]);
%     zlim([-1, 1]);
%     
%     drawnow;
% end


% load("+RM/data/as_rm.mat")
% figure()
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

set(gca, 'CameraViewAngleMode', 'auto', 'CameraTargetMode', 'auto', ...
         'CameraPositionMode', 'manual');
vec=[0,0,-1]';
r=eul2rotm([0.1,-0.2,0.1]);

angle=0;
while true
    vec=r*vec;

    [points, ris] = asrm.get_rm(vec');
    b=(abs(points(:,3))<0.025) | (abs(points(:,2))<0.027);

%     subplot(1,3,1)
    quiver3(0,0,0,vec(1),vec(2),vec(3),0.3,'k','LineWidth',2);
    hold on
    quiver3(0,0,0,1,0,0,0.2,'r');
    quiver3(0,0,0,0,1,0,0.2,'g');
    quiver3(0,0,0,0,0,1,0.2,'b');
    
%     xlim([-1, 1]);
%     ylim([-1, 1]);
%     zlim([-1, 1]);

%     subplot(1,3,2:3)
    Visualization.draw_rm(points(b,:), ris(b,:))
    xlim([-0.4, 1.2]);
    ylim([-1, 1]);
    zlim([-1, 1]);
    axis equal
    hold off
        set(gca, 'CameraPosition', [3*sin(angle), 3*cos(angle), 3]);
%     pause(0.01);
angle=angle+0.05;
    drawnow;

end

