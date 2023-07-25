asirm=RM.ASIRM()
%%
task_point=[1 1 0];
% task_point=[1 1 0.5];
vec=[0,0,-1]';
r=eul2rotm([0.1,-0.2,0.1]);

zz=0

while true
    vec=r*vec;
    task_point(3)=-0.1 +zz;
    zz=mod(zz+0.05,0.8);
    [points, iris]=asirm.get_irm_at(task_point,vec,45);    

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

%     subplot(1,3,2:3)
    quiver3(task_point(1),task_point(2),task_point(3),vec(1),vec(2),vec(3),0.2,'k','LineWidth',2);
    
    hold on
    quiver3(task_point(1),task_point(2),0,1,0,0,0.2,'r');
    quiver3(task_point(1),task_point(2),0,0,1,0,0.2,'g');
    quiver3(task_point(1),task_point(2),0,0,0,1,0.2,'b');
    xlim([task_point(1)+ (-1), task_point(1)+ (1)]);
    ylim([task_point(2)+ (-1), task_point(2)+ (1)]);
    zlim([-.5, 1.5]);
    xlabel("x")
    ylabel("y")
%     view(0,90);
    view(30,60);  
%     view(0,0);  
    drawnow;
    if sum(iris>0.5)==0
        hold off        
        continue
    end
    Visualization.draw_poses4(points(iris>0.5,:),0.75)
    hold off
  
    drawnow;
    disp("redrawed") 
end