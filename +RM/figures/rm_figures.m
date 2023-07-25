load("+RM/data/as_rm.mat")
vec=[0,0,-1]';
r=eul2rotm([0.1,-0.2,0.1]);

  

    [points, ris] = asrm.get_rm(vec');
    b=abs(points(:,3))<0.025;
    

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
    set(gcf,"Color",[1 1 1])
    subplot(1,2,1)  
    
    Visualization.draw_rm(points(b,:), ris(b,:))
    
    
    set(gca,"FontSize",16)
    view(0,90)    
    grid(gca,'minor')
    axis equal
    title("Ground level")
    xlabel("x(m)")
    ylabel("y(m)")
    colorbar
    xlim([-0.6, 1.2]);
    ylim([-1, 1]);
    zlim([-1, 1]);
    
    drawnow;

    
    
    
        [points, ris] = asrm.get_rm(vec');
    b=abs(points(:,3)-0.3)<0.025;
    

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

    subplot(1,2,2)
    Visualization.draw_rm(points(b,:), ris(b,:))
   
    set(gca,"FontSize",16)
    view(0,90)
   
    grid(gca,'minor')
    axis equal
    title("20cm above ground level")
    xlabel("x(m)")
    ylabel("y(m)")
    colorbar  
     xlim([-0.6, 1.2]);
    ylim([-1, 1]);
    zlim([-1, 1]);
    drawnow;