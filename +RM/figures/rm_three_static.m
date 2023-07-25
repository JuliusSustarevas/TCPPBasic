load("+RM/data/as_rm.mat")
vec=[0,0,-1]';

clf

% tiledlayout(1,3, 'Padding', 'none', 'TileSpacing', 'compact');
% % % subplot(1,3,1)
% nexttile
% plot_rm(asrm,[0,0,-1])
% % 
% nexttile
% % subplot(1,3,2)
% plot_rm(asrm,[1,0,0])
% nexttile
% % % subplot(1,3,3)
plot_rm(asrm,[0,-1,0])
% set(gcf,"Color",[1 1 1])
% %     
function plot_rm(asrm,vec)

    [points, ris] = asrm.get_rm(vec);
    b=(abs(points(:,3))<0.015) | (abs(points(:,2))<0.026);
    Visualization.draw_rm(points(b,:), ris(b,:))
    xlim([-0.4, 1.2]);
    ylim([-1, 1]);
    zlim([-1, 1]);
    set(gca,"FontSize",14)
    xlabel("x(m)")
    ylabel("y(m)")
    zlabel("z(m)")
%     colorbar
    axis equal
    view(55,40)

end