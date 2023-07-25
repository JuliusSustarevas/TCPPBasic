% I think this is a test script that simply samples from IRM  for a given
% task point. 

% asirm=RM.ASIRM() % I keep this commented out because loading RM.ASIRM()
% takes a minute or two.
%%
task_point=[1 -1 0.1];
% task_point=[1 -1 -0.1];
n=1000;
vec=[1,0,0];

while true    
    
    [points, bools, iris]=asirm.sample_irm_at(task_point, 0, n);
    %% Vec test
    %b=iris>0.2;
    %sum(b)/length(iris)
    iris=asirm.validate_basepoint( points, bools,vec);  
    b=iris>0.2;
    sum(b)/length(iris)
    
    
    %% index test
    bp=points(1,:)
    iri=iris(1)
    lbp=[(bp(1:3) - task_point) bp(4)];
    asirm.point2iri(lbp)
    
    iris=asirm.validate_basepoint( points, bools,vec);
    [~, k]=max(iris);
    bp=points(k,:)
    iri=iris(k)
    lbp=[(bp(1:3) - task_point) bp(4)];
    asirm.point2iri(lbp, vec)
        
    %% Draw
   
    
    quiver3(task_point(1),task_point(2),0,1,0,0,0.2,'r');
    hold on
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
    Visualization.draw_poses4(points(b,:),0.1);
    hold off
  
    drawnow;

end

%%
