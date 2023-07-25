% I think this script draws vectors that define an "elipse" around a base
% pose s.t. the IRI of base poses in that elipse is >0.5.

% asirm=RM.ASIRM() % I keep this commented out because loading RM.ASIRM()
% takes a minute or two.
%%
task_point=[1 3 1];
vec=[0,0,1]';
%%

[points, iris]=asirm.get_irm_at(task_point,vec,45);    
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
Visualization.draw_poses4(points(iris>0.5,:),0.75)


%%
while true
    [points, bools, iris]=asirm.sample_irm_at(task_point, 0, 5);
    iris=asirm.validate_basepoint( points, bools,vec);
    [~, k]=max(iris);
    bp=points(k,:);  
    iris(k)

    elipse=asirm.gen_elipse(task_point,bp, vec);
    if exist('qs')
        cellfun(@(c)delete(c),qs)
    end
    qs=Visualization.draw_elipse(bp,elipse);
    drawnow
    input("")
    
end