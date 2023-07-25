function path=teb_like_smooth(rrt, path, path_pass_n, node_n)


    for ii=1:path_pass_n

        pi=1+randperm(length(path)-2);   
        
        c=0;
        for jj=pi                             
            c=c+1;
            fprintf('Pass: %d; node: %d\n',ii,c);
            
            node=path(jj);
            nparent=path(jj-1);
            nchild=path(jj+1);
            costs=[];
            node_rands=[];

            for kk=1:node_n*5
                
                if kk==1
                    n_rand=node;
                else
                    n_rand=sample_better_node(rrt,node,path(jj-1),path(jj+1));
                    if n_rand.pose(5) <= nparent.pose(5) || n_rand.pose(5) >= nchild.pose(5) 
                        continue
                    end
                    temp_reach=rrt.e_reach;
                    rrt.e_reach=2*rrt.e_reach;
                    [~, reach1] = rrt.extend(nparent , n_rand);
                    [~, reach2] = rrt.extend(n_rand, nchild);
                    rrt.e_reach=temp_reach;
                    if ~(reach1 && reach2)
                        continue
                    end
                end

                node_rands=[node_rands; n_rand];                
                costs=[costs; get_costs(nparent,n_rand,nchild)];
                
                if length(node_rands)==node_n
                    break
                end
                
            end

            [~,ids]=sort(costs);
            %  Note something here should be checked about bools
            node.pose=node_rands(ids(1)).pose; 
            node.bools=[];
        end

    end

end


function costs=get_costs(nim1, ni, nip1)
     
%     a= 2*abs(( nip1.dist(ni)/(nip1.pose(5)-ni.pose(5))  ) - ( ni.dist(nim1)/(ni.pose(5) - nim1.pose(5)))) ;
%     a=0;


    d1=sqrt(nip1.dist(ni)^2 - nip1.weights(5)*((nip1.pose(5)-ni.pose(5))^2));
    d2=sqrt(ni.dist(nim1)^2 - ni.weights(5)*((ni.pose(5)-nim1.pose(5))^2));        
    d=(d1+d2)/(nip1.pose(5) - nim1.pose(5));
    a= abs((d1/(nip1.pose(5)-ni.pose(5))) - (d2/(ni.pose(5)-nim1.pose(5))));
    

%     Too much delta s old:
%     d1=sqrt(nip1.dist(ni)^2 - nip1.weights(5)*((nip1.pose(5)-ni.pose(5))^2));
%     d2=sqrt(ni.dist(nim1)^2 - ni.weights(5)*((ni.pose(5)-nim1.pose(5))^2));
%     d= (d1/(nip1.pose(5)-ni.pose(5))) + (d2/(ni.pose(5)-nim1.pose(5)));
%     a= 2*abs((d1/(nip1.pose(5)-ni.pose(5))) - (d2/(ni.pose(5)-nim1.pose(5))));
% [d, a]
%     d1=ni.dist(nim1);
%     d2=nip1.dist(ni);
%     s1=ni.pose(5)-nim1.pose(5);
%     s2=nip1.pose(5)-ni.pose(5);
%     
%     a= 2*abs(nip1.dist(ni)  -  ni.dist(nim1)) ;
%     d= d1**2-;

    costs=a+d;
    
end

        
function sample = sample_better_node(rrt, node,parent,child) 

    xyth=[node.pose(1), node.pose(2),atan2(node.pose(4),node.pose(3))];            

    while true     
        
        if node.pose(5)==0 || node.pose(5)==rrt.scene.task_smax
            %  this is either first or last node.
            s=node.pose(5);
        else
%             s=max(min(node.pose(5)+0.1*randn(),rrt.scene.task_smax),0);
            s= parent.pose(5)+ rand()*((child.pose(5)-parent.pose(5)));
        end        
        
        
        sample=RRT.Node(xyth(1)+randn()*0.05,xyth(2)+randn()*0.05,xyth(3)+randn()*0.05,s);
        if rrt.scene.isValid7(sample)                    
            break
        end
    end

end


