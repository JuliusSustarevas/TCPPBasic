function path=skip_smooth(rrt, path, path_pass_n)


    for ii=1:path_pass_n

        pi=1+randperm(length(path)-2);   
        
        for jj=pi                                     
            fprintf('Pass: %d;\n',ii);
            
            node=path(jj);
            nparent=path(jj-1);
            nchild=path(jj+1);            

            temp_reach=rrt.e_reach;
            rrt.e_reach=2*rrt.e_reach;
            [~, reach] = rrt.extend(nparent , nchild);           
            rrt.e_reach=temp_reach;
            if ~(reach)
                continue
            end            
           
            nchild.rrt_parent=nparent;        
         
        end
        
        rrt.draw=false;
        path=rrt.trace_path(path(end));

    end   
       

end


