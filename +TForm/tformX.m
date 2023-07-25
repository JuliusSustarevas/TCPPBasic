function T = tformX(T1,T2)
    %This function multiplies tforms along the 3rd dimension
    n1=size(T1,3);
    n2=size(T2,3);

    if n1==n2
        T=zeros(size(T1));
        for ii=1:n1
            T(:,:,ii)=T1(:,:,ii)*T2(:,:,ii);
        end
    elseif n1==1
        T=zeros(size(T2));
        for ii=1:n2
            T(:,:,ii)=T1*T2(:,:,ii);
        end
    elseif n2==1
        T=zeros(size(T1));
        for ii=1:n1        
            T(:,:,ii)=T1(:,:,ii)*T2;
        end
    else
        disp("not defined");
        T=[];
    end

end


