function vec = tform2vec(T,vec)
% transform tform into vec form. optionally provide existant vec
if exist('vec') && isequal(size(vec), [size(T,3),7])    
%     pass
else
    vec=zeros(size(T,3),7);
end
vec(:,1:3)=squeeze(T(1:3,4,:))';
vec(:,4:7)=tform2quat(T);
end

