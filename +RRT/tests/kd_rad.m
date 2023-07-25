n=10000;
randw=[5,5,2*pi,100];
data=rand(n,4).*randw;
datan=[data(:,1:2) cos(data(:,3)) sin(data(:,3)) data(:,4)];
%%
nodes=cell(n,1);
for ii=1:n
    nodes{ii}= RRT.Node(data(ii,1),data(ii,2),data(ii,3),data(ii,4));
end
kdtree=RRT.KDTree();
%%
for ii=1:n
     [isReached, isInserted] = kdtree.insert(nodes{ii});
     assert(isReached && isInserted)
end
%%
rad=1;
%% assert test 

for ii=1:10
    tp=rand(1,4).*randw;
    test_node=RRT.Node(tp(1),tp(2),tp(3),tp(4));
    [kdnn, kdmind] = kdtree.NNRad(test_node,rad);
    
    m=datan-test_node.pose;
%     m=test_node.pose - datan;
    m(m(:,5)<=0,5)=Inf;
    d= sqrt(sum((m.^2).*RRT.Node.weights,2));
    [mind, mini]  = sort(d);
    
    % sorts and extraction for comparison
    % extract
    minnn=[];
    mindd=[];
    for jj=mini(mind<rad)
        minnn=[minnn nodes{jj}];
        mindd=[mindd d(jj)];
    end   
    %sort
    [~,ids]=sort(kdmind);
    kdnn=kdnn(ids);
    kdmind=kdmind(ids);
    
    assert(all(kdnn==minnn))
    assert(all(abs(mindd-kdmind)<0.0001))
end
%% time test 
tic();
for ii=1:300
    tp=rand(1,4).*randw;
    test_node=RRT.Node(tp(1),tp(2),tp(3),tp(4));
    [kdnn, kdmind] = kdtree.NNRad(test_node,rad);
    
end
toc()/300

% growing list : 0037  00385  00334 00354 00354
%linked list: 0.0355 00332 00363 00376 00345