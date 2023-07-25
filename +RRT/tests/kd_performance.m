n=1000;
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
tic
for ii=1:n
     [isReached, isInserted] = kdtree.insert(nodes{ii});
     assert(isReached && isInserted)
end
toc()/n
%%
test_point=RRT.Node(1,1,1,100);
[nn, min_dist] = kdtree.NN(test_point);
%%
weights=RRT.Node.weights;
m=test_point.pose - datan;
d= sqrt(sum((m.^2).*weights,2));

[min_dist_m, nni]  = min(d);
nnm=nodes{nni};

display("results:")
[min_dist;min_dist_m]
[nn.pose;nnm.pose]
%% assert test 

for ii=1:10
    tp=rand(1,4).*randw;
    test_node=RRT.Node(tp(1),tp(2),tp(3),tp(4));
    [kdnn, kdmind] = kdtree.NN(test_node);
    
    m=test_node.pose - datan;
    m(m(:,5)<0,5)=Inf;
    d= sqrt(sum((m.^2).*weights,2));    

    [mind, mini]  = min(d);
    minnn=nodes{mini};
    
    assert(kdnn==minnn)
    assert(abs(mind-kdmind)<0.0001)
end

%% time test 
tic();
for ii=1:100
    tp=rand(1,4).*randw;
    test_node=RRT.Node(tp(1),tp(2),tp(3),tp(4));
    [kdnn, kdmind] = kdtree.NN(test_node);
    
end
toc()/10000
