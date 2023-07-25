import Tasks.*
p = HPath(2,400);
p.smooth(0.075);
p.plot()
q = UPath(.1);
q.scale([0.2 0.05 1]);
%%
p.scale([6, 6, 1]);
p.resample(0.001);
p.plot();
%% 
p.superimpose(q) 
task=p;
task.resample(0.01);
task.plot();
