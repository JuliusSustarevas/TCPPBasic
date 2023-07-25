import Tasks.*
p = HPath(2,200);
p.smooth(0.08);
%%
p.scale([1, 2, 3]);
p.resample(0.01);
p.plot();
%%
f=p.asfun();
p.gett(.01);
p.is_repeating();
%%
p.toTForm(p);
p.fromTForm(p.toTForm(p));
p.plotTForm(p.toTForm(p),0.1)
%%   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
import Tasks.*
p = UPath(.1);
%%
p.scale([1, 2, 3]);
p.resample(0.01);
p.plot();
%%
f=p.asfun();
p.t(.01);
p.is_repeating();
%%
p.toTForm(p);
p.fromTForm(p.toTForm(p));
p.plotTForm(p.toTForm(p),0.1)
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
import Tasks.*
p = ArcPath(pi);
%%
p.scale([1, 2, 3]);
p.resample(0.01);
p.plot();
%%
f=p.asfun();
p.gett(.01);
p.is_repeating();
%%
p.toTForm(p);
p.fromTForm(p.toTForm(p));
p.plotTForm(p.toTForm(p),0.1)
%% %%%%%%%%%%%%%%%%%%%%%


