nozzle=0.006;
ww=0.02;
%%
pts=[0 0 0;
    0 -400 0;
    200 -400 0;
    200 (-400 +131.5) 0;
    200 (-400 +131.5) 1;
    200 -400 1;
    200 -400 0;
    400 -400 0;
    400 -160 0;
    196 -160 0;
    196 -160 1;
    (400-160) -160 1;
    (400-160) -160 0;
    (400-160) (-160+37) 0;
    (400-160) (-160+37) 1;
    (400-160) -160 1;
    400 -160 1;
    400 -160 0;
    400 0 0;
    0 0 0;
%     0 0 1;
%     0 -400 1;
%     400 -400 1;
%     400 0 1;
%     0 0 1;
    ];

d=30;

l1= @(z) [(linspace(0,1,100)') zeros(100,1) ones(100,1)*z];
l2= @(z) [(linspace(1,1+cosd(d),100)') (linspace(0,sind(d),100)') ones(100,1)*z];
l3= @(z) [(linspace(1+cosd(d),1+cosd(d)+cosd(2*d),100)') (linspace(sind(d),sind(d)+sind(2*d),100)') ones(100,1)*z];
l4= @(z) [(linspace(1+cosd(d),1+2*cosd(d),100)') (linspace(sind(d),2*sind(d),100)') ones(100,1)*z];
l5= @(z) [(linspace(1+cosd(d),1+cosd(d)+1,100)') (linspace(sind(d),sind(d),100)') ones(100,1)*z];
l6= @(z) [(linspace(1,1+cosd(-d),100)') (linspace(0,sind(-d),100)') ones(100,1)*z];
l7= @(z) [(linspace(1+cosd(-d),1+cosd(-d)+cosd(-d),100)') (linspace(sind(-d),0,100)') ones(100,1)*z];
l8= @(z) [(linspace(1+cosd(-d),1+2*cosd(-d),100)') (linspace(sind(-d),2*sind(-d),100)') ones(100,1)*z];
l9= @(z) [(linspace(1+cosd(-d),1+cosd(-d)+cosd(-2*d),100)') (linspace(sind(-d),sind(-d)+sind(-2*d),100)') ones(100,1)*z];



%%
pts=[l1(0);l2(0);l3(0);l4(0);l5(0);l6(0);l7(0);l8(0);l9(0);]
% plot(pts(:,1), pts(:,2),'.')
%%
lh=0.01;
pts=[
trim(l1(0))    ;
trim(l2(0))    ;
trim(l3(0))    ;
flipud(trim(l3(1))) ;
flipud(trim(l2(1))) ;
flipud(trim(l1(1))) ;
trim(l1(2)) ;
trim(l2(2));
trim(l4(0));
flipud(trim(l4(1))) ;
flipud(trim(l2(3))) ;
flipud(trim(l1(3)));
trim(l1(4)) ;
trim(l2(4));
trim(l5(0));
flipud(trim(l5(1))) ;
flipud(trim(l2(5))) ;
flipud(trim(l1(5)));
trim(l1(6)) ;
trim(l6(0));
trim(l7(0));
flipud(trim(l7(1))) ;
flipud(trim(l6(1))) ;
flipud(trim(l1(7)));
trim(l1(8)) ;
trim(l6(2));
trim(l8(0));
flipud(trim(l8(1))) ;
flipud(trim(l6(3))) ;
flipud(trim(l1(9)));
trim(l1(10)) ;
trim(l6(4));
trim(l9(0));
flipud(trim(l9(1))) ;
flipud(trim(l6(5))) ;
flipud(trim(l1(11)));
]

pts(:,3)=pts(:,3)*0.01;


%%

path= Tasks.GenericPath(pts);
path.resample(0.001);
path.smooth(0.005);

%%
cla
n=size(path.path,1);
h = animatedline;

xlim([-0.1 3])
ylim([-1.5 1.5])
zlim([-0.1 1])
view(5,35)
drawnow
for ii=1:10:n
%     lol= Tasks.GenericPath(path.path(1:ii,:));
%     path.plot()
    addpoints(h,path.path(ii:ii+9,1),path.path(ii:ii+9,2),path.path(ii:ii+9,3));
    drawnow
    disp(ii)
end


%%
function l=trim(l)
l=l(10:end-10,:);
end



