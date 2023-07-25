% I think my IK stack would write booleeans into a txt (dont ask) and then
% I read from it to create a matlab data structure to be used in RM/IRM
% classes
fid = fopen('+SamplingRM/data/ik_temp.txt');

tline = fgetl(fid);

n=17262;
m=200;
rm_bools=false(n,m);
ii=1;

while ischar(tline)
    tline=strrep(tline,' ','');
    tline=strrep(tline,'[','');
    tline=strrep(tline,']','');    
    rm_bools(ii,:)=cellfun(@(c) strcmp(c,'True'),split(tline,','));
    tline = fgetl(fid);
    ii=ii+1;
end

save('rm_bools','rm_bools')