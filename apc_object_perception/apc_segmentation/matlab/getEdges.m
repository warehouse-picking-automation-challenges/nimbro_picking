function edges=getEdges(limg, thickness)
% quickly determine edges from labeled image


if nargin<2, thickness=1; end

% make sure 1 < thickness < 3, int
thickness=round(max(1,thickness));
thickness=min(3,thickness);

limg=sum(limg,3);

av=limg; ah=limg;

for t=1:thickness
    av=~~(diff(av));
    ah=~~(diff(ah,[],2));
end


av(end+1,:)=0;
ah(:,end+1)=0;
% ah(:,end+1)=0; 

% edges=av+ah;

edges=av+ah;

end