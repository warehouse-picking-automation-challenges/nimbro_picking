%%
% colorize greyscale segmentation
segim = imread('/home/milan/research/projects/APC/semseg/Sven/000003_labels.png');
npix = prod(size(segim));
imshow(segim);
colseg = zeros(size(segim,1), size(segim,2), 3);

ids = unique(segim)';
for id = ids
    col = getColorFromID(id);
    px = find(segim == id);
    colseg(px) = col(1);
    colseg(px+npix) = col(2);
    colseg(px+2*npix) = col(3);
end
imshow(colseg);