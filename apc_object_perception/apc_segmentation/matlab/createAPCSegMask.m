function [totalMask, overlayIm] = createAPCSegMask(capturesDir, dir1, dir2)

blendAlpha = .25;

imDir = fullfile(capturesDir,dir1,dir2,filesep);
rgbIm = imread(fullfile(imDir,'rgb.png')); rgbIm=double(rgbIm)/255;
availMasks = dir([imDir,'mask*']);
% imshow(rgbIm)
% pause
[imH,imW,~]=size(rgbIm);
numPx = imH*imW;

totalMask = zeros(imH,imW,'uint8');
overlayIm = rgbIm;
% first, handle box (and ground plane for shelf)
fnames = {'mask_box.png', 'mask_ground_metal.png'};
if ~isempty(strfind(capturesDir,'tote')), fnames = {'mask_box.png'}; end
for ff=fnames
    maskFile = char(ff);
    id = getAPCClassIDFromFilename(maskFile);
    col = getColorFromID(id);
    mask = zeros(size(rgbIm));
    maskIm = imread(fullfile(imDir,maskFile));
    maskedPx = find(maskIm);

    mask(maskedPx) = col(1);mask(maskedPx+numPx) = col(2);mask(maskedPx+2*numPx) = col(3);
    totalMask(maskedPx) = id;
    overlayIm = overlayIm + blendAlpha*mask;
end

% now iterate through rest
for a=1:length(availMasks)
    maskFile = availMasks(a).name;
    
    id = getAPCClassIDFromFilename(maskFile);
    if id==0 || id==2 || id == 43, continue; end % ignore box mask or ground plane

    col = getColorFromID(id);
    mask = zeros(size(rgbIm));
    maskIm = imread(fullfile(imDir,maskFile));
    maskedPx = find(maskIm);
    
    mask(maskedPx) = col(1);
    mask(maskedPx+numPx) = col(2);
    mask(maskedPx+2*numPx) = col(3);
%     imshow(mask)
%     pause
    totalMask(maskedPx) = id;
%     imtoshow = totalMask;
%     imshow(imtoshow,[min(imtoshow(:)),max(imtoshow(:))]);
%     pause
    overlayIm = overlayIm + blendAlpha*mask;
end


edges=getEdges(totalMask);
edgeIm = zeros(size(rgbIm));
for c=1:3, edgeIm(:,:,c) = edges; end
overlayIm = overlayIm + edgeIm;


% segFilename = fullfile(imDir,'segmentation.png');
% overlayFilename = fullfile(imDir,'overlay.png');

% fprintf('Writing segmentation mask into %s\n',segFilename);
% imwrite(uint8(totalMask), segFilename);
% 
% fprintf('Writing overlay image into %s\n',overlayFilename);
% imwrite(overlayIm, overlayFilename);
end