%% max to mine
direc = '/home/milan/research/projects/APC/semseg/Sven';

im = imread(fullfile(direc,'segm_combined.png'));
[imH, imW] = size(im);

% boxMask = ones(size(im));
boxMask = imread(fullfile(direc,'mask_box.png'));

firstLeft = find(sum(boxMask,1),1,'first'); lastRight = find(sum(boxMask,1),1,'last');
firstTop = find(sum(boxMask,2),1,'first'); lastBottom = find(sum(boxMask,2),1,'last');


targetWidth = 640; targetHeight = 480;
%%%%%    VARIANT II   %%%%%
% crop from left/right, putting it in the h-center of the mask_box
maskCenter = round(firstLeft + (lastRight - firstLeft)/2);
bxTop = 1; bxBottom = imH;
bxLeft = maskCenter-720+1; bxRight = maskCenter+720;

if bxLeft<1,    bxRight=bxRight+abs(bxLeft)+1; bxLeft = 1; end % move right if necessary
if bxRight>imW, bxLeft=bxLeft-(bxRight-imW); bxRight=imW; end % move left if necessary
bxWidth = bxRight - bxLeft+1; bxHeight = bxBottom-bxTop+1; aR = bxWidth/bxHeight;

degRot=180;
im = im(bxTop:bxBottom, bxLeft:bxRight,:); 
im = imresize(im, [targetHeight, targetWidth],'nearest'); 
im=imrotate(im, degRot);



[imH, imW] = size(im);
numPix = imH * imW;

mySeg = zeros(imH, imW, 3);

objIDs = [13,3,1,12,9,11,2,5,4,7,38,10];
% objIDs = [6,7,11,13,14,15,17,18,20,29,38,39];
% objIDs = objIDs(end:-1:1);
% objIDs = 1:12;

for c=0:max(im(:))
    classInd = find(im(:)==c);
    thisChannel = zeros(size(im));
    thisChannel(im==c)=1;
    
    id = 8;
    if c>0, id = objIDs(c); end
    col = getColorFromID(id);
    mySeg(classInd) = col(1);
    mySeg(numPix + classInd) = col(2);
    mySeg(2*numPix + classInd) = col(3);
    
%     sum(thisChannel(:))
end

imshow(mySeg)
imwrite(mySeg,fullfile(direc,'segm_combined_recolored.png'));

%% prob masks
% availProbs = dir([direc,'/prob_*.png']);
% 
% % now iterate through rest
% for a=1:length(availProbs)
%     probFile = availProbs(a).name;
%     
%     
%     im = imread(fullfile(direc,probFile));
%     [imH, imW] = size(im);
% 
%     boxMask = ones(size(im));
%     firstLeft = find(sum(boxMask,1),1,'first'); lastRight = find(sum(boxMask,1),1,'last');
%     firstTop = find(sum(boxMask,2),1,'first'); lastBottom = find(sum(boxMask,2),1,'last');
% 
% 
%     targetWidth = 640; targetHeight = 480;
%     %%%%%    VARIANT II   %%%%%
%     % crop from left/right, putting it in the h-center of the mask_box
%     maskCenter = round(firstLeft + (lastRight - firstLeft)/2);
%     bxTop = 1; bxBottom = imH;
%     bxLeft = maskCenter-720+1; bxRight = maskCenter+720;
% 
%     if bxLeft<1,    bxRight=bxRight+abs(bxLeft)+1; bxLeft = 1; end % move right if necessary
%     if bxRight>imW, bxLeft=bxLeft-(bxRight-imW); bxRight=imW; end % move left if necessary
%     bxWidth = bxRight - bxLeft+1; bxHeight = bxBottom-bxTop+1; aR = bxWidth/bxHeight;
% 
%     degRot=180;
%     im = im(bxTop:bxBottom, bxLeft:bxRight,:); 
%     im = imresize(im, [targetHeight, targetWidth],'nearest'); 
%     im=imrotate(im, degRot);
%     
%     imwrite(im,fullfile(direc,probFile));
% 
% 
% end
