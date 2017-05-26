%%
dir1=imagesDir(imD).name;
dir2=instanceDirs(inst).name;
imDir = fullfile(capturesDir,dir1,dir2,filesep);
% imDir = '/home/milan/research/projects/APC/apc_data/shelf/capture_20160419_1504/image001';

% overlayIm = imread(fullfile(imDir,'overlay.png'));
% segIm = imread(fullfile(imDir,'segmentation.png'));
[segIm, overlayIm] = createAPCSegMask(capturesDir, dir1, dir2);
rgbIm = imread(fullfile(imDir,'rgb.png')); rgbIm = double(rgbIm)/255;
% depthIm = imread(fullfile(imDir,'depth.png'));




boxMask = imread(fullfile(imDir,'mask_box.png'));
featDensecap = getDensecapProbs(imDir);
%rboPosteriors = getRBOPosteriors(imDir) %%%RBO%%%
[imH,imW]=size(boxMask);


% find box corners
croppingVersion = 2;

firstLeft = find(sum(boxMask,1),1,'first'); lastRight = find(sum(boxMask,1),1,'last');
firstTop = find(sum(boxMask,2),1,'first'); lastBottom = find(sum(boxMask,2),1,'last');

targetWidth = 640;
targetHeight = 480;
if croppingVersion == 1
    %%%%%    VARIANT I   %%%%%
    % put a 4/3 box on the ground plane of the mask_box window
    bxTop = max(1,firstTop); bxLeft=max(1,firstLeft); bxRight=min(imW,lastRight);bxBottom=min(imH,lastBottom);
    bxWidth = bxRight - bxLeft; bxHeight = bxBottom-bxTop; aR = bxWidth/bxHeight;

    % make 4:3 aspect ratio
    if aR > 4/3 % keep height, cut off sides
        bxCenterX = round(bxLeft + bxWidth/2);
        bxWidth = round(bxHeight * 4 / 3); % new width
        bxLeft = round(bxCenterX - bxWidth/2);
        bxRight = round(bxCenterX + bxWidth/2);
    else % keep width, clip top of box
        bxHeight = round(bxWidth * 3 / 4);
        bxBottom = bxTop + bxHeight;
    end
elseif croppingVersion == 2
    %%%%%    VARIANT II   %%%%%
    % crop from left/right, putting it in the h-center of the mask_box
    maskCenter = round(firstLeft + (lastRight - firstLeft)/2);
    bxTop = 1; bxBottom = imH;
    bxLeft = maskCenter-720+1; bxRight = maskCenter+720;

    if bxLeft<1,    bxRight=bxRight+abs(bxLeft)+1; bxLeft = 1; end % move right if necessary
    if bxRight>imW, bxLeft=bxLeft-(bxRight-imW); bxRight=imW; end % move left if necessary
    bxWidth = bxRight - bxLeft+1; bxHeight = bxBottom-bxTop+1; aR = bxWidth/bxHeight;
elseif croppingVersion == 3
    %%%%%    VARIANT III   %%%%%
    % just downsample twice full frame (half res)
    bxTop = 1; bxBottom = imH; bxLeft = 1; bxRight = imW;
    bxWidth = bxRight - bxLeft+1; bxHeight = bxBottom-bxTop+1; aR = bxWidth/bxHeight;
    targetWidth = 960; targetHeight = 540;
else
    error('huh?')
end

aR = bxWidth/bxHeight;


setting = 'shelf';
if ~isempty(strfind(capturesDir,'tote')), setting='tote'; end

degRot=180;

fprintf('Box: %d, %d, %d, %d. (%d x %d). Ratio = %.2f\n', bxLeft, bxTop, bxRight, bxBottom, bxWidth, bxHeight, aR);



% check if we already have everything
hhaFilename     = sprintf('%s/%s/%06d_hha.png',APCOutDataDir,setting,instCnt);
labelsFilename  = sprintf('%s/%s/%06d_labels.png',APCOutDataDir,setting,instCnt);
rgbFilename     = sprintf('%s/%s/%06d_rgb.png',APCOutDataDir,setting,instCnt);
depthFilename   = sprintf('%s/%s/%06d_depth.png',APCOutDataDir,setting,instCnt);
d2wFilename     = sprintf('%s/%s/%06d_dist2wall.png',APCOutDataDir,setting,instCnt);
rmdFilename     = sprintf('%s/%s/%06d_rmd.png',APCOutDataDir,setting,instCnt);
featFilename    = sprintf('%s/%s/%06d_features.mat',APCOutDataDir,setting,instCnt);

resave = true;
metaFilename = sprintf('%s/%s/%06d_info.txt',APCOutDataDir, setting,instCnt);
mfInfo = sprintf('%s,%s,%s,%d,%d,%d,%d,%d,%d,%.6f,%d', ...
    setting,dir1,dir2,bxLeft,bxTop,bxRight,bxBottom,bxWidth,bxHeight,aR,degRot);

if exist(metaFilename, 'file')    
    metaFile = fopen(metaFilename,'r');
    mfContent = textscan(metaFile,'%s'); mfContent=char(mfContent{1});
    if strcmp(mfContent,mfInfo) ...
            && exist(rgbFilename,'file') ...
            && exist(labelsFilename,'file') ...
            && exist(featFilename,'file')
        
        resave = false;
    end
end


if resave
    % load float depth
    ptd = loadpcd(sprintf('%s/frame_uncompressed.pcd',imDir));
    featDepth = ptd(:,:,3);

    segIm = segIm(bxTop:bxBottom, bxLeft:bxRight,:); segIm = imresize(segIm, [targetHeight, targetWidth],'nearest'); segIm=imrotate(segIm, degRot);
    imwrite(segIm,labelsFilename)
    rgbIm = rgbIm(bxTop:bxBottom, bxLeft:bxRight,:); rgbIm = imresize(rgbIm, [targetHeight, targetWidth],'nearest'); rgbIm=imrotate(rgbIm, degRot);
    imwrite(rgbIm,rgbFilename)
    featDepth = featDepth(bxTop:bxBottom, bxLeft:bxRight,:); featDepth = imresize(featDepth, [targetHeight, targetWidth],'nearest'); featDepth=imrotate(featDepth, degRot);
    imwrite(featDepth,depthFilename)

    % hha
    featHHA = imread(fullfile(imDir,'feature_hha.png')); featHHA = double(featHHA)/255;
    featHHA = featHHA(bxTop:bxBottom, bxLeft:bxRight,:); featHHA = imresize(featHHA, [targetHeight, targetWidth],'nearest'); featHHA=imrotate(featHHA, degRot);
    imwrite(featHHA,hhaFilename)

    % dist2wall
    featd2sFilename = 'feature_dist2shelf.raw'; if strcmp(setting, 'tote'), featd2sFilename = 'feature_dist2wall.raw'; end
    fileID=fopen(fullfile(imDir,featd2sFilename));
    featDist2Wall = fread(fileID,'float'); featDist2Wall = reshape(featDist2Wall,1920,1080)';
    featDist2Wall = featDist2Wall(bxTop:bxBottom, bxLeft:bxRight,:); featDist2Wall = imresize(featDist2Wall, [targetHeight, targetWidth],'nearest'); featDist2Wall=imrotate(featDist2Wall, degRot);
    minv=min(featDist2Wall(:)); maxv=max(featDist2Wall(:)); rangev = maxv-minv; visDist2Wall = (featDist2Wall-minv) / rangev;
    imwrite(visDist2Wall,d2wFilename)
    fclose(fileID);

    % relativeMeanDiff
    if strcmp(setting,'tote')
        fileID=fopen(fullfile(imDir,'feature_relativeMeanDiff.raw'));
        featRelMedianDiff = fread(fileID,'float'); featRelMedianDiff = reshape(featRelMedianDiff,1920,1080)';
        featRelMedianDiff = featRelMedianDiff(bxTop:bxBottom, bxLeft:bxRight,:); featRelMedianDiff = imresize(featRelMedianDiff, [targetHeight, targetWidth],'nearest'); featRelMedianDiff=imrotate(featRelMedianDiff, degRot);
        imwrite(featRelMedianDiff,rmdFilename)
        fclose(fileID);
    end

    % densecap prob
    if strcmp(setting,'tote')
        featDensecap = featDensecap(bxTop:bxBottom, bxLeft:bxRight,:); featDensecap = imresize(featDensecap, [targetHeight, targetWidth],'nearest'); featDensecap=imrotate(featDensecap, degRot);    
    end

    % save features
    save(featFilename,'feat*')



    % RBO rboPosteriors %%%RBO%%%

    % if strcmp(setting,'tote')
    %    rboPosteriors = rboPosteriors(bxTop:bxBottom, bxLeft:bxRight,:); rboPosteriors = imresize(rboPosteriors, [targetHeight, targetWidth],'nearest'); rboPosteriors=imrotate(rboPosteriors, degRot);
    %    save(sprintf('%s/%s/%06d_rbo.mat',APCOutDataDir,setting,instCnt),'rbo')
    % end


    % crpIm(bxTop:bxBottom, bxLeft:bxRight,:) =crpIm(bxTop:bxBottom, bxLeft:bxRight,:)*.5;

    % meta info
    metaFile = fopen(metaFilename,'w');
    fprintf(metaFile, mfInfo);
    fclose(metaFile);
else
    fprintf('%s already done. Skipping...\n',metaFilename);
end

% imshow(crpIm);
