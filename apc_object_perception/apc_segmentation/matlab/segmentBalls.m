%% segment blue ball
imfile = '/home/milan/research/projects/APC/apc_data/tote/run_20160628_1704/image_013/rgb.png';
rgbIm = double(imread(imfile))/255;
rgbImR = rgbIm(:,:,1);
rgbImG = rgbIm(:,:,2);
rgbImB = rgbIm(:,:,3);
grayIm = rgb2gray(rgbIm);
numPix = size(rgbIm,1) * size(rgbIm,2);

maskfile = '/home/milan/research/projects/APC/apc_data/tote/run_20160628_1704/image_013/mask_combined.png';
imshow(rgbIm);

figure(1)
mask = imread(maskfile);
objID = 6;
mask(mask ~= objID) = 0;
mask(mask == objID) = 1;

mask = double(mask);

figure(2)
imshow(mask);

objInd = find(mask(:));
objIndR = objInd;objIndG = objInd+numPix;objIndB = objInd+numPix*2;

whiteThr = .8;
alphablend = .5;
% red
redInd = find(rgbImR(objInd)>rgbImB(objInd) & rgbImR(objInd)>rgbImG(objInd) & grayIm(objInd)<whiteThr);

figure(3)
maskR = zeros(size(mask)); maskR(objInd(redInd))=1;
maskR = gray2rgb(maskR);
imshow(alphablend*maskR + (1-alphablend)*rgbIm);

% yellow
greenInd = find(rgbImG(objInd)>rgbImB(objInd) & rgbImG(objInd)>rgbImB(objInd) & grayIm(objInd)<whiteThr);
figure(4)
maskG = zeros(size(mask)); maskG(objInd(greenInd))=1;
maskG = gray2rgb(maskG);
imshow(alphablend*maskG + (1-alphablend)*rgbIm);

% blue
blueInd = find(rgbImB(objInd)>rgbImR(objInd) & rgbImB(objInd)>rgbImG(objInd) & grayIm(objInd)<whiteThr);

figure(5)
maskB = zeros(size(mask)); maskB(objInd(blueInd))=1;
maskB = gray2rgb(maskB);
imshow(alphablend*maskB + (1-alphablend)*rgbIm);


%%% HSV Separation
HSVLimits =  ...
    [.02 .1; % orange
     .15 .3;  %yellow
     .5 .65;]; %blue

% orange
hsvIm = rgb2hsv(rgbIm);
hue = hsvIm(:,:,1);
redInd = find(hue>HSVLimits(1,1) & hue<HSVLimits(1,2));
figure(3)
maskR = zeros(size(mask)); maskR(redInd)=1; maskR(~mask)=0;
maskR = gray2rgb(maskR);
imshow(alphablend*maskR + (1-alphablend)*rgbIm); 

% yellow
yellowInd = find(hue>HSVLimits(2,1) & hue<HSVLimits(2,2));
figure(4)
maskY = zeros(size(mask)); maskY(yellowInd)=1; maskY(~mask)=0;
maskY = gray2rgb(maskY);
imshow(alphablend*maskY + (1-alphablend)*rgbIm); 

% blue
blueInd = find(hue>HSVLimits(3,1) & hue<HSVLimits(3,2));
figure(5)
maskB = zeros(size(mask)); maskB(blueInd)=1; maskB(~mask)=0;
maskB = gray2rgb(maskB);
imshow(alphablend*maskB + (1-alphablend)*rgbIm); 