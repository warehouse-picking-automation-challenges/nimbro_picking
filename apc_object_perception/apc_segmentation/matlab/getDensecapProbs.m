function probs = getDensecapProbs(imDir)

setting = 'shelf';
if ~isempty(strfind(imDir,'tote')), setting = 'tote'; end

nClasses = getNumClasses(setting);
probs = zeros(1080, 1920, nClasses);

% availMasks = dir([imDir,'mask_*.png']);
availProbs = dir([imDir,'/prob_*.raw']);
fprintf('%d densecap masks found\n',length(availProbs));
if length(availProbs) == 0
    fprintf('WARNING!!! NO DENSECAP AVAILABLE!!!\n')
end

% now iterate through rest
for a=1:length(availProbs)
    probFile = availProbs(a).name;
    
    id = getAPCClassIDFromFilename(probFile);
%     prob = double(imread(fullfile(imDir,probFile)))/255;

    fileID=fopen(fullfile(imDir,probFile));
    prob = fread(fileID,'float'); prob = reshape(prob,1920,1080)';
    fclose(fileID);
    
    probs(:,:,id) = prob;
end

end