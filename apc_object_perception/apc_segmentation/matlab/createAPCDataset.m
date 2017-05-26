%% shelf hard
addpath(genpath('./external'))  % dependencies
[APCDataDir, APCOutDataDir] = getAPCDataDir();
instCnt=0;
allFilesShelf = {};
unknownClassID = 44;
doShelf = true;
doTote = false;
[y,m,d] = datevec(date);
% mainName = 'apc_lowres_hha_0622';
mainName = sprintf('apc_lowres_%02d%02d',m,d);

%% shelf hard
if doShelf
    fprintf('--- SHELF  HARD ---\n');
    capturesDir = [APCDataDir,'shelf/20160424_hard/'];
    imagesDir = dir([capturesDir,'bin_*']);
    for imD = 1:length(imagesDir)
        instanceDirs = dir([capturesDir,imagesDir(imD).name,filesep,'image*']);
        fprintf('Processing %s...\n',imagesDir(imD).name);
        for inst = 1:length(instanceDirs)
            instCnt=instCnt+1;        
            allFilesShelf{instCnt} = fullfile('shelf','20160424_hard',imagesDir(imD).name, instanceDirs(inst).name);
            fprintf('Processing %s...\n',instanceDirs(inst).name);
    %         pause
            cropBox;
        end
    end

    %% shelf captures
    fprintf('--- SHELF  ---\n');
    capturesDir = [APCDataDir,'shelf/'];
    imagesDir = dir([capturesDir,'capture_*']);
    for imD = 1:length(imagesDir)
        instanceDirs = dir([capturesDir,imagesDir(imD).name,filesep,'image*']);
        fprintf('Processing %s...\n',imagesDir(imD).name);
        for inst = 1:length(instanceDirs)
            instCnt=instCnt+1;        
            allFilesShelf{instCnt} = fullfile('shelf',imagesDir(imD).name, instanceDirs(inst).name);
            fprintf('Processing %s...\n',instanceDirs(inst).name);
            cropBox;
        end
    end
    allFilesShelf = allFilesShelf'; % make Nx1 cell array
end

%% tote captures
if doTote
    instCnt=0;
    unknownClassID = 41;
    allFilesTote = {};
    fprintf('--- TOTE  ---\n');
    capturesDir = [APCDataDir,'tote/'];
    imagesDir = dir([capturesDir,'capture_*']);
    for imD = 1:length(imagesDir)
        instanceDirs = dir([capturesDir,imagesDir(imD).name,filesep,'image*']);
        fprintf('Processing %s...\n',imagesDir(imD).name);
        for inst = 1:length(instanceDirs)
            instCnt=instCnt+1;        
            allFilesTote{instCnt} = fullfile('tote',imagesDir(imD).name, instanceDirs(inst).name);
            fprintf('Processing %s...\n',instanceDirs(inst).name);
            cropBox;
        end
    end
    fprintf('--- TOTE   New  ---\n');
    capturesDir = [APCDataDir,'tote/'];
    imagesDir = dir([capturesDir,'run_*']);
    for imD = 1:length(imagesDir)
        fprintf('Processing %s...\n',imagesDir(imD).name);

        instanceDirs = dir([capturesDir,imagesDir(imD).name,filesep,'image*']);
        fprintf('%d dirs...\n', length(instanceDirs))
        % Remove directories w/o a polygons.yaml file
        instanceDirs = instanceDirs(cellfun(@(x) isAnnotated(fullfile(capturesDir, imagesDir(imD).name, x)), {instanceDirs.name}));
        fprintf('%d annotated...\n', length(instanceDirs))

        for inst = 1:length(instanceDirs)
            instCnt=instCnt+1;        
            allFilesTote{instCnt} = fullfile('tote',imagesDir(imD).name, instanceDirs(inst).name);
            fprintf('Processing %s...\n',instanceDirs(inst).name);
            cropBox;
        end
    end
    
    allFilesTote = allFilesTote'; % make Nx1 cell array
end

%% put everything into mat files for training
doSetting = [];
if doShelf, doSetting = [doSetting, 1]; end
if doTote, doSetting = [doSetting, 2]; end
rng(321)
datasetDir = '/home/local/apc/seg_apc_data/';
TrainingRatio = .8;
for ss=doSetting % shelf / tote    
    
    setting = 'shelf';
    filesArray = allFilesShelf;
    if ss==2
        setting='tote'; 
        filesArray = allFilesTote;
    end
    nClasses = getNumClasses(setting);
    fprintf('Setting: %s\n',setting);
    
    % save all files
    fileID = fopen(sprintf('%s/%s_%s_all_files.txt', datasetDir,mainName,setting),'w');
    [nrows,ncols] = size(filesArray);
    for row = 1:nrows, fprintf(fileID,'%s\n',filesArray{row}); end
    
%     nSamples = length(dir(sprintf('%s/%s/*rgb*',datasetDir,setting)));
%     allIdx = randperm(nSamples);
%     nTraining = round(nSamples*TrainingRatio);
%     trngIdx = allIdx(1:nTraining);
%     testIdx = allIdx(nTraining+1:end);
%     nTest = length(testIdx); nTraining=length(trngIdx);
    
    % testfile
    testfileNames = importdata('/home/max/apc/src/apc_object_perception/apc_densecap/densecap/data/split00.txt');
    
    testIdx = [];
    for i = 1:length(testfileNames)
        for j = 1:length(filesArray)
            if (strcmp(strcat(setting, '/', testfileNames(i)) , filesArray(j)))
                testIdx(length(testIdx) + 1) = j;
            end
        end
    end
    
    trainSplitFiles = ['/home/max/apc/src/apc_object_perception/apc_densecap/densecap/data/split01.txt';
                       '/home/max/apc/src/apc_object_perception/apc_densecap/densecap/data/split02.txt';
                       '/home/max/apc/src/apc_object_perception/apc_densecap/densecap/data/split03.txt';
                       '/home/max/apc/src/apc_object_perception/apc_densecap/densecap/data/split04.txt'];
   
    trngIdx =[];         
    disp(length(trainSplitFiles))
    for splitFile = 1:4
        disp(trainSplitFiles(splitFile,:))
        trainfileNames = importdata(trainSplitFiles(splitFile,:));
        
        for i = 1:length(trainfileNames)
            for j = 1:length(filesArray)
                if (strcmp(strcat(setting, '/', trainfileNames(i)) , filesArray(j)))
                    trngIdx(length(trngIdx) + 1) = j;
                end
            end
         end
    end
    fprintf('No. of train/test images: %d/%d\n', length(trngIdx), length(testIdx));

        
    for tt=1:2 % train / test / all
        trainTestMode = 'train'; 
        idx = trngIdx; 
        
        if tt==2 
            idx=testIdx; 
            trainTestMode='test';
        elseif tt==3
            idx = allIdx;
            trainTestMode='all';
        end
        fprintf('Mode: %s\n',trainTestMode);

        
        % go training
        imArray=zeros(targetHeight,targetWidth,3,length(idx));
        labelArray=zeros(targetHeight,targetWidth,length(idx),'uint8');
        depths=zeros(targetHeight,targetWidth,length(idx));
        hha=zeros(targetHeight,targetWidth,3,length(idx));
        if strcmp(setting,'tote')
            dcprobs=zeros(targetHeight,targetWidth,nClasses,length(idx));
        end
        %rboposts=zeros(targetHeight,targetWidth,nClasses,length(idx)); %%%RBO%%%
%         d2s=zeros(targetHeight,targetWidth,length(idx));
%         rmd=zeros(targetHeight,targetWidth,length(idx));
        for t = 1:length(idx)
            imArray(:,:,:,t)=double(imread(sprintf('%s/%s/%06d_rgb.png',datasetDir,setting,idx(t))))/255;
            labelArray(:,:,t)=imread(sprintf('%s/%s/%06d_labels.png',datasetDir,setting,idx(t)));
%             depths(:,:,t)=double(imread(sprintf('%s/%s/%06d_depth.png',datasetDir,setting,idx(t))))/255;
            load(sprintf('%s/%s/%06d_features.mat',datasetDir,setting,idx(t)));
            
            imArray=single(imArray); 
  %         dcprobs=single(dcprobs);
            %rboposts=single(rboposts); %%%RBO%%%
            labelArray=uint8(labelArray); 
            featDepth=single(featDepth); 
            featHHA = single(featHHA); 
%             d2s=single(d2s);
%             rmd=single(rmd);
        
            depths(:,:,t)=featDepth;
            hha(:,:,:,t)=featHHA;
%             d2s(:,:,t)=featDist2Wall;
            dcFilename = sprintf('%s/%s/%06d_densecap.mat',datasetDir,setting,idx(t));
            if strcmp(setting,'tote'), dcprobs(:,:,:,t)=featDensecap; end
%             if ss==2 && exist(dcFilename, 'file')% tote
%                 rmd(:,:,t) = featRelMedianDiff;
%                 load(dcFilename);
%                 dcprobs(:,:,:,t)=densecap;
                %load(sprintf('%s/%s/%06d_rbo.mat',datasetDir,setting,idx(t))); %%%RBO%%%
                %rboposts(:,:,:,t)=rbo; %%%RBO%%%
%             end
        end
        labelArray(labelArray==0)=nClasses;
        labelArray(labelArray>nClasses)=nClasses;
        
        imArray=permute(imArray,[2,1,3,4]); 
        if strcmp(setting,'tote'), dcprobs=permute(dcprobs,[2,1,3,4]); end
        %rboposts=permute(dcprobs,[2,1,3,4]); %%%RBO%%%
        labelArray=permute(labelArray,[2,1,3]);
        depths=permute(depths,[2,1,3]);
        hha=permute(hha,[2,1,3,4]);
%         d2s=permute(d2s,[2,1,3]);
%         rmd=permute(rmd,[2,1,3]);
                

        
        if strcmp(setting,'tote')
            dcprobs1=dcprobs(:,:,1:10,:);
            dcprobs2=dcprobs(:,:,11:20,:);
            dcprobs3=dcprobs(:,:,21:30,:);
            dcprobs4=dcprobs(:,:,31:41,:);
            clear dcprobs
        end
        dcprobs_dummy = 0;
        
%         save(sprintf('%s/%s_%s_%s.mat',datasetDir,mainName,setting,trainTestMode),'imArray','labelArray','depths','hha','d2s','rmd','dcprobs','idx','filesArray*','-v7.3');
        save(sprintf('%s/%s_%s_%s.mat',datasetDir,mainName,setting,trainTestMode),'imArray','labelArray','depths','hha','dcprobs*','idx','filesArray*','-v7.3');
        
        
        %save(sprintf('%s/%s_%s_%s.mat',datasetDir,mainName,setting,trainTestMode),'imArray','labelArray','depths','hha','d2s','rmd','dcprobs','rboposts','idx','filesArray*','-v7.3'); %%%RBO%%%
        
        filesArraySubset = filesArray(idx,:);
        % save train / test split        
        fileID = fopen(sprintf('%s/%s_%s_%s_files.txt',datasetDir,mainName,setting,trainTestMode),'w');
        [nrows,ncols] = size(filesArraySubset);
        for row = 1:nrows, fprintf(fileID,'%s\n',filesArraySubset{row}); end        
        
        % LIGHT SUBSET
        n = min(length(idx),5);
        imArray=imArray(:,:,:,1:n);
        labelArray=labelArray(:,:,1:n);        
        depths=depths(:,:,1:n); 
        hha=hha(:,:,:,1:n);
%         d2s=d2s(:,:,1:n);
%         rmd=rmd(:,:,1:n);
        if strcmp(setting,'tote')
    %         dcprobs=dcprobs(:,:,:,1:n); 
            dcprobs1=dcprobs1(:,:,:,1:n); 
            dcprobs2=dcprobs2(:,:,:,1:n); 
            dcprobs3=dcprobs3(:,:,:,1:n); 
            dcprobs4=dcprobs4(:,:,:,1:n); 
        end
        
        %rboposts=rboposts(:,:,:,1:n); %%%RBO%%%
        idx=idx(1:n); 
        filesArraySubset = filesArraySubset(1:n,:);
        save(sprintf('%s/%s_%s_%s_light.mat',datasetDir,mainName,setting,trainTestMode),'imArray','labelArray','depths','hha','dcprobs*','idx','filesArray*','-v7.3');
        
	% save(sprintf('%s/%s_%s_%s_light.mat',datasetDir,mainName,setting,trainTestMode),'imArray','labelArray','depths','hha','d2s','rmd','dcprobs','rboposts','idx','filesArray*','-v7.3'); %%%RBO%%%

        
        % save train / test split        
        fileID = fopen(sprintf('%s/%s_%s_%s_files_light.txt',datasetDir,mainName,setting,trainTestMode),'w');
        [nrows,ncols] = size(filesArraySubset);
        for row = 1:nrows, fprintf(fileID,'%s\n',filesArraySubset{row}); end
    
    end
end
