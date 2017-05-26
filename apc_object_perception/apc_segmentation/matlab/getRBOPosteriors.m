function posteriors = getRBOPosteriors(imDir)

    setting = 'shelf';
    if ~isempty(strfind(imDir,'tote')), setting = 'tote'; end

    nClasses = getNumClasses(setting);
    posteriors = zeros(1080, 1920, nClasses);


    availPosteriors = dir([imDir,'post_*.raw']);


    % now iterate through rest
    for a=1:length(availPosteriors)
        postFile = availPosteriors(a).name;

        id = getAPCClassIDFromFilename(postFile);

        fileID=fopen(fullfile(imDir,postFile));
        post = fread(fileID,'float'); post = reshape(post,1920,1080)';
        fclose(fileID);

        posteriors(:,:,id) = post;

    end

end