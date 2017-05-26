function classID = getAPCClassIDFromFilename(filename)

% explode filename
[~,n,~]=fileparts(filename);
n=n(6:end); % remove 'mask_'

% get object list and search for object
apc_obj = getAPCObjList;
[~, classID]=ismember(n,apc_obj);

end