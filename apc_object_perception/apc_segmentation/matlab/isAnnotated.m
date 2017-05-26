function ret = isAnnotated( path )
    fprintf('Checking "%s"', fullfile(path, 'polygons.yaml'));
    ret = exist(fullfile(path, 'polygons.yaml'), 'file') == 2;
end

