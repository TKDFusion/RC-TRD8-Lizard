function [PIDdata] = readLog(basename)


    %% Check inputs
    
    if ~exist('basename', 'var')
        basename = '*.log';         %no input: load any .log file
    elseif ~endsWith(regexptranslate('wildcard',basename),'.log')
        if endsWith(regexptranslate('wildcard',basename),'*')
            basename = strcat(basename,'.log'); %input without file identifier but with wildcard
        else
            basename = strcat(basename,'*.log'); %input without file identifier and without wildcard
        end
    end
    
    files = dir(basename);
    
    if size(files,1)==0
        disp('No such file or directory.')
        PIDdata = NaN;
        return
    elseif size(files,1)>1
        fprintf('More than one matching file found in current directory.\nFollowing data is loaded:\n')
    end
    
    %% Load data
    
    fprintf(strcat('Loading',32,'"',files(1).name,'"','...\n')); 
    PIDdata = readtable(files(1).name);
    disp('Data successfully added to workspace!')

end
