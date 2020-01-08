function [out] = enable_uncertainty(function_name)
%enable_uncertainty replacing all ./, .* and .^ with /,* and ^
    str = fileread(strcat(function_name,'.m'));
    str = strrep(str,'./','/');
    str = strrep(str,'.*','*');
    str = strrep(str,'.^','^');
    fid  = fopen(strcat(function_name,'.m'),'w');
    fprintf(fid,'%s',str);
    fclose(fid);
    out = 1;
end

