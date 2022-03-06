function [jsonoutput] = readJSON(filepath)

    json_cam2 = fopen(filepath);
    json_cam2_raw = fread(json_cam2,inf);
    str2 = char(json_cam2_raw');
    fclose(json_cam2);
    if str2(length(str2)) ~= ']'
        str2 = str2+"]";
    end
    jsonoutput = jsondecode(str2);
end

