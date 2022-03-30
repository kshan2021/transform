function JSONstr = detectVehiclesGetCoord
oldDir = pwd;
cd('../darknet');

system("./darknet detector test ./cfg/coco.data ./training/t0907s-c80-b10000_2021-12-03/yolov4-t0907s-c80-b10000.cfg ./training/t0907s-c80-b10000_2021-12-03/yolov4-t0907s-c80-b10000_last.weights -dont_show -ext_output tmp.jpg -out tmp.json ");

inImage = imread('tmp.jpg');
[h, w, c] = size(inImage);

inFile = 'tmp.json';
fid = fopen(inFile);
raw = fread(fid, inf);
str = char(raw');
fclose (fid);
JSONstr = jsondecode(str);
%vehicles = JSONstr.objects;

cd(oldDir);
end

