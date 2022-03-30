%% given JSON files with a 2D bounding box, track objects for duration of video
% input image with bounding box needs corresponding txt output (from YOLOv4)

% Authors: Sebastian Gunner (u1922654) and Matteo Penlington (u1924672) 

%% Initial setup
clear RefImg_Ms;
clear;
close all;

%%%%for realtime video and TrakRslt.mat
if ispc %for Windows platform
    rootDir = "C:/Warwick/Autoplex/";
else %isunix or ismac
    rootDir = "~/Autoplex/";
end
useYOLO = false; %if true, call YOLO, otherwise load detection results from JSON

% define cameras to process
%img_range = [1,2,3,4,5,6,7,8,9,10];
img_range = [1,2,3,4,5];

% Choose whether to reload JSON files if already in workspace
reloadJSON = false;
outputVideo = true;

% Select output file name (has to be .avi on Linux)
outputVideoName = "output.avi";

% Choose whether to output polygons or just points
plotPoly = false;

% Select number of frames to output
% framesOut = -1;
framesOut = 20;

% Plot point size
plotSize = 1; % Needs to be 10 for the 4k screens, else 1

% Load existing top-down map into CamraImages/map figure
MapImg_Ms=imread("CamraImages/Mapv4.png");

% filter by bbox size, half_height, triangle. if none, leave empty
filterBBoxSize = "half_height";

% pre-define colour per class
pgon_colours = {}; 
pgon_colours{2} = 'g'; % car
pgon_colours{3} = 'b'; % motorbike
pgon_colours{5} = 'm'; % bus
pgon_colours{7} = 'r'; % truck

colorMap = {};

if useYOLO
    %%%%for realtime video and TrakRslt.mat
    for i=camera_range(1):camera_range(end)
        RefVid{i} = VideoReader(rootDir + "Data Annotation/Transform/Videos/" + "ch" + i + "_compressed.mp4");
        RefVid_nframes{i} = RefVid{i}.NumFrames;
    end
    % Preallocate array, will store YOLO detection results
    JSON_data = cell(1,length(camera_range));
else
    %% Load in json files
    if (~reloadJSON && exist("JSON_data","var"))
        disp("Loading JSON files from workspace")
    else

        disp("Loading JSON files (this might take a moment)"); jsonloadtime = tic();
        % Preallocate array
        JSON_data = cell(1,length(img_range));

        parfor image = img_range
            JSON_data{image} = readJSON("videos/ch"+image+".json");
        end
        disp("JSON files loaded in "+toc(jsonloadtime)+"s");
        fprintf("JSON files loaded in %4.2fs\n",toc(jsonloadtime))
    end
end


%% Load in reference images?
for i = img_range(1):img_range(end)
    RefImg_Ms{i} = imread("CamraImages/clearStills/ch"+i+".jpg");
end

% Get indexes of interest
imagesToProcess = img_range;
    
% Reference 2-D map image to world co-ords
MapoutputView = imref2d(size(MapImg_Ms));

for i = imagesToProcess

    CamraPostn{i}.RefImg_Ms = importdata("CamraImages/clearStills/ch"+i+".jpg");
    % get dimensions of image (width, height)
    CamraPostn{i}.Img(1) = size(CamraPostn{i}.RefImg_Ms,1); % img width
    CamraPostn{i}.Img(2) = size(CamraPostn{i}.RefImg_Ms,2); % img height

    % Load mask
    curImgMask = load("masks/img"+i+"mask.mat").ImgMask;

    Imasked{i} = zeros(size(RefImg_Ms{i}),class(RefImg_Ms{i}));
    Imasked{i} = bsxfun(@times, RefImg_Ms{i}, cast(curImgMask, 'like', RefImg_Ms{i}));
    
    movingPointsFile = "transformPoints/movingPoints_ch"+num2str(i) + ".mat";
    fixedPointsFile = "transformPoints/fixedPoints_ch"+num2str(i) + ".mat";
    
    movingPoints = load(movingPointsFile).movingPoints;
    fixedPoints = load(fixedPointsFile).fixedPoints;

    MapTfrm{i} = fitgeotrans(movingPoints, fixedPoints, 'projective'); %nonreflectivesimilarity, similarity, affine, projective
    warpedImg{i} = imwarp((Imasked{i}), MapTfrm{i}, 'outputView', MapoutputView, 'interp', 'nearest');

end

%create an image with all value is 1
imgNum = uint8(zeros(size(imbinarize(MapImg_Ms))));
imgNum(:) = 1;
imgNum = rgb2gray(imgNum); %greyscale image, vaue 1

%binarize all refImg, and sum up
for i=imagesToProcess
       bw = uint8(im2bw(warpedImg{i}, 0.0001)); %binary image value 0,1
       imgNum = imadd(imgNum, bw); %greyscale image. pixel value is number of non-zero pixels from all images.
end
imgNumRGB = cat(3, imgNum, imgNum, imgNum);

%average multi images
%I = uint32(MapImg_Ms);
I = uint32(warpedImg{imagesToProcess(1)});
for i = imagesToProcess(2:end)
    I = imadd(I, uint32(warpedImg{i})); % each pixel is 32*3 bits
end

%total = index_b-index_a+1+1;  %total number of warped image + 1 map image
I = uint8(imdivide(I, uint32(imgNumRGB))); % most area of warped image is dark, each pixel actually added only 1~5 non-zero values

figure("Name","CombinedImages w/ center");
imshow(I);
title("Overlayed images for cameras "+imagesToProcess);

% remove not required variables for optimisation
clearvars bw imgNum imgNumRGB warpedImg curImgMask Imasked movingPoints fixedPoints RefImg_Ms MapoutputView MapImg_Ms

%% Show image
% imshow(MapImg_Ms);
% title("Overlayed bounding boxes")
% hold on;

%% Initialise video output
if outputVideo
    % create the video writer with 25 fps
    writerObj = VideoWriter(outputVideoName);
    writerObj.FrameRate = 25;
    writerObj.Quality = 100;
    % open the video writer
    open(writerObj);
end
fps = [];

%% Cycle through frames in JSON file
% Assumed that all JSON files have same length, so we use the last one here
if framesOut == -1
    lastFrame = JSON_data{1}(end).frame_id;
else
    lastFrame = framesOut;
end
totaltimecounter = tic();

% initialise tracker
tracker = trackerGNN;

%% will output results to TrakRslt_*.mat
%% Create Table
Id_Z = uint32([]);
BndryBoxHgt_Lpix = [];
BndryBoxWdth_L = [];
VehClassConfd_Pc_Ary = double.empty(0,4);
SmplCnt_Z=[];
TTbleIndx_Z = [];
TrakTble = table(Id_Z,BndryBoxHgt_Lpix,BndryBoxWdth_L,VehClassConfd_Pc_Ary,SmplCnt_Z,TTbleIndx_Z);

%Track Time Table
Time_T = duration.empty(0,1);
ObjPostn_L = double.empty(0,2);
Obj_V = double.empty(0,2);
ObjCov = double.empty(0,18);
EmptyTrak = table(Time_T ,ObjPostn_L,Obj_V,ObjCov);

for frame = 1:lastFrame
    fpscounter = tic();
    fprintf("==================\nStarting to process frame %i\n",frame)

    % For tracker
    detections = {};

    % For other stuff
    pgon = {};
    vehicleClass = int16.empty; % used to filter out by vehicle class
    
    Confd = {}; %% confidence score of each type (car, motobike, bus, truck), will be output to TrakRslt_*.mat

    %% convert 2d bbox from relative to absolute coordinates
    bboxProcessTime = tic();
%     disp("Loading bounding boxes")
    for i=img_range(1):img_range(end)
        % load the txt files
        CamraPostn{i}.pgon = {};
        % get vehicle class

        %%%%for realtime video and TrakRslt.mat
        if useYOLO
            disp(["camera" i "frame" frame " call YOLO to detect vehicles"]);
            %RefImg_Ms{i} = read(RefVid{i}, frame);
            imwrite(read(RefVid{i}, frame), "../darknet/tmp.jpg");
            JSON_data{i}(end+1) = detectVehiclesGetJSONstr; %attache detection result to end frame
        end
        
        CamraPostn{i}.vehicleClass = [JSON_data{i}(frame).objects.class_id]';
    
        % get dimensions of image (width, height)
%         warning("Don't leave the image dimensions hardcoded")
%         CamraPostn{i}.Img(1) = 1920; % img width
%         CamraPostn{i}.Img(2) = 1080; % img height
       
        % extract 2D BBox 
        % YOLO outputs relative coordinates

%         dets = [JSON_data{2}(frame).objects.relative_coordinates];
%         t.BBox = [[dets.center_x]' [dets.center_y]' [dets.width]' [dets.height]'];
%         t.BBox(:,1:2) = [];
%         CamraPostn{i}.BBox2DRel = t.BBox;
        % Convert BBox to absolute cordinates
        dets = [JSON_data{i}(frame).objects.relative_coordinates];
        t.BBoxRelCenterX = [dets.center_x]';
        t.BBoxRelCenterY = [dets.center_y]';
        t.BBoxRelWidth = [dets.width]';
        t.BBoxRelHeight = [dets.height]';
        
        t.BBoxAbsCenterX = CamraPostn{i}.Img(2) .* t.BBoxRelCenterX;
        t.BBoxAbsCenterY = CamraPostn{i}.Img(1) .* t.BBoxRelCenterY;
        t.BBoxAbsWidth = CamraPostn{i}.Img(2) .* t.BBoxRelWidth;
        t.BBoxAbsHeight = CamraPostn{i}.Img(1) .* t.BBoxRelHeight;
    
        % bbox size can be turned into half height or triangle
        if isequal(filterBBoxSize, "triangle")
            t.TopLeft = [t.BBoxAbsCenterX, t.BBoxAbsCenterY];
            t.TopRight = [t.BBoxAbsCenterX, t.BBoxAbsCenterY];
            t.BottomLeft = [t.BBoxAbsCenterX - 0.5 * t.BBoxAbsWidth, t.BBoxAbsCenterY + 0.5 * t.BBoxAbsHeight];
            t.BottomRight = [t.BBoxAbsCenterX + 0.5 * t.BBoxAbsWidth, t.BBoxAbsCenterY + 0.5 * t.BBoxAbsHeight];
        elseif isequal(filterBBoxSize, "half_height")
            t.TopLeft = [t.BBoxAbsCenterX - 0.5 * t.BBoxAbsWidth, t.BBoxAbsCenterY];
            t.TopRight = [t.BBoxAbsCenterX+ 0.5 * t.BBoxAbsWidth, t.BBoxAbsCenterY];
            t.BottomLeft = [t.BBoxAbsCenterX - 0.5 * t.BBoxAbsWidth, t.BBoxAbsCenterY + 0.5 * t.BBoxAbsHeight];
            t.BottomRight = [t.BBoxAbsCenterX + 0.5 * t.BBoxAbsWidth, t.BBoxAbsCenterY + 0.5 * t.BBoxAbsHeight];
        else
            t.TopLeft = [t.BBoxAbsCenterX - 0.5 * t.BBoxAbsWidth, t.BBoxAbsCenterY - 0.5 * t.BBoxAbsHeight];
            t.TopRight = [t.BBoxAbsCenterX + 0.5 * t.BBoxAbsWidth, t.BBoxAbsCenterY - 0.5 * t.BBoxAbsHeight];
            t.BottomLeft = [t.BBoxAbsCenterX - 0.5 * t.BBoxAbsWidth, t.BBoxAbsCenterY + 0.5 * t.BBoxAbsHeight];
            t.BottomRight = [t.BBoxAbsCenterX + 0.5 * t.BBoxAbsWidth, t.BBoxAbsCenterY + 0.5 * t.BBoxAbsHeight];
        end
        
        CamraPostn{i}.BBox2DAbs = [t.TopLeft, t.TopRight, t.BottomLeft, t.BottomRight];
    
        t.x1 = t.BBoxAbsCenterX - 0.5 * t.BBoxAbsWidth;
        t.y1 = t.BBoxAbsCenterY - 0.5 * t.BBoxAbsHeight;
    
        CamraPostn{i}.BBox2Dxywh = [t.x1,t.y1,t.BBoxAbsWidth,t.BBoxAbsHeight];
    
        t.x = [t.TopLeft(:,1),t.TopRight(:,1),t.BottomLeft(:,1),t.BottomRight(:,1)];
        t.y = [t.TopLeft(:,2),t.TopRight(:,2),t.BottomLeft(:,2),t.BottomRight(:,2)];
    
        CamraPostn{i}.BBox2Dxy = [t.x,t.y];

        %% Transform bounding boxes to polygons
%         disp("Transforming bounding boxes ");
        transformingbboxesClock = tic;
    
        warpedVertices{i}.tl = transformPointsForward(MapTfrm{i},CamraPostn{i}.BBox2DAbs(:,1:2));
        warpedVertices{i}.tr = transformPointsForward(MapTfrm{i},CamraPostn{i}.BBox2DAbs(:,3:4));
        warpedVertices{i}.bl = transformPointsForward(MapTfrm{i},CamraPostn{i}.BBox2DAbs(:,5:6));
        warpedVertices{i}.br = transformPointsForward(MapTfrm{i},CamraPostn{i}.BBox2DAbs(:,7:8));
    
        warpedVerticesSize = size(warpedVertices{i}.tl);
        for j = 1:warpedVerticesSize(1)
            pgon{end+1} = polyshape([warpedVertices{i}.tl(j,:);warpedVertices{i}.tr(j,:);warpedVertices{i}.br(j,:);warpedVertices{i}.bl(j,:)]);
            vehicleClass(end+1) = CamraPostn{i}.vehicleClass(j); % used to filter out by class
            Confd{end+1} = JSON_data{i}(frame).objects.confidence; %% confidence score of each type (car, motobike, bus, truck), will be output to TrakRslt_*.mat
        end
%         fprintf("Bounding boxes transformed in %4.2f s\n",toc(transformingbboxesClock))

    end
%     fprintf("Bounding box processing complete in %4.2fs\n",toc(bboxProcessTime))
    

    %% find intersection of polygons and apply filters
    isectarr = {};
    centroids = {};
    isectpgon = [];     % which polygons are involved in intersections
    doublepgon = [];    % which polygons have been used for 2 or more intersections
    isectClass = [];

    isectConfd = {}; %% confidence score of each type (car, motobike, bus, truck), will be output to TrakRslt_*.mat
    boundingBoxSizes = {}; %%%% width height of bounding box (intersect of 2), for realtime video and TrakRslt.mat
    boundingBox3Sizes = {}; %%%% width height of bounding box (intersect of 3), for realtime video and TrakRslt.mat
    hold on;

    disp("Polygon filtering starting")
        disp("Polygon plotting starting")
    polyplot = tic();
    figure(1)
%     for i = 1:length(pgon)
%         for j = 1:length(pgon)
%             if i ~= j
%                 isect = intersect(pgon{i},pgon{j});
% %                 plot(isect);
%                 [intersectx, intersecty] = centroid(isect);
%                 plot(intersectx,intersecty,'+','LineWidth',2);
%                 isectarr{end+1} = isect;
%             end
%         end
%     end
    for i = 1:length(pgon)
        for j = i+1:length(pgon)
            % need to filter out if pgon has already been detection with same
            % combination, j=2, i=14 compared to i=2, j=14
            if i~= j && ~ismember(i, isectpgon) && ~ismember(j,isectpgon)
                if isequal(vehicleClass(i), vehicleClass(j)) % filters out by class
                    if (min(pgon{j}.Vertices(:,1)) > max(pgon{i}.Vertices(:,1))) %filte
                        continue
                    elseif (min(pgon{i}.Vertices(:,1)) > max(pgon{j}.Vertices(:,1))) %filte
                        continue
                    elseif (min(pgon{i}.Vertices(:,2)) > max(pgon{j}.Vertices(:,2)))
                        continue
                    elseif (min(pgon{j}.Vertices(:,2)) > max(pgon{i}.Vertices(:,2)))
                        continue
                    
                    else

                        isect = intersect(pgon{i},pgon{j});
                        % Test to see if new filter mechanism works
%                         if (min(pgon{j}.Vertices(:,1)) > max(pgon{i}.Vertices(:,1)))
%                             if isect.NumRegions == 0
%                                 disp("Match")
%                             else
%                                 disp("False")
%                             end
%                         end
    
                        if isect.NumRegions ~= 0
                            [intersectx, intersecty] = centroid(isect);
            
                            % don't plot if no intersection present
                            if ~isnan(intersectx) &&  ~isnan(intersecty)
        %                         disp("MATCH for bboxes "+i+" and "+j+" in frame "+frame)
                                % due to nature of mask, some detections for camera 1 have negative
                                % pixels
                                try
                                    R = I(round(intersecty),round(intersectx),1);
                                    G = I(round(intersecty),round(intersectx),2);
                                    B = I(round(intersecty),round(intersectx),3);
                                    if R ~= 0 || G~=0 || B~=0
                                        % plot intersection area of polygons
            %                             plot(isect,'FaceColor',pgon_colours{vehicleClass(i)});
                %                         plot(pgon{i}, 'FaceColor',pgon_colours{vehicleClass(i)})
                %                         plot(pgon{j}, 'Facecolor',pgon_colours{vehicleClass(i)})
                
                                        % plot centroid
            %                             img = plot(intersectx,intersecty,'+','LineWidth',10);
                                        centroids{end+1} = [intersectx, intersecty]; 
                                        isectarr{end+1} = isect;
                                        isectClass(end+1) = vehicleClass(i);
                                        isectpgon(end+1) = i;
                                        isectpgon(end+1) = j;
                
                                        %%%%for realtime video and TrakRslt.mat
                                        %pgon is 4*2 array. TL, TR, BR, BL.
                                        %intersected bounding box size is average size of box i and box j
                                        ixMin = min([pgon{i}.Vertices(1,1),pgon{i}.Vertices(2,1),pgon{i}.Vertices(3,1),pgon{i}.Vertices(4,1)]);
                                        ixMax = max([pgon{i}.Vertices(1,1),pgon{i}.Vertices(2,1),pgon{i}.Vertices(3,1),pgon{i}.Vertices(4,1)]);
                                        iyMin = min([pgon{i}.Vertices(1,2),pgon{i}.Vertices(2,2),pgon{i}.Vertices(3,2),pgon{i}.Vertices(4,2)]);
                                        iyMax = max([pgon{i}.Vertices(1,2),pgon{i}.Vertices(2,2),pgon{i}.Vertices(3,2),pgon{i}.Vertices(4,2)]);
                                        iwidth = ixMax-ixMin;
                                        iheight = iyMax-iyMin;
                                        jxMin = min([pgon{j}.Vertices(1,1),pgon{j}.Vertices(2,1),pgon{j}.Vertices(3,1),pgon{j}.Vertices(4,1)]);
                                        jxMax = max([pgon{j}.Vertices(1,1),pgon{j}.Vertices(2,1),pgon{j}.Vertices(3,1),pgon{j}.Vertices(4,1)]);
                                        jyMin = min([pgon{j}.Vertices(1,2),pgon{j}.Vertices(2,2),pgon{j}.Vertices(3,2),pgon{j}.Vertices(4,2)]);
                                        jyMax = max([pgon{j}.Vertices(1,2),pgon{j}.Vertices(2,2),pgon{j}.Vertices(3,2),pgon{j}.Vertices(4,2)]);
                                        jwidth = jxMax-jxMin;
                                        jheight = jyMax-jyMin;
                                        boundingBoxSizes{end+1} = [(iwidth+jwidth)/2,(iheight+jheight)/2];
                                        isectConfd{end+1} = (Confd{i} + Confd{j})/2; %% confidence score of each type (car, motobike, bus, truck), will be output to TrakRslt_*.mat
                                        %at the moment our YOLO JSON
                                        %provide top score only.

                                    end
                                catch ME
                                    continue
                                end
                            end
%                         else
%                             figure;
%                             plot(pgon{i})
%                             hold on;
%                             plot(pgon{j})
%                             disp("Pgons failted to filter for "+i+" and "+j);
                        end
                    end
                end

            end
        end
    end
    fprintf("Polygon plotting complete in %4.2fs\n",toc(polyplot))
%     disp(hits)

    %% filtering with single view detections
    % find single view detections
    singleViewDetections = int16.empty;
    singleViewCenter = {};
    for i = 1:length(pgon)
        if length(isectpgon) - sum(~ismember(isectpgon,i))<1 % if polygon has not been used for intersection of 2 views
            [intersectx, intersecty] = centroid(pgon{i});
            % due to nature of mask, some detections for camera 1 have negative
            % pixels
            try
%                 if ( intersectx > 0 || intersectx < size(I,2) ) && ( intersecty > 0 || intersecty < size(I,1))
                    R = I(round(intersecty),round(intersectx),1);
                    G = I(round(intersecty),round(intersectx),2);
                    B = I(round(intersecty),round(intersectx),3);
                    if R ~= 0 || G~=0 || B~=0   % values need to be inside mask
                        singleViewDetections(end+1) = i;
                        singleViewCenter{end+1} = [intersectx, intersecty];
                    end
%                 end
            catch 
                continue
            end
        end
    end
    
    %% Find center of 3 intersection points
    isectarr3 = {};
    centroids3 = {};
    isectpgon3 = [];  
    
    isectClass3 = []; %%%%for realtime video and TrakRslt.mat
    isectConfd3 = {};
    
    isect2poly = isectarr;
    
    for i = 1:length(isectarr)
        for j = 1:length(singleViewDetections)
            % need to filter out if pgon has already been detection with same
            % combination, j=2, i=14 compared to i=2, j=14
            if i~= j && ~ismember(i, isectpgon3) && ~ismember(j,isectpgon3)
                if isequal(isectClass(i), vehicleClass(j)) % filters out by class
                    isect3 = intersect(isectarr{i},pgon{singleViewDetections(j)});
                    [intersectx3, intersecty3] = centroid(isect3);
    
                    if ~isnan(intersectx3) &&  ~isnan(intersecty3)
                            centroids3{end+1} = [intersectx3, intersecty3]; 
                            isectarr3{end+1} = isect3;
                            
                            isectpgon3(end+1) = i;
                            isectpgon3(end+1) = j;
                                
                            centroids{i} = [];
                            singleViewCenter{j} = [];
                            isect2poly{i} =isect3;

                            %%%%for realtime video and TrakRslt.mat
                            boundingBox3Sizes{end+1} = boundingBoxSizes{i};
                            boundingBoxSizes{i} = [];
                            isectClass3(end+1) = isectClass(i); %%%%for realtime video and TrakRslt.mat
                            isectConfd3{end+1} = isectConfd{i}; %% confidence score of each type (car, motobike, bus, truck), will be output to TrakRslt_*.mat
                            %at the moment our YOLO JSON
                            %provide top score only.
    %                     end
                    end
                end
            end
        end
    end
    fprintf("Polygon filtering complete in %4.2fs\n",toc(polyplot))

    %% Plot to image

        disp("Object plotting starting")
        objplot = tic();
    
        fprintf("Object plotting complete in %4.2fs\n",toc(polyplot))

        %% Convert point detections from this frame from centroids to detections
        disp("Object tracking starting")
        objtrack = tic();
        % Add centroids (intersections between two cameras) to detections
        for centroidIdx = 1:length(centroids)
            % Account for empty detections
            if ~isempty(cell2mat(centroids(centroidIdx)))
                % Investigate adding object class ID?
                % For some reason MATLAB requires 3D coordinates for obj
                % tracking...
                %%%%for realtime video and TrakRslt.mat
                detections{end+1} = objectDetection(frame,...
                    [cell2mat(centroids(centroidIdx)) 0]',...
                    'ObjectAttributes', {boundingBoxSizes(centroidIdx),...
                    isectClass(centroidIdx),...
                    isectConfd(centroidIdx)...
                    });
            end
        end

        % Repeat for centroids between 3 cameras
        for centroid3Idx = 1:length(centroids3)
            % Account for empty detections
            if ~isempty(cell2mat(centroids3(centroid3Idx)))
                %%%%for realtime video and TrakRslt.mat
                detections{end+1} = objectDetection(frame,...
                    [cell2mat(centroids3(centroid3Idx)) 0]',...
                    'ObjectAttributes', {boundingBox3Sizes(centroid3Idx),...
                    isectClass3(centroid3Idx),...
                    isectConfd3(centroid3Idx)...
                    });
            end
        end

%         % Repeat for single view centre
%         for singleViewCenterIdx = 1:length(singleViewCenter)
%             % Account for empty detections
%             if ~isempty(cell2mat(singleViewCenter(singleViewCenterIdx)))
%                 detections{end+1} = objectDetection(frame, [cell2mat(singleViewCenter(singleViewCenterIdx)) 0]');
%             end
%         end

    %% Convert detections to a tracker
    [confirmedTracks,tentativeTracks,allTracks] = tracker(detections,frame);

    fprintf("Object tracking complete in %4.2fs\n",toc(objtrack))

    %% New! Plot the objects from object tracking
    % Object tracking stores actual location data in the format [x; vx; y;
    % vy; z; vz] in the State parameter
    for objectTrackIdx = 1:length(confirmedTracks)
        %For consistent colours across tracks
        if confirmedTracks(objectTrackIdx).TrackID > length(colorMap)
            colorMap{confirmedTracks(objectTrackIdx).TrackID} = rand(1,3);
        end
        if isempty(colorMap{confirmedTracks(objectTrackIdx).TrackID})
            colorMap{confirmedTracks(objectTrackIdx).TrackID} = rand(1,3);
        end
        plot(confirmedTracks(objectTrackIdx).State(1),confirmedTracks(objectTrackIdx).State(3),'+','LineWidth',plotSize,'Color',colorMap{confirmedTracks(objectTrackIdx).TrackID})
        
        %% %%%%for realtime video and TrakRslt.mat
        %% update table
        if any(TrakTble.Id_Z == confirmedTracks(objectTrackIdx).TrackID)
            TrakIndx_B_Ary = TrakTble.Id_Z == confirmedTracks(objectTrackIdx).TrackID; %update every matched track
            for AtrbtIndx_Z = 1:numel(confirmedTracks(objectTrackIdx).ObjectAttributes) %Need to Average the Attributes over the course of the track, holding the object atributes for every frame requires to much memory
                TrakTble(TrakIndx_B_Ary,"SmplCnt_Z")={TrakTble{TrakIndx_B_Ary,"SmplCnt_Z"}+1}; %Update the number of samples, where the cameras overlap there could be multiple samples per time step
                CurVal_Z = TrakTble{TrakIndx_B_Ary,"BndryBoxHgt_Lpix"}; %Boundary Box Average height in pixals
                TrakTble(TrakIndx_B_Ary,"BndryBoxHgt_Lpix")={CurVal_Z + (confirmedTracks(objectTrackIdx).ObjectAttributes{AtrbtIndx_Z}{1,1}{1}(2)-CurVal_Z)/TrakTble{TrakIndx_B_Ary,"SmplCnt_Z"}};
                CurVal_Z = TrakTble{TrakIndx_B_Ary,"BndryBoxWdth_L"}; %Boundary Box Average Length in metres
                TrakTble(TrakIndx_B_Ary,"BndryBoxWdth_L")={CurVal_Z + (confirmedTracks(objectTrackIdx).ObjectAttributes{AtrbtIndx_Z}{1,1}{1}(1)-CurVal_Z)/TrakTble{TrakIndx_B_Ary,"SmplCnt_Z"}};
                %TrakTble(TrakIndx_B_Ary,"VehClassConfd_Pc_Ary") =  VehClassConfd_Pc_Ary; 
                %%VehClassId_Z = (confirmedTracks(objectTrackIdx).ObjectAttributes{AtrbtIndx_Z}{1,4}); %Average of the vehicle class probability 
                VehClassConfd_Pc_Ary = zeros(1,4); %confidence score of each class
                VehType = confirmedTracks(objectTrackIdx).ObjectAttributes{AtrbtIndx_Z}{2}; %attributes{1}{1} positions, {1}{2} type, {1}{3} confd
                switch VehType
                    case 2 %car
                        VehClassConfd_Pc_Ary(1) = confirmedTracks(objectTrackIdx).ObjectAttributes{AtrbtIndx_Z}{3}{1};
                    case 3 %motorbike
                        VehClassConfd_Pc_Ary(2) = confirmedTracks(objectTrackIdx).ObjectAttributes{AtrbtIndx_Z}{3}{1};
                    case 5 %bus
                        VehClassConfd_Pc_Ary(3) = confirmedTracks(objectTrackIdx).ObjectAttributes{AtrbtIndx_Z}{3}{1};
                    otherwise %7 truck
                        VehClassConfd_Pc_Ary(4) = confirmedTracks(objectTrackIdx).ObjectAttributes{AtrbtIndx_Z}{3}{1};
                end
                CurVal_Z = TrakTble{TrakIndx_B_Ary,"VehClassConfd_Pc_Ary"};
                TrakTble(TrakIndx_B_Ary,"VehClassConfd_Pc_Ary") = {CurVal_Z + (VehClassConfd_Pc_Ary-CurVal_Z)/TrakTble{TrakIndx_B_Ary,"SmplCnt_Z"}};
            end

            %update structure
            StrutIndx_Tbl = TrakTble(TrakIndx_B_Ary,"TTbleIndx_Z");
            StrutIndx_Ary = StrutIndx_Tbl{:,1};
            for k = StrutIndx_Ary
                if k % non zero
                    % Add Track to the structure
                    NewTTbleRowIndx_Z =  height(TrakStrut(k).Trak)+1;
                    Temp_Z = triu(confirmedTracks(objectTrackIdx).StateCovariance); %Take the upper part of Cov as diagonally symetrical
                    Temp_Z = Temp_Z([1 7 8 13 14 15 19 20 21 22 26 27 28 29 33 34 35 36]); % Non-Zero values
                    %TrakStrut(k).Trak(NewTTbleRowIndx_Z,:) = {time2num(seconds(fpscounter)),... %Populate Track Data
                    TrakStrut(k).Trak(NewTTbleRowIndx_Z,:) = {seconds(confirmedTracks(objectTrackIdx).UpdateTime),... %Populate Track Data
                       [confirmedTracks(objectTrackIdx).State(1),confirmedTracks(objectTrackIdx).State(3)],... % Position
                       [confirmedTracks(objectTrackIdx).State(2),confirmedTracks(objectTrackIdx).State(4)],... % Velocity
                       Temp_Z}; %State Covariance
                end
            end
        else %%append table
            NewRowIndx_Z = height(TrakTble)+1;
            % Id_Z,BndryBoxHgt_Lpix,BndryBoxWdth_L,VehClassConfd_Pc_Ary,SmplCnt_Z,TTbleIndx_Z
            VehClassId_Z = confirmedTracks(objectTrackIdx).TrackID; %track id generated by trackerGNN, not vehicle type
            VehType = confirmedTracks(objectTrackIdx).ObjectAttributes{1}{2}; %attributes{1}{1} positions, {1}{2} type, {1}{3} confd
            boundb_width = confirmedTracks(objectTrackIdx).ObjectAttributes{1}{1}{1}(1);
            boundb_height = confirmedTracks(objectTrackIdx).ObjectAttributes{1}{1}{1}(2);
            VehClassConfd_Pc_Ary = zeros(1,4); %confidence score of each class
            switch VehType
                case 2 %car
                    VehClassConfd_Pc_Ary(1) = confirmedTracks(objectTrackIdx).ObjectAttributes{1}{3}{1};
                case 3 %motorbike
                    VehClassConfd_Pc_Ary(2) = confirmedTracks(objectTrackIdx).ObjectAttributes{1}{3}{1};
                case 5 %bus
                    VehClassConfd_Pc_Ary(3) = confirmedTracks(objectTrackIdx).ObjectAttributes{1}{3}{1};
                otherwise %7 truck
                    VehClassConfd_Pc_Ary(4) = confirmedTracks(objectTrackIdx).ObjectAttributes{1}{3}{1};
            end
            %VehClassConfd_Pc_Ary(VehClassId_Z) = VehClassConfd_Pc;
            TrakTble(NewRowIndx_Z,:)={VehClassId_Z,...
               boundb_width, boundb_height, ... %bounding box size
               VehClassConfd_Pc_Ary,...
               1,...
               NewRowIndx_Z};
            % Add Track to the structure
            Temp_Z = triu(confirmedTracks(objectTrackIdx).StateCovariance); %Take the upper part of Cov as diagonally symetrical
            Temp_Z = Temp_Z([1 7 8 13 14 15 19 20 21 22 26 27 28 29 33 34 35 36]); % Non-Zero values
            TrakStrut(NewRowIndx_Z).Trak = EmptyTrak; %New Track
            %TrakStrut(NewRowIndx_Z).Trak(1,:) = {time2num(seconds(fpscounter)),... %Populate Track Data
            TrakStrut(NewRowIndx_Z).Trak(1,:) = {seconds(confirmedTracks(objectTrackIdx).UpdateTime),... %Populate Track Data
               [confirmedTracks(objectTrackIdx).State(1),confirmedTracks(objectTrackIdx).State(3)],... % Position
               [confirmedTracks(objectTrackIdx).State(2),confirmedTracks(objectTrackIdx).State(4)],... % Velocity
               Temp_Z}; %State Covariance    
        end
    end

    % Also plot any single view detections - we don't add these to tracker
    % because of uncertainty over location (especially as they are likely
    % to be at the edge of the frame)
    for i=1:length(isect2poly)

        if length(singleViewCenter) >= i
            if ~isempty(singleViewCenter{i})
                plot(singleViewCenter{i}(1),singleViewCenter{i}(2),'^','LineWidth',plotSize,'Color',pgon_colours{isectClass(i)})
            end
        end
    end

    %% Write frame to video
    if outputVideo
        try
            writeVideo(writerObj, getframe(gcf));
        catch ME
            break
        end
    end
    fprintf("\n%4.2f fps\n\n",1/toc(fpscounter))
    fps(end+1) = 1/toc(fpscounter);
end

%%%%for realtime video and TrakRslt.mat
save(strcat("TrakRslt_",num2str(10),".mat"),'TrakTble','TrakStrut') %Save the results

disp(lastFrame+" frames processed in "+toc(totaltimecounter)+"s at an average "+mean(fps)+"fps")
if outputVideo
    disp("Writing video")
    % close the writer object
    close(writerObj);
    disp("Video written")
end
