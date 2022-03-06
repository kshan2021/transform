
%load video

%for loop (process each frame of all camera)
%for loop (one same frame of each video)
%call YOLO to detect bounding box
%transformedBoundingBox = TransformBoundingBox(*boundingBox, *cameraNum)
%[transformedBoundingBox, cameraNum] = push (transformedBoundingBox, cameraNum)

%IntersectBoxes(*[transformedBoundingBox, cameraNum, class])
%display boundingBox


%function transformedBoundingBox = TransformBoundingBox(*boundingBox, *cameraNum)

%function IntersectBoxes(*[transformedBoundingBox, cameraNum])
%for loop all boxes of Camera 2 and 3
%check all boxes of camera 2 and 3. if overlapped, fine-tune boundingBox by intersection

if ispc %for Windows platform
    rootDir = "C:/Warwick/Autoplex/";
else %isunix or ismac
    rootDir = "~/Autoplex/";
end


% define video range
img_range = [1,2];

MapImg_Ms = imread("Mapv4.png");

% filter by bbox size, half_height, triangle. if none, leave empty
filterBBoxSize = "triangle";

show2DBBox = false;
showWarpedImage = false;
showGoogleOverlay = false;
showCombinedImages = true;

%frame by frame processing of video files
disp("Preloading video files"); tic;
for image = img_range
    RefVid{image} = VideoReader(rootDir + "Data Annotation/Transform/Videos/" + "ch" + image + "_compressed.mp4");
    RefVid_nframes{image} = RefVid{image}.NumFrames;
end
disp("Video files loaded in "+toc+"s");

%call YOLO if json file unavailalbe
disp("Loading JSON files"); jsonloadtime = tic();
for image = img_range
    JSON_data{image} = readJSON(rootDir + "Data Annotation/Transform/Videos/" + "ch" + image + ".json");
end
disp("JSON files loaded in "+toc(jsonloadtime)+"s");
fprintf("JSON files loaded in %4.2fs\n", toc(jsonloadtime))

%for each camera, transform matrix are same for all frames
for i=img_range(1):img_range(end)
    movingPointsFile = "transformPoints/movingPoints_ch"+num2str(i) + ".mat";
    fixedPointsFile = "transformPoints/fixedPoints_ch"+num2str(i) + ".mat";
    movingPoints = load(movingPointsFile).movingPoints;
    fixedPoints = load(fixedPointsFile).fixedPoints;
    MapTfrm{i} = fitgeotrans(movingPoints, fixedPoints, 'projective'); %nonreflectivesimilarity, similarity, affine, projective
end

%for frame = 1:RefVid_nframes{img_range(1)} %assume all video have same img_range
for frame = 1:20 %assume all video have same img_range
    fps = tic();
    fprintf("===========\nStarting to process frame %i\n", frame)
    for i = img_range(1):img_range(end) %same frame from each video
        RefImg_Ms{i} = read(RefVid{i}, frame);
    end
    
    idxs = ~cellfun('isempty', RefImg_Ms);
    imagesToProcess = find(idxs == 1);
    
    MapoutputView = imref2d(size(MapImg_Ms));
    
	figure(1);
	
    pgon = {};
    vehicleClass = {}; %to filter out by vehicle class
    
    %% convert 2d bbox from relative to absolute coordinates
    bboxProcessTime = tic();
    disp("Loading bounding boxes")
    for i=img_range(1):img_range(end)
        % load the jpg and txt files
        CamraPostn{i}.RefImg_Ms = RefImg_Ms{i};
        %CamraPostn{i}.Detections = importdata("Detections/cam"+i+"_" +vehicleType+".txt");
        CamraPostn{i}.pgon = {};
        % get vehicle class
        CamraPostn{i}.vehicleClass = [JSON_data{i}(frame).objects.class_id]';
    
        % get dimensions of image (width, height)
        CamraPostn{i}.Img(1) = size(CamraPostn{i}.RefImg_Ms,1); % img width
        CamraPostn{i}.Img(2) = size(CamraPostn{i}.RefImg_Ms,2); % img height
       
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
        CamraPostn{i}.Centers = [t.BBoxAbsCenterX, t.BBoxAbsCenterY];
    
        t.x1 = t.BBoxAbsCenterX - 0.5 * t.BBoxAbsWidth;
        t.y1 = t.BBoxAbsCenterY - 0.5 * t.BBoxAbsHeight;
    
        CamraPostn{i}.BBox2Dxywh = [t.x1,t.y1,t.BBoxAbsWidth,t.BBoxAbsHeight];
    
        t.x = [t.TopLeft(:,1),t.TopRight(:,1),t.BottomLeft(:,1),t.BottomRight(:,1)];
        t.y = [t.TopLeft(:,2),t.TopRight(:,2),t.BottomLeft(:,2),t.BottomRight(:,2)];
    
        CamraPostn{i}.BBox2Dxy = [t.x,t.y];
    end
    fprintf("Bounding box processing complete in %4.2fs\n",toc(bboxProcessTime))
    %% Mask out images
    disp("Transforming images ");
    transformingImagesClock = tic;
    for i = imagesToProcess
        % Load mask
        curImgMask = load("masks/img"+i+"mask.mat").ImgMask;
    
        Imasked{i} = zeros(size(RefImg_Ms{i}),class(RefImg_Ms{i}));
        Imasked{i} = bsxfun(@times, RefImg_Ms{i}, cast(curImgMask, 'like', RefImg_Ms{i}));
        
        warpedImg{i} = imwarp((Imasked{i}), MapTfrm{i}, 'outputView', MapoutputView, 'interp', 'nearest');
        
        warpedVertices{i}.tl = transformPointsForward(MapTfrm{i},CamraPostn{i}.BBox2DAbs(:,1:2));
        warpedVertices{i}.tr = transformPointsForward(MapTfrm{i},CamraPostn{i}.BBox2DAbs(:,3:4));
        warpedVertices{i}.bl = transformPointsForward(MapTfrm{i},CamraPostn{i}.BBox2DAbs(:,5:6));
        warpedVertices{i}.br = transformPointsForward(MapTfrm{i},CamraPostn{i}.BBox2DAbs(:,7:8));
        warpedVertices{i}.center = transformPointsForward(MapTfrm{i},CamraPostn{i}.Centers);
    
        for j = 1:length(warpedVertices{i}.tl)
            pgon{end+1} = polyshape([warpedVertices{i}.tl(j,:);warpedVertices{i}.tr(j,:);warpedVertices{i}.br(j,:);warpedVertices{i}.bl(j,:)]);
            vehicleClass{end+1} = CamraPostn{i}.vehicleClass(j); % used to filter out by class
    
        end
    
        if showWarpedImage
            figure;
            imshow(warpedImg{i}); hold on;
            %plot(warpedVertices{i}.tl(:,1),warpedVertices{i}.tl(:,2),'r+');
            %plot(warpedVertices{i}.tr(:,1),warpedVertices{i}.tr(:,2),'r+');
            %plot(warpedVertices{i}.bl(:,1),warpedVertices{i}.bl(:,2),'r+');
            %plot(warpedVertices{i}.br(:,1),warpedVertices{i}.br(:,2),'r+');
            plot(warpedVertices{i}.center(:,1), warpedVertices{i}.center(:,2), 'y+');
            title("Warped Image "+i);
     
    
        end
    
        if showGoogleOverlay
            figure;
            imshowpair(MapImg_Ms, warpedImg{i}, "falsecolor");
            title("Warped Image "+i+" overlayed with Google Maps")
        end
    end
    fprintf("Images transformed in %4.2f s\n",toc(transformingImagesClock))

    disp("Beginning final image stitch")
    imagestitch = tic();
    %create an image with all value is 1
    imgNum = uint8(zeros(size(imbinarize(MapImg_Ms))));
    imgNum(:) = 1;
    imgNum = rgb2gray(imgNum); %greyscale image, the pixel value will indicate how many images at this pixel is non-zero
    
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
   
    %mark all vehicles
    hold on;
    for i = imagesToProcess %for each camera
        radius = ones(size(warpedVertices{i}.center(:,1)))*15;
        I = insertShape(I, 'FilledCircle', [warpedVertices{i}.center(:,1), warpedVertices{i}.center(:,2), radius], 'LineWidth', 5, 'color', [255 255 0]);        
        %plot(warpedVertices{i}.center(:,1), warpedVertices{i}.center(:,2), 'y+');
    end    
    
    if showCombinedImages
        figure(1);
        imshow(I);
        title("Frame "+frame+": Overlayed images for cameras "+imagesToProcess);
        axis on
    end
    fprintf("Image stitch complete in %4.2fs\n",toc(imagestitch))

    
   
    %% find intersection of polygons and apply filters
    isectarr = {};
    isectpgon = [];
    doublepgon = [];
    hold on;
    disp("Polygon plotting starting")
    polyplot = tic();
%     for i = 1:length(pgon)
%         for j = 1:length(pgon)
%             if i ~= j
%                 isect = intersect(pgon{i},pgon{j});
%                 plot(isect);
%                 [intersectx, intersecty] = centroid(isect);
%                 plot(intersectx,intersecty,'+','LineWidth',10);
%                 isectarr{end+1} = isect;
%             end
%         end
%     end

    for i = 1:length(pgon)
        for j = 1:length(pgon)
            % need to filter out if pgon has already been detection with same
            % combination, j=2, i=14 compared to i=2, j=14
            if i~= j && ~ismember(i, isectpgon) && ~ismember(j,isectpgon)
                if isequal(vehicleClass(i), vehicleClass(j)) % filters out by class
                    isect = intersect(pgon{i},pgon{j});
                    [intersectx, intersecty] = centroid(isect);
    
                    % only plot centroid and intersection area if within the mask I
                    if ~isnan(intersectx) &&  ~isnan(intersecty) && intersectx > 0 && intersecty > 0
                        disp(round(intersectx) + "," + round(intersecty))
                        
    
                        if I(round(intersecty),round(intersectx)) ~= 0
                            % plot intersection area of polygons
                            figure(1)
                            plot(isect);
                            plot(pgon{i})
                            plot(pgon{j})
    
                            % plot centroid
                            img = plot(intersectx,intersecty,'+','LineWidth',10);
                            isectarr{end+1} = isect;
                            
                            % as you get double detections, need to remember
                            % if polygon detection have already been done.
                            if ismember(i, isectpgon)
                                doublepgon(end+1) = i;
                            else
                                isectpgon(end+1) = i;
                            end
    
                            if ismember(j, isectpgon)
                                doublepgon(end+1) = j;
                            else
                                isectpgon(end+1) = j;
                            end
                        
                        end
                    end
                end
            end
        end
    end
    fprintf("Polygon plotting complete in %4.2fs\n",toc(polyplot))
    fprintf("\n%4.2f fps\n\n",1/toc(fps))
    pause(0.1)
        
end
    
    
    
    






%{
bVideo = true; %input videos, otherwise input images
bVerbose = false; %display more intermediate images?
bDetectBoundingBox = true; %detect and show vehicles' bounding box if true
selectPoints = [0 0 0 0 0 0 0 0 0 0]; % call cpselect to edit (or create) ref points if 1

%camera images with vehicles, to test performance of transformation
imgFilename{1} = rootDir + "Data Annotation/Datasets/e0804-dataset/Annotated-Manual-Ground-Truth-e0804/2021-06-05-1100/NVR_ch1_main_20210605110000_20210605120000-vlcsnap-2021-07-21-11h19m24s383.jpg";
imgFilename{2} = rootDir + "Data Annotation/Datasets/e0804-dataset/Annotated-Manual-Ground-Truth-e0804/2021-06-05-1100/NVR_ch2_main_20210605110000_20210605120000-vlcsnap-2021-07-21-11h17m52s327.jpg";
imgFilename{3} = rootDir + "Data Annotation/Datasets/e0804-dataset/Annotated-Manual-Ground-Truth-e0804/2021-06-05-1100/NVR_ch3_main_20210605110001_20210605120001-vlcsnap-2021-07-21-11h16m39s324.jpg";
imgFilename{4} = rootDir + "Data Annotation/Datasets/e0804-dataset/Annotated-Manual-Ground-Truth-e0804/2021-06-05-1100/NVR_ch4_main_20210605110001_20210605120001-vlcsnap-2021-07-21-11h15m33s743.jpg";
imgFilename{5} = rootDir + "Data Annotation/Datasets/e0804-dataset/Annotated-Manual-Ground-Truth-e0804/2021-06-05-1100/NVR_ch5_main_20210605110002_20210605120002-vlcsnap-2021-07-21-11h08m41s927.jpg";
imgFilename{6} = rootDir + "Data Annotation/Datasets/e0804-dataset/Annotated-Manual-Ground-Truth-e0804/2021-06-05-1100/NVR_ch6_main_20210605110002_20210605120002-vlcsnap-2021-07-21-11h06m59s770.jpg";
imgFilename{7} = rootDir + "Data Annotation/Datasets/e0804-dataset/Annotated-Manual-Ground-Truth-e0804/2021-06-05-1100/NVR_ch7_main_20210605110003_20210605120003-vlcsnap-2021-07-21-11h05m40s555.jpg";
imgFilename{8} = rootDir + "Data Annotation/Datasets/e0804-dataset/Annotated-Manual-Ground-Truth-e0804/2021-06-05-1100/NVR_ch8_main_20210605110003_20210605120003-vlcsnap-2021-07-21-11h04m34s856.jpg";
imgFilename{9} = rootDir + "Data Annotation/Video Annotation/ch9_vlcsnap-2022-02-17-11h33m39s755.jpg";
%imgFilename{10}= rootDir + "Data Annotation/Datasets/t0824-dataset/Annotated-Manual-Ground-Truth-t0824s/2021-07-15-1000/NVR_ch10_main_20210715100004_20210715110004.mp4-vlcsnap-2021-08-22-22h02m57s188.jpg";
imgFilename{10}= rootDir + "Data Annotation/Datasets/t0824-dataset/Annotated-Manual-Ground-Truth-t0824s/2021-07-15-1000/NVR_ch10_main_20210715100004_20210715110004.mp4-vlcsnap-2021-08-22-22h06m25s943.jpg";
% ch9 ch10 images size changed after e0804 collected. 2021-06-05 is old size, 2021-07-15 is new size

[~, total] = size(imgFilename); %total number of cameras

%read camera images
for i=1:total
    RefImg_Ms{i} = imread(imgFilename{i});
end

fig_map=figure('Name','Map', 'NumberTitle','off');
MapImg_Ms=imread("Mapv4.png");
imshow(MapImg_Ms);

MapoutputView = imref2d(size(MapImg_Ms));

for i=1:total
    RefImgSize{i} = size(RefImg_Ms{i});

    if bVerbose
        figure('Name', "Ref Img " + num2str(i), 'NumberTitle', 'off');
        imshow(RefImg_Ms{i});
    end
    
    bRefPointsSelectFinished = false; %false: Ref Points need to be re-selected and fine-tuned. true: Ref Points finalised.
    bFigureCreated = false; %use old figure if it's already created before
    while ~bRefPointsSelectFinished %re-select ref points until user didn't change anything in cpselect
        clear movingPoints fixedPoints;
        movingPointsFile = "movingPoints_ch"+num2str(i) + ".mat";
        fixedPointsFile = "fixedPoints_ch"+num2str(i) + ".mat";
        if isfile(movingPointsFile) && isfile(fixedPointsFile) %ref points already available
            if selectPoints(i) %need to edit ref points
                movingPoints_old = load(movingPointsFile).movingPoints;
                fixedPoints_old = load(fixedPointsFile).fixedPoints;
                [movingPoints, fixedPoints] = cpselect(RefImg_Ms{i}, MapImg_Ms, movingPoints_old, fixedPoints_old, 'Wait', true);
                if ~isequal(movingPoints, movingPoints_old)
                    save(movingPointsFile, 'movingPoints')
                end
                if ~isequal(fixedPoints, fixedPoints_old)
                    save(fixedPointsFile, 'fixedPoints')
                end
                if ((movingPoints_old == movingPoints) & (fixedPoints_old == fixedPoints))
                    bRefPointsSelectFinished = true; %Ref points not changed, selection is completed
                else
                    bRefPointsSelectFinished = false; %Ref points changed, need to re-select
                end
            else %do not re-select ref points
                movingPoints = load(movingPointsFile).movingPoints;
                fixedPoints = load(fixedPointsFile).fixedPoints;
                bRefPointsSelectFinished = true; %selection is completed
            end
        else %Ref points not available, need to select new Ref points
            [movingPoints, fixedPoints] = cpselect(RefImg_Ms{i}, MapImg_Ms, 'Wait', true);
            save(movingPointsFile, 'movingPoints')
            save(fixedPointsFile, 'fixedPoints')
            movingPoints_old = [0 0]; fixedPoints_old = [0 0]; %new selection, will ask for re-select to confirm
            bRefPointsSelectFinished = false; %need to re-select
        end

        MapTfrm{i} = fitgeotrans(movingPoints, fixedPoints, 'projective'); %nonreflectivesimilarity, similarity, affine, projective
        warpedImg{i} = imwarp(RefImg_Ms{i}, MapTfrm{i}, 'outputView', MapoutputView, 'interp', 'nearest');

        if bDetectBoundingBox && bRefPointsSelectFinished == true %cpselect completed, detect vheicle and get bounding box
            copyfile(imgFilename{i}, "~/GitHub/darknet/tmp.jpg");
            vehicles = detectVehiclesGetCoord;
            [numVehicles, ~]=size(vehicles);
            for k=1:numVehicles
                %if vehicles(k).confidence<0.5
                %    continue;
                %end
                coord = vehicles(k).relative_coordinates;
                coord.center_x = coord.center_x * RefImgSize{i}(2);
                coord.center_y = coord.center_y * RefImgSize{i}(1);
                coord.width = coord.width * RefImgSize{i}(2);
                coord.height = coord.height * RefImgSize{i}(1);
                %vehicle size
                x1 = coord.center_x - coord.width/2;
                x2 = coord.center_x + coord.width/2;
                y1 = coord.center_y - coord.height/2;
                y2 = coord.center_y + coord.height/2;
                r=(abs(x2-x1)+abs(y2-y1))/2/2;
                %transform x1 x2 y1 y2 to map
                X = [x1 x1 x2 x2 coord.center_x]; Y = [y1 y2 y1 y2 coord.center_y];
                [U V] = transformPointsForward(MapTfrm{i}, X, Y);
                %after transform, rectangle become parallelgram in map
                U_width = (abs(U(3)-U(1)) + abs(U(4)-U(2)))/2;
                V_height = (abs(V(4)-V(3)) + abs(V(2)-V(1)))/2;
                radius = (U_width+V_height)/2/2;
                U_center = U(5); V_center = V(5);
                RefImg_Ms{i} = insertShape(RefImg_Ms{i}, 'rectangle', [x1,y1,coord.width,coord.height], 'LineWidth', 5);
                warpedImg{i} = insertShape(warpedImg{i}, 'FilledCircle', [U_center, V_center, radius], 'LineWidth', 5, 'color', [255 255 0]);
            end
            figure;
            imshow(RefImg_Ms{i});
        end
        
        if bFigureCreated == false %new selection, create new figure
            fig_warped = figure('Name',"warped "+num2str(i), 'NumberTitle','off');
            fig_mapped = figure('Name',"mapped "+num2str(i), 'NumberTitle','off');
            if bVerbose
                fig_accuracy = figure('Name',"Current Transform Accuracy "+num2str(i), 'NumberTitle','off');
            end
        end
        figure(fig_warped); imshow(warpedImg{i});
        figure(fig_mapped); imshowpair(MapImg_Ms, warpedImg{i}, "falsecolor");
        if bVerbose
            figure(fig_accuracy);
            imshowpair(MapImg_Ms, warpedImg{i}, "falsecolor");
            [TedX, TedY] = transformPointsForward(MapTfrm{i}, [0 RefImgSize{i}(2)], [0 RefImgSize{i}(1)]);
            TedX = max(0, TedX); TedY = max(0, TedY);
            hold on
            plot(TedX, TedY, 'go', "MarkerFaceColor", "g")
            axis([min(TedX(1), TedX(2)), max(TedX(1), TedX(2)) min(TedY(1), TedY(2)) max(TedY(1), TedY(2))])
        end
        bFigureCreated = true; %use old figure in next loop
    end
end

%create an image with all value is 0
imgNum = uint8(zeros(size(imbinarize(MapImg_Ms))));
%imgNum(:) = 1;
imgNum = rgb2gray(imgNum); %greyscale image, the pixel value will indicate how many images at this pixel is non-zero
%binarize all refImg, and sum up
for i=1:total
       bw = uint8(im2bw(warpedImg{i}, 0.0001)); %binary image value 0,1
       imgNum = imadd(imgNum, bw); %greyscale image. pixel value is number of non-zero pixels from all images.
end
imgNumRGB = cat(3, imgNum, imgNum, imgNum);

%average all warped map images
I = uint32(zeros(size(MapImg_Ms)));
for i = 1:total
    I = imadd(I, uint32(warpedImg{i})); % each pixel is 32*3 bits
end
%total = index_b-index_a+1+1;  %total number of warped image + 1 map image
I = uint8(imdivide(I, uint32(imgNumRGB))); % most area of warped image is dark, each pixel actually added only 1~5 non-zero values
figure;
imshow(I);
title("overlaid images");
%}





