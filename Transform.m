
if ispc %for Windows platform
    rootDir = "C:/Warwick/Autoplex/";
else %isunix or ismac
    rootDir = "~/Autoplex/";
end

bVerbose = false; %display more intermediate images?
bDetectBoundingBox = false; %detect and show vehicles' bounding box if true
selectPoints = [0 0 0 0 0 0 0 0 0 0]; % call cpselect to edit (or create) ref points if 1
%{
%clear road without vehicle, suitable for cpselect
imgFilename{1} = rootDir + "Data Annotation/Stills/Clear Road/NVR_ch1_main_20211010080000_20211010090000.mp4-vlcsnap-2022-02-08-10h20m30s042.jpg";
imgFilename{2} = rootDir + "Data Annotation/Stills/Clear Road/NVR_ch2_main_20211010080000_20211010090000.mp4-vlcsnap-2022-02-08-10h37m38s785.jpg";
imgFilename{3} = rootDir + "Data Annotation/Stills/Clear Road/NVR_ch3_main_20211010080001_20211010090001.mp4-vlcsnap-2022-02-08-10h38m11s392.jpg";
imgFilename{4} = rootDir + "Data Annotation/Stills/Clear Road/NVR_ch4_main_20211010080001_20211010090001.mp4-vlcsnap-2022-02-08-10h38m39s088.jpg";
imgFilename{5} = rootDir + "Data Annotation/Stills/Clear Road/NVR_ch5_main_20211010080002_20211010090002.mp4-vlcsnap-2022-02-08-10h39m35s506.jpg";
imgFilename{6} = rootDir + "Data Annotation/Stills/Clear Road/NVR_ch6_main_20211010080002_20211010090002.mp4-vlcsnap-2022-02-08-10h40m03s571.jpg";
imgFilename{7} = rootDir + "Data Annotation/Stills/Clear Road/NVR_ch7_main_20211010080003_20211010090003.mp4-vlcsnap-2022-02-08-10h40m25s335.jpg";
imgFilename{8} = rootDir + "Data Annotation/Stills/Clear Road/NVR_ch8_main_20211010080003_20211010090003.mp4-vlcsnap-2022-02-08-10h40m42s412.jpg";
imgFilename{9} = rootDir + "Data Annotation/Stills/Clear Road/NVR_ch9_main_20211010080004_20211010090004.mp4-vlcsnap-2022-02-08-10h40m57s792.jpg";
imgFilename{10}= rootDir + "Data Annotation/Stills/Clear Road/NVR_ch10_main_20211010080004_20211010090004.mp4-vlcsnap-2022-02-08-10h41m14s910.jpg";
%}

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

	fig_warped = figure('Name',"warped "+num2str(i), 'NumberTitle','off');
	fig_mapped = figure('Name',"mapped "+num2str(i), 'NumberTitle','off');

    if bVerbose
        fig_accuracy = figure('Name',"Current Transform Accuracy "+num2str(i), 'NumberTitle','off');
        figure('Name', "Ref Img " + num2str(i), 'NumberTitle', 'off');
        imshow(RefImg_Ms{i});
    end
    
    bRefPointsSelectFinished = false; %false: Ref Points need to be re-selected and fine-tuned. true: Ref Points finalised.
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





