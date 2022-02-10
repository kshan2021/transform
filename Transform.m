
selectPoints = [0 0 0 0 0 0 0 0 0 0]; %select ref points if selectPoints element is 1

bVerbose = false; %display more intermediate images?

if ispc %for Windows platform
    rootDir = "C:/Warwick/Autoplex/";
else %isunix or ismac
    rootDir = "~/Autoplex/";
end
RefImg_Ms{1} = imread(rootDir + "Data Annotation/Stills/Clear Road/NVR_ch1_main_20211010080000_20211010090000.mp4-vlcsnap-2022-02-08-10h20m30s042.jpg");
RefImg_Ms{2} = imread(rootDir + "Data Annotation/Stills/Clear Road/NVR_ch2_main_20211010080000_20211010090000.mp4-vlcsnap-2022-02-08-10h37m38s785.jpg");
RefImg_Ms{3} = imread(rootDir + "Data Annotation/Stills/Clear Road/NVR_ch3_main_20211010080001_20211010090001.mp4-vlcsnap-2022-02-08-10h38m11s392.jpg");
RefImg_Ms{4} = imread(rootDir + "Data Annotation/Stills/Clear Road/NVR_ch4_main_20211010080001_20211010090001.mp4-vlcsnap-2022-02-08-10h38m39s088.jpg");
RefImg_Ms{5} = imread(rootDir + "Data Annotation/Stills/Clear Road/NVR_ch5_main_20211010080002_20211010090002.mp4-vlcsnap-2022-02-08-10h39m35s506.jpg");
RefImg_Ms{6} = imread(rootDir + "Data Annotation/Stills/Clear Road/NVR_ch6_main_20211010080002_20211010090002.mp4-vlcsnap-2022-02-08-10h40m03s571.jpg");
RefImg_Ms{7} = imread(rootDir + "Data Annotation/Stills/Clear Road/NVR_ch7_main_20211010080003_20211010090003.mp4-vlcsnap-2022-02-08-10h40m25s335.jpg");
RefImg_Ms{8} = imread(rootDir + "Data Annotation/Stills/Clear Road/NVR_ch8_main_20211010080003_20211010090003.mp4-vlcsnap-2022-02-08-10h40m42s412.jpg");
RefImg_Ms{9} = imread(rootDir + "Data Annotation/Stills/Clear Road/NVR_ch9_main_20211010080004_20211010090004.mp4-vlcsnap-2022-02-08-10h40m57s792.jpg");
RefImg_Ms{10}= imread(rootDir + "Data Annotation/Stills/Clear Road/NVR_ch10_main_20211010080004_20211010090004.mp4-vlcsnap-2022-02-08-10h41m14s910.jpg");
[~, total] = size(RefImg_Ms);

fig_map=figure('Name','Map', 'NumberTitle','off');
MapImg_Ms=imread("Mapv2.png");
imshow(MapImg_Ms);


MapoutputView = imref2d(size(MapImg_Ms));

for i=1:total
    RefImgSize{i} = size(RefImg_Ms{i});

    if bVerbose
        figure('Name', "Ref Img " + num2str(i), 'NumberTitle', 'off');
        imshow(RefImg_Ms{i});
        title();
    end
    
    x = 1; %'' selection completed, 0 re-select, 1 new select
    while ~isempty(x) %re-select ref points until user press Enter or Space
        movingPointsFile = "movingPoints_ch"+num2str(i) + ".mat";
        fixedPointsFile = "fixedPoints_ch"+num2str(i) + ".mat";
        clear movingPoints fixedPoints;
        if isfile(movingPointsFile) && isfile(fixedPointsFile) %ref points already available
            if selectPoints(i) %need to re-select ref points
                movingPoints_old = load(movingPointsFile).movingPoints;
                fixedPoints_old = load(fixedPointsFile).fixedPoints;
                [movingPoints, fixedPoints] = cpselect(RefImg_Ms{i}, MapImg_Ms, movingPoints_old, fixedPoints_old, 'Wait', true);
                if ~isequal(movingPoints, movingPoints_old)
                    save(movingPointsFile, 'movingPoints')
                end
                if ~isequal(fixedPoints, fixedPoints_old)
                    save(fixedPointsFile, 'fixedPoints')
                end
            else %do not re-select ref points
                movingPoints = load(movingPointsFile).movingPoints;
                fixedPoints = load(fixedPointsFile).fixedPoints;
                movingPoints_old = movingPoints; fixedPoints_old = fixedPoints; %no need to re-select
            end
        else
            [movingPoints, fixedPoints] = cpselect(RefImg_Ms{i}, MapImg_Ms, 'Wait', true);
            save(movingPointsFile, 'movingPoints')
            save(fixedPointsFile, 'fixedPoints')
            movingPoints_old = [0 0]; fixedPoints_old = [0 0]; %new selection, will ask for re-select to confirm
        end

        MapTfrm{i} = fitgeotrans(movingPoints, fixedPoints, 'projective'); %nonreflectivesimilarity, similarity, affine, projective
        warpedImg{i} = imwarp(RefImg_Ms{i}, MapTfrm{i}, 'outputView', MapoutputView, 'interp', 'nearest');
        if x==1 %new selection, create new figure
            fig_warped = figure('Name',"warped "+num2str(i), 'NumberTitle','off');
        else % x=0, re-select, use old figure
            figure(fig_warped);
        end
        imshow(warpedImg{i});
        if x==1
            fig_mapped = figure('Name',"mapped "+num2str(i), 'NumberTitle','off');
        else
            figure(fig_mapped)
        end
        imshowpair(MapImg_Ms, warpedImg{i}, "falsecolor");

        if bVerbose
            if x==1
                fig_accuracy = figure('Name',"Current Transform Accuracy "+num2str(i), 'NumberTitle','off');
            else
                figure(fig_accuracy)
            end
            imshowpair(MapImg_Ms, warpedImg{i}, "falsecolor");
            [TedX, TedY] = transformPointsForward(MapTfrm{i}, [0 RefImgSize{i}(2)], [0 RefImgSize{i}(1)]);
            TedX = max(0, TedX); TedY = max(0, TedY);
            hold on
            plot(TedX, TedY, 'go', "MarkerFaceColor", "g")
            axis([min(TedX(1), TedX(2)), max(TedX(1), TedX(2)) min(TedY(1), TedY(2)) max(TedY(1), TedY(2))])
        end

        %if selectPoints(i) %need to re-select ref points
        %    x=input("press Enter or Space to continue, any other letter to re-select reference points...", 's');
        %else
        %    x='';
        %end
        
        if isequal(movingPoints, movingPoints_old) && isequal(fixedPoints, fixedPoints_old)
            x=''; %selection completed
        else
            x=0; %need to select again
        end        
    end
end

%create an image with all value is 1
imgNum = uint8(zeros(size(imbinarize(MapImg_Ms))));
imgNum(:) = 1;
imgNum = rgb2gray(imgNum); %greyscale image, vaue 1
%binarize all refImg, and sum up
for i=1:total
       bw = uint8(im2bw(warpedImg{i}, 0.0001)); %binary image value 0,1
       imgNum = imadd(imgNum, bw); %greyscale image. pixel value is number of non-zero pixels from all images.
end
imgNumRGB = cat(3, imgNum, imgNum, imgNum);

%average multi images
I = uint32(MapImg_Ms);
for i = 1:total
    I = imadd(I, uint32(warpedImg{i})); % each pixel is 32*3 bits
end
%total = index_b-index_a+1+1;  %total number of warped image + 1 map image
I = uint8(imdivide(I, uint32(imgNumRGB))); % most area of warped image is dark, each pixel actually added only 1~5 non-zero values
figure;
imshow(I);
title("overlaid images");





