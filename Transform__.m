
selectPoints = [0 0 0 0 0 0 0 0 0 0]; %select ref points if selectPoints element is 1

RefImg_Ms{1} = imread("NVR_ch1_main_20210819120000_20210819130000-vlcsnap-2021-11-16-12h34m53s112.jpg");
RefImg_Ms{2} = imread("NVR_ch2_main_20210711120000_20210711130000.mp4-vlcsnap-2021-08-18-22h19m29s318.jpg");
RefImg_Ms{3} = imread("NVR_ch3_main_20210819120001_20210819130001-vlcsnap-2021-11-16-12h51m52s887.jpg");
RefImg_Ms{4} = imread("NVR_ch4_main_20210621140001_20210621145916.mp4-vlcsnap-2021-08-19-09h14m48s594.jpg");
RefImg_Ms{5} = imread("NVR_ch5_main_20210819120002_20210819130002-vlcsnap-2021-11-16-12h52m43s665.jpg");
RefImg_Ms{6} = imread("NVR_ch6_main_20210619130002_20210619140002-vlcsnap-2021-08-09-15h03m07s754.jpg");
RefImg_Ms{7} = imread("NVR_ch7_main_20210605110003_20210605120003-vlcsnap-2021-08-04-12h04m53s488.jpg");
RefImg_Ms{8} = imread("NVR_ch8_main_20210619130003_20210619140003-vlcsnap-2021-08-09-15h44m20s478.jpg");
RefImg_Ms{9} = imread("NVR_ch9_main_20210605110004_20210605120004-vlcsnap-2021-07-21-11h01m54s560.jpg");
RefImg_Ms{10} = imread("NVR_ch10_main_20210624100004_20210624110004.mp4-vlcsnap-2021-08-24-09h01m10s597.jpg");
[~, total] = size(RefImg_Ms);

figure;
MapImg_Ms=imread("Mapv2.png");
imshow(MapImg_Ms);
title('Mapv2 Img');

MapoutputView = imref2d(size(MapImg_Ms));

for i=1:total
    RefImgSize{i} = size(RefImg_Ms{i});
    figure;
    imshow(RefImg_Ms{i});
    title("Ref Img " + num2str(i));
    x = 1;
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
            end
        else
            [movingPoints, fixedPoints] = cpselect(RefImg_Ms{i}, MapImg_Ms, 'Wait', true);
            save(movingPointsFile, 'movingPoints')
            save(fixedPointsFile, 'fixedPoints')
        end

        MapTfrm{i} = fitgeotrans(movingPoints, fixedPoints, 'projective'); %nonreflectivesimilarity, similarity, affine, projective
        warpedImg{i} = imwarp(RefImg_Ms{i}, MapTfrm{i}, 'outputView', MapoutputView, 'interp', 'nearest');
        figure;
        imshow(warpedImg{i});
        title("warped "+num2str(i))
        figure
        imshowpair(MapImg_Ms, warpedImg{i}, "falsecolor");
        title("mapped "+num2str(i));

        figure
        imshowpair(MapImg_Ms, warpedImg{i}, "falsecolor");
        [TedX, TedY] = transformPointsForward(MapTfrm{i}, [0 RefImgSize{i}(2)], [0 RefImgSize{i}(1)]);
        TedX = max(0, TedX); TedY = max(0, TedY);
        hold on
        plot(TedX, TedY, 'go', "MarkerFaceColor", "g")
        axis([min(TedX(1), TedX(2)), max(TedX(1), TedX(2)) min(TedY(1), TedY(2)) max(TedY(1), TedY(2))])
        title("Current Transform Accuracy "+num2str(i));
        if selectPoints(i) %need to re-select ref points
            x=input("press Enter or Space to continue, any other letter to re-select reference points...", 's');
        else
            x='';
        end
    end
end


%create an image with all value is 1
img1 = zeros(size(im2bw(MapImg_Ms)));
img1(:) = 1;
%binarize all RefImg, and sum up
for i=1:total
    img1 = imadd(img1, im2bw(warpedImg{i}, 0.0001));
end
figure
imshow(img1);

%average multi images
I = uint32(MapImg_Ms);
for i = 1:total
    I = imadd(I, uint32(warpedImg{i}));
end
%total = index_b-index_a+1+1;  %total number of warped image + 1 map image
I = uint8(I/5); % most area of warped image is dark, each pixel actually added only 1~3 non-zero values
figure
imshow(I);
title("overlaid images");





