index_a = 1; index_b = 5;
RefImg_Ms{1} = imread("NVR_ch1_main_20210819120000_20210819130000-vlcsnap-2021-11-16-12h34m53s112.jpg");
RefImg_Ms{2} = imread("NVR_ch2_main_20210711120000_20210711130000.mp4-vlcsnap-2021-08-18-22h19m29s318.jpg");
RefImg_Ms{3} = imread("NVR_ch3_main_20210819120001_20210819130001-vlcsnap-2021-11-16-12h51m52s887.jpg");
RefImg_Ms{4} = imread("NVR_ch4_main_20210621140001_20210621145916.mp4-vlcsnap-2021-08-19-09h14m48s594.jpg");
RefImg_Ms{5} = imread("NVR_ch5_main_20210819120002_20210819130002-vlcsnap-2021-11-16-12h52m43s665.jpg");

figure;
MapImg_Ms=imread("Mapv2.png");
imshow(MapImg_Ms);
title('Mapv2 Img');

MapoutputView = imref2d(size(MapImg_Ms));

for i=index_a:index_b
    RefImgSize{i} = size(RefImg_Ms{i});
    figure;
    imshow(RefImg_Ms{i});
    title("Ref Img " + num2str(i));
    x = 1;
    while ~isempty(x) %re-select ref points until user press Enter or Space
        movingPointsFile = "movingPoints_ch"+num2str(i) + ".mat";
        fixedPointsFile = "fixedPoints_ch"+num2str(i) + ".mat";
        clear movingPoints fixedPoints;
        if isfile(movingPointsFile) && isfile(fixedPointsFile)
            movingPoints_old = load(movingPointsFile).movingPoints;
            fixedPoints_old = load(fixedPointsFile).fixedPoints;
            [movingPoints, fixedPoints] = cpselect(RefImg_Ms{i}, MapImg_Ms, movingPoints_old, fixedPoints_old, 'Wait', true);
            if ~isequal(movingPoints, movingPoints_old)
                save(movingPointsFile, 'movingPoints')
            end
            if ~isequal(fixedPoints, fixedPoints_old)
                save(fixedPointsFile, 'fixedPoints')
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
        x=input("press Enter or Space to continue, any other letter to re-select reference points...", 's');
    end
end

%average multi images
I = uint32(MapImg_Ms);
for i = index_a:index_b
    I = imadd(I, uint32(warpedImg{i}));
end
total = index_b-index_a+1+1;  %total number of warped image + 1 map image
I = uint8(I/total);
figure
imshow(I);
title("overlaid images");





