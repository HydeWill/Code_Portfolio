function im = Robust_Recognition(I)
%figure, imshow(I)

% Step-2: Covert image to grayscale
I_gray = rgb2gray(I);
% figure, imshow(I_gray)

% Step-3: Rescale image
I_gray = imresize(I_gray, 0.5, 'bilinear');
% figure, imshow(I_gray)

% Step-4: Produce histogram before enhancing
% figure, imhist(I_gray)

% Step-5: Enhance image before binarisation
%I_en = histeq(I_gray,256);
%I_en = adapthisteq(I_gray);
%I_en = imadjust(I_gray);
I_en = imadjust(I_gray,[],[],2.5);
I_en = adapthisteq(I_en);
%figure, imshow(I_en)

% Step-6: Histogram after enhancement
%figure, imhist(I_en)

% Step-7: Image Binarisation
I_bin = imbinarize(I_en,'adaptive','foregroundPolarity','dark','Sensitivity',0.38);
%figure, imshow(I_bin)

%% Task 2: Edge detection ------------------------
%bound = edge(I_bin);
bound_C = edge(I_bin,'sobel');
% bound_P = edge(I_bin,'Prewitt');
%figure, imshow(bound_C)
%% Task 3: Simple segmentation --------------------
se = strel('disk',2);
clo = imclose(bound_C,se);
% figure, imshow(clo);
fill = imfill(clo,'holes');
% figure, imshow(fill);
se = strel('square',10);
fill = imopen(fill,se);
% figure, imshow(fill)

%% Task 4: Object Recognition --------------------
%Gets the circularity and perimeter data of each object
cir = regionprops('table',fill,'Circularity');
per = regionprops('table',fill,'Perimeter');
cc = bwconncomp(fill);

%Circles will have a circularity greater than one
%These are put in a label and convert to make their own image
i = find([cir.Circularity] >=1);
Wash = ismember(labelmatrix(cc),i);
Wash = label2rgb(Wash);
Wash = imcomplement(Wash);

%Small screws will have a smaller circularity then circles and smaller
%perimeter than longer screws
i = find([cir.Circularity] <1);
j = find([per.Perimeter] <= 250);
objs = i(ismember(i,j));
ss = ismember(labelmatrix(cc),objs);
ss = label2rgb(ss,'turbo');
ss = imcomplement(ss);

%Long screws will have a larger perimeter then any other object
j = find([per.Perimeter] > 250);
ls = ismember(labelmatrix(cc),j);
ls = label2rgb(ls,'winter');
ls = imcomplement(ls);

%The three images are put back together 
im = ls + ss + Wash;
figure, imshow(im);

