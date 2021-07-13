clear; close all;

%% Task 1: Pre-processing -----------------------
% Step-1: Load input image
I = imread('IMG_01.jpg');
figure, imshow(I)

% Step-2: Covert image to grayscale
I_gray = rgb2gray(I);
figure, imshow(I_gray)

% Step-3: Rescale image
I_gray = imresize(I_gray, 0.5, 'bilinear');
figure, imshow(I_gray)

% Step-4: Produce histogram before enhancing
figure, imhist(I_gray)

% Step-5: Enhance image before binarisation
%I_en = histeq(I_gray,256);
%I_en = adapthisteq(I_gray);
%I_en = imadjust(I_gray);
I_en = imadjust(I_gray,[],[],2);
I_en = adapthisteq(I_en);
figure, imshow(I_en)

% Step-6: Histogram after enhancement
figure, imhist(I_en)

% Step-7: Image Binarisation
I_bin = imbinarize(I_en,'adaptive','foregroundPolarity','dark','Sensitivity',0.38);
figure, imshow(I_bin)

%% Task 2: Edge detection ------------------------
%Applies canny edge dectection 
%bound = edge(I_bin);
bound_C = edge(I_bin,'Canny');
% bound_P = edge(I_bin,'Prewitt');
figure, imshow(bound_C)
%% Task 3: Simple segmentation --------------------
%Applies closing to the edges 
se = strel('disk',2);
clo = imclose(bound_C,se);
figure, imshow(clo);

%Fills holes of closed edges
fill = imfill(clo,'holes');
figure, imshow(fill);

%Opens filled image to remove noise and bridges
se = strel('square',10);
fill = imopen(fill,se);
figure, imshow(fill)

%% Task 4: Object Recognition --------------------
%Finds circularity of the objects
cir = regionprops('table',fill,'Circularity');
cc = bwconncomp(fill);

%Seperates the objects on circularity and applies them to a label matrix
i = find([cir.Circularity] >=1);
Wash = ismember(labelmatrix(cc),i);
Wash = label2rgb(Wash);
Wash = imcomplement(Wash);

i = find([cir.Circularity] <1);
Screws = ismember(labelmatrix(cc),i);
Screws = label2rgb(Screws,'turbo'); %Colour map is applied to segment 
Screws = imcomplement(Screws);

%Combines the two images with black backgrounds to produce final image
Combined = Screws + Wash;
figure, imshow(Combined);

