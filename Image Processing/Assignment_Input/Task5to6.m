clear; close all;
% Task 5: Robust method --------------------------
I = imread('IMG_01.jpg');
I_2 = imread('IMG_02.jpg');
I_3 = imread('IMG_03.jpg');
I_4 = imread('IMG_04.jpg');
I_5 = imread('IMG_05.jpg');
I_6 = imread('IMG_06.jpg');
I_7 = imread('IMG_07.jpg');
I_8 = imread('IMG_08.jpg');
I_9 = imread('IMG_09.jpg');
I_10 = imread('IMG_10.jpg');

Robust_Recognition(I);
Robust_Recognition(I_2);
Robust_Recognition(I_3);
Robust_Recognition(I_4);
Robust_Recognition(I_5);
Robust_Recognition(I_6);
Robust_Recognition(I_7);
Robust_Recognition(I_8);
Robust_Recognition(I_9);
Robust_Recognition(I_10);
% Task 6: Performance evaluation -----------------
% Step 1: Load ground truth data
%GT = imread("IMG_01_GT.png");

% To visualise the ground truth image, you can
% use the following code.
% L_GT = label2rgb(GT, 'prism','k','shuffle');
% figure, imshow(L_GT)
