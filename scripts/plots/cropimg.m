
folder = '~/';
maxnum = 9;
for k=1:maxnum
    I = imread([folder num2str(k) '.png']);
    %imshow(I)
    Io = I(140:600, 670:1120, :);
    outname = ['out_' num2str(k) '.png']
    imwrite(Io, [folder outname])
    imshow(Io)
end
