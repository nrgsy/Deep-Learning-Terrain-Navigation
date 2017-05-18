[training_images, training_labels, test_images,test_labels] =  get_data();
fname = './dataset.csv';

%first, scale 0-1 instead of 0-255
training_images = training_images/255;
test_images = test_images/255;

fid = fopen (fname, 'w');
display 'writing images to disk in csv format'
[y,x] = size(training_images);
for i = 1:y
    for j = 1:x
        fprintf(fid,'%6.4f, ', training_images(i,j));
    end
    fprintf(fid,'%6.4f\n', training_labels(i));
    
    if rem(i,1000) == 0
        disp([num2str(i) ' training images written'])
    end
end
[y,x] = size(test_images);
for i = 1:y
    for j = 1:x
        fprintf(fid,'%6.4f, ', test_images(i,j));
    end
    fprintf(fid,'%6.4f\n', test_labels(i));
    
    if rem(i,1000) == 0
        disp([num2str(i) ' test images written'])
    end
end
fclose('all');



