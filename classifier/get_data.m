function [training_images, training_labels, test_images, test_labels]= get_data()

%check if .mat's exist, and if so just load em return, otherwise load
%from heightmap files (slower), save returns as .mat for next time
f1 = './data/training_images.mat';
f2 = './data/training_labels.mat';
f3 = './data/test_images.mat';
f4 = './data/test_labels.mat';
if (exist(f1, 'file') == 2 && ...
        exist(f2, 'file') == 2 && ...
        exist(f3, 'file') == 2 && ...
        exist(f4, 'file') == 2)
    display 'loading from .mat files'
    load(f1);
    load(f2);
    load(f3);
    load(f4);
    return;
end
display 'the proper .mat files were not found, loading from heightmaps'

image_x = 26;
image_y = 34;
test_data_fraction = 0.1; %used 10% as test data
training_data_fraction = 1 - test_data_fraction;

%negative_image_folder = '../data/negative_heightmap_subset/';
%positive_image_folder = '../data/positive_heightmap_subset/';
% negative_image_folder = '../data/NEGATIVE_OUTPUT/';
% positive_image_folder = '../data/POSITIVE_OUTPUT/';
negative_image_folder = './data/NEGATIVE_CROPPED_OUTPUT/';
positive_image_folder = './data/POSITIVE_CROPPED_OUTPUT/';

folder = dir([negative_image_folder, '*.data']);
num_negative_images = length(folder(not([folder.isdir])))
folder = dir([positive_image_folder, '*.data']);
num_positive_images = length(folder(not([folder.isdir])))

%round up
num_neg_training = ceil(training_data_fraction * num_negative_images);
num_pos_training = ceil(training_data_fraction * num_positive_images);
num_training_images = num_neg_training + num_pos_training;

training_images = zeros(num_training_images,image_x*image_y);
training_labels = zeros(num_training_images,1);
%to remember the indicies we've placed
training_used_indicies = zeros(num_training_images,1);

%round down
num_neg_test = floor(test_data_fraction * num_negative_images);
num_pos_test = floor(test_data_fraction * num_positive_images);
num_test_images = num_neg_test + num_pos_test;

if (num_positive_images + num_negative_images ~= ...
    num_training_images + num_test_images)
    display 'ERROR image number mismatch'
end

test_images = zeros(num_test_images,image_x*image_y);
test_labels = zeros(num_test_images,1);
test_used_indicies = zeros(num_test_images,1);

display('reading in negatively labeled heightmaps: ')
i = 1;
image_index = 0;
while (i <= num_negative_images)

    filename = [negative_image_folder 'heightmap' num2str(image_index) '.data'];
    if exist(filename, 'file') == 2
        fid = fopen (filename, 'r');
        img = fread(fid,'uint8');
        fclose(fid);

        %first images are test images
        if (i <= num_neg_test)
            %place randomly within test_images
            index = randi([1,num_test_images]);
            image_placed = false;
            while (~image_placed)
                %check if this index was already used, else try next index
                if test_used_indicies(index) == 0
                    test_images(index,:) = img';
                    test_labels(index) = 0;
                    test_used_indicies(index) = 1;
                    image_placed = true;
                else
                    index = index + 1;
                    if (index > num_test_images)
                        index = 1;
                    end
                end
            end
        else
            %the rest are training images, place randomly within training_images
            index = randi([1,num_training_images]);
            image_placed = false;
            while (~image_placed)
                %check if this index was already used, else try next index
                if training_used_indicies(index) == 0
                    training_images(index,:) = img';
                    training_labels(index) = 0;
                    training_used_indicies(index) = 1;
                    image_placed = true;
                else
                    index = index + 1;
                    if (index > num_training_images)
                        index = 1;
                    end
                end
            end
        end

        if rem(i,500) == 0 || i > num_negative_images-10
            disp([num2str(i) ' of ' num2str(num_negative_images) ' read in']);
        end
        i = i+1;
    end
    image_index = image_index + 1;
end

%now repeat for the positive images, filling in the rest of the gaps in
%the image/label matricies
display('reading in positively labeled heightmaps: ')
i = 1;
image_index = 0;
while (i <= num_positive_images)

    filename = [positive_image_folder 'heightmap' num2str(image_index) '.data'];
    if exist(filename, 'file') == 2
        fid = fopen (filename, 'r');
        img = fread(fid,'uint8');
        fclose(fid);

        %first images are test images
        if (i <= num_pos_test)
            %place randomly within test_images
            index = randi([1,num_test_images]);
            image_placed = false;
            while (~image_placed)
                %check if this index was already used, else try next index
                if test_used_indicies(index) == 0
                    test_images(index,:) = img';
                    test_labels(index) = 1;
                    test_used_indicies(index) = 1;
                    image_placed = true;
                else
                    index = index + 1;
                    if (index > num_test_images)
                        index = 1;
                    end
                end
            end
        else
            %the rest are training images, place randomly within training_images
            index = randi([1,num_training_images]);
            image_placed = false;
            while (~image_placed)
                %check if this index was already used, else try next index
                if training_used_indicies(index) == 0
                    training_images(index,:) = img';
                    training_labels(index) = 1;
                    training_used_indicies(index) = 1;
                    image_placed = true;
                else
                    index = index + 1;
                    if (index > num_training_images)
                        index = 1;
                    end
                end
            end
        end

        if rem(i,500) == 0 || i > num_positive_images-10
            disp([num2str(i) ' of ' num2str(num_positive_images) ' read in']);
        end
        i = i+1;
    end
    image_index = image_index + 1;
end

%error checking for unfilled rows of all zeros
if(any(~any(training_images, 2)) || any(~any(test_images, 2)))
    display 'ERROR: data improperly loaded'
else
    display 'saving results to .mat files'
    save(f1, 'training_images');
    save(f2, 'training_labels');
    save(f3, 'test_images');
    save(f4, 'test_labels');

    display 'data loading complete'
end
