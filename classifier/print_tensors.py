# from tensorflow.python.tools.inspect_checkpoint import print_tensors_in_checkpoint_file
# checkpoint_path = os.path.join("/home/thomas/Desktop/thesis_partial/classifier/saves/", "FINAL-24221_185578_2285714_35_1e-06_False_2conv1024-0.5_8243-7946-1300-1068_0.872393166999-2285714")
#
# # List ALL tensors example output: v0/Adam (DT_FLOAT) [3,3,1,80]
# print_tensors_in_checkpoint_file(file_name=checkpoint_path, tensor_name='')
#
# # List contents of v0 tensor.
# # Example output: tensor_name:  v0 [[[[  9.27958265e-02   7.40226209e-02   4.52989563e-02   3.15700471e-02
# # print_tensors_in_checkpoint_file(file_name=checkpoint_path, tensor_name='v0')
#
# # List contents of v1 tensor.
# # print_tensors_in_checkpoint_file(file_name=checkpoint_path, tensor_name='v1')



import os
from tensorflow.python import pywrap_tensorflow

# checkpoint_path = os.path.join("/home/thomas/Desktop/thesis_partial/classifier/saves/", "FINAL-24221_185578_2285714_35_1e-06_False_2conv1024-0.5_8243-7946-1300-1068_0.872393166999-2285714")
# checkpoint_path = os.path.join("/home/thomas/Desktop/thesis_partial/classifier/saves/", "FINAL-751_185578_142857_35_1e-05_False_2conv1024-0.25_8070-7900-1346-1241_0.860591690467-142857")
checkpoint_path = os.path.join("/home/thomas/Desktop/thesis_partial/classifier/saves/", "FINAL-114458_185578_35_1e-06_False_2conv1024-0.5_8228-7925-1327-1077_0.870453198254-22857142")

reader = pywrap_tensorflow.NewCheckpointReader(checkpoint_path)
var_to_shape_map = reader.get_variable_to_shape_map()
for key in var_to_shape_map:
    print("tensor_name: ", key)
    # print(reader.get_tensor(key)) # Remove this is you want to print only variable names
