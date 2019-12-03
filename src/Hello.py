# coding=gbk
from PIL import Image
import numpy as np
# import scipy
import matplotlib.pyplot as plt
import csv

def ImageToMatrix(filename):
    # 读取图片
    im = Image.open(filename)
    # 显示图片
#     im.show()  
    width,height = im.size
	
    im = im.convert("L") 
    data = im.getdata()
    data = np.matrix(data,dtype='float')/255.0
    #new_data = np.reshape(data,(width,height))
    new_data = np.reshape(data,(height,width))
    print("width: " , width ," height: " , height)
    return new_data
#     new_im = Image.fromarray(new_data)
#     # 显示图片
#     new_im.show()

#filename = 'levine.pgm'
filename = 'map_1.pgm'
data = ImageToMatrix(filename)
print("the matrix is:")

print(data) 
'''
with open("map.csv", "w") as csvfile:
	writer = csv.writer(csvfile)

	for line in data:
		writer.writerows(line)
	'''

np.savetxt('mymap_1.csv', data, delimiter = ',')
print("finish write csv")



'''
new_im = MatrixToImage(data)
plt.imshow(data, cmap=plt.cm.gray, interpolation=‘nearest‘)
new_im.show()
new_im.save(‘lena_1.bmp‘)
'''
