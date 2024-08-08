from PIL import ImageOps,Image
import numpy as np
import matplotlib.pyplot as plt
import random
from matplotlib.pyplot import rcParams

np.set_printoptions(precision=3,suppress=True)
rcParams['font.family'] = 'sans-serif'
rcParams['font.sans-serif'] = ['Tahoma']
plt.rcParams['font.size'] = 11

img = Image.open('trial.png')
img = ImageOps.grayscale(img)

np_img = np.array(img)
np_img = ~np_img
np_img[np_img > 0] =1
plt.set_cmap('binary')
plt.imshow(np_img)

np.save('trial.npy',np_img)
#Read Image
grid = np.load('trial.npy')
plt.imshow(grid)
plt.tight_layout()
plt.show()

