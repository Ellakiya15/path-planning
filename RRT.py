from PIL import ImageOps,Image
import numpy as np
import matplotlib.pyplot as plt

img = Image.open('trial.png')
img = ImageOps.grayscale(img)

np_img = np.array(img)
np_img = ~np_img
np_img[np_img > 0] =1
plt.set_cmap('binary')
plt.imshow(np_img)
