import easyocr
import numpy as np

import PIL.Image
if not hasattr(PIL.Image, 'Resampling'):  # Pillow<9.0
    PIL.Image.Resampling = PIL.Image

reader = easyocr.Reader(['ch_sim','en']) # this needs to run only once to load the model into memory
#result = reader.readtext('chinese.jpg')

#input = np.
result = reader.readtext('turnRight.png')
print(result)