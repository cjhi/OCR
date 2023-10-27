import easyocr
import numpy as np

#resolves error that pops up without this:
import PIL.Image
if not hasattr(PIL.Image, 'Resampling'):  # Pillow<9.0
    PIL.Image.Resampling = PIL.Image


#run code
reader = easyocr.Reader(['ch_sim','en']) # this needs to run only once to load the model into memory
#result = reader.readtext('chinese.jpg')

#input = np.
result = reader.readtext('turnRight.png')
print(result)

#consider putting signs in one color, pre-processing images to find text of interest before running OCR
#frequency of checking for text will depend on latency of running OCR function - if 5 seconds, 1 image/five seconds, if 2s, 1 image/2 seconds

#look at particle filter to see how you to move forward as soon as image updates7