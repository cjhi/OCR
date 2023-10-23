# OCR
Robot direction-taking with OCR for ROS2



## Installing and Running EasyOCR
pip install easyocr

In Python:

```python
import easyocr
import numpy as np

#resolves error that pops up without this:
import PIL.Image
if not hasattr(PIL.Image, 'Resampling'):  # Pillow<9.0
    PIL.Image.Resampling = PIL.Image

#run code
reader = easyocr.Reader(['ch_sim','en']) # this needs to run only once to load the model into memory
result = reader.readtext('chinese.jpg') # where 'chinese.jpg' can be replaced with any image file
print(result)
```


