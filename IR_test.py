import numpy as np
import cv2
from IR_sensor import IR_sensor_Reg, IR_sensor

print(f" Numpy Version: {np.__version__}")
print(f"OpenCV Version: {cv2.__version__}")

def bilinear_interpolation(img, scale=2.0):
    '''
    scale the image by the factor
    img: np.array that stores pixel-wise data
    scale: float, factor used to scale the origin image, greater than 0
    '''

    # get origin image size
    h, w = img.shape
    
    # calculate new size based on scaling factor
    new_h = int(h * scale)
    new_w = int(w * scale)

    # create empty array
    new_img = np.empty([new_h, new_w])

    # calculate scaling factor from scaled image to origin image
    x_scale = w / new_w
    y_scale = h / new_h

    # iterate through the scaled image pixels
    for r in range(new_h):
        for c in range(new_w):
            x = x_scale * c
            y = y_scale * r
            # get coordinate for each corner
            x1, x2 = int(np.floor(x)), min(int(np.ceil(x)), w-1)
            y1, y2 = int(np.floor(y)), min(int(np.ceil(y)), h-1)
            
            if x1 != x2 and y1 != y2:
                # get intensity va;ues of corners
                q11, q12 = img[y1, x1], img[y2, x1]
                q21, q22 = img[y1, x2], img[y2, x2]
                # interpolate
                new_img[r, c] = (1 / ((x2-x1) * (y2-y1))) * (
                                q11 * (x2-x) * (y2-y) +
                                q12 * (x2-x) * (y-y1) +
                                q21 * (x-x1) * (y2-y) + 
                                q22 * (x-x1) * (y-y1)) + 0.5
            else:
                if x1 == x2 and y1 == y2:
                    x, y = int(x), int(y)
                    new_img[r, c] = img[y, x]
                elif x1 == x2:
                    x = int(x)
                    q1, q2 = img[y1, x], img[y2, x]
                    new_img[r, c] = 1/(y2-y1) * (q1*(y2-y) + q2*(y-y1)) + 0.5
                elif y1 == y2:
                    y = int(y)
                    q1, q2 = img[y, x1], img[y, x2]
                    new_img[r, c] = 1/(x2-x1) * (q1*(x2-x) + q2*(x-x1)) + 0.5
                else:
                    print(f"That's weird: x = {x}, y = {y}")
    new_img = np.asarray(new_img, dtype=np.uint8)
    return new_img




sensor = IR_sensor(dev=r"/dev/ttyTHS0")
sensor.open()

name = "IR Camera 8x8"
cv2.namedWindow(name, cv2.WINDOW_AUTOSIZE)

temp_map = None
while(True):
    if sensor.isOpen():
        temp_map = sensor.get_temp_map()
    else:
        print("Sensor is closed!")
        break
    if temp_map is not None:
        #img_gray = np.frombuffer(temp_map, dtype=np.uint8).reshape(8,8)
        img_gray = np.asarray(temp_map, dtype=np.uint8).reshape(8,8)
        img_gray = bilinear_interpolation(img_gray, scale=4.0)
        #print(img_gray)
        img_color = cv2.applyColorMap(img_gray, cv2.COLORMAP_JET)
        cv2.imshow(name, img_color)

    k = cv2.waitKey(100)
    if k == 27: # ESC
        print("Keyboard Exit: ESC")
        break

cv2.destroyAllWindows()
'''
import serial

ser = serial.Serial(r"/dev/serial0", 9600, timeout=0.5)

if not ser.isOpen():
    ser.open()

for i in range(5):
    frame = ser.read(64)
    print(frame)
    print()

ser.close()
'''
