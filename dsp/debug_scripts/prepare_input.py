import cv2

def prepare_input(image):
    #Just load the image and write the array raw binary data to a file
    image = cv2.imread(image, cv2.IMREAD_GRAYSCALE)
    with open('input.raw', 'wb') as f:
        f.write(image.tobytes())

def prepare_input_2(image):
    #Just load the image and write the array raw binary data to a file
    image = cv2.imread(image, cv2.IMREAD_GRAYSCALE)
    with open('input_stereo.raw', 'wb') as f:
        f.write(image.tobytes())


prepare_input('test_img.png')
prepare_input_2('test_img_stereo.png')