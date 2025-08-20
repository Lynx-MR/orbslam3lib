import numpy as np
import matplotlib.pyplot as plt

def show_output_raw(imagePath, width, height):
    #Read the raw binary data and display it
    with open(imagePath, 'rb') as f:
        image = f.read()
    image = np.frombuffer(image, dtype=np.uint8).reshape((height, width))

    #Display the image in a subplot
    plt.imshow(image, cmap='gray', label=imagePath)


# Show 4 images in subplots
# plt.figure(figsize=(10, 10))
# plt.subplot(221)
# show_output_raw('tests_out/outputLevel0_Dumb.raw', 640, 480)
# plt.subplot(222)
# show_output_raw('tests_out/outputLevel1_Dumb.raw', 384, 240)
# plt.subplot(223)
# show_output_raw('tests_out/outputLevel2_Dumb.raw', 256, 120)
# plt.subplot(224)
# show_output_raw('tests_out/outputLevel3_Dumb.raw', 128, 60)
# plt.show()

# # Show 4 images in subplots
# plt.figure(figsize=(10, 10))
# plt.subplot(221)
# show_output_raw('tests_out/outputLevel0_HVX.raw', 640, 480)
# plt.subplot(222)
# show_output_raw('tests_out/outputLevel1_HVX.raw', 384, 240)
# plt.subplot(223)
# show_output_raw('tests_out/outputLevel2_HVX.raw', 256, 120)
# plt.subplot(224)
# show_output_raw('tests_out/outputLevel3_HVX.raw', 128, 60)
# plt.show()

show_output_raw('input_stereo.raw', 1280, 512000//1280)
plt.show()