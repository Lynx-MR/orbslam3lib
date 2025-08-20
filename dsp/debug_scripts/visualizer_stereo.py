import matplotlib.pyplot as plt
import numpy as np
from PIL import Image  

# Read tuples from a text file and plot them
def plot_tuples(file_name,delta):
    with open(file_name, 'r') as f:
        lines = f.readlines()
        x = []
        y = []
        for line in lines:
            x.append(int(line.split(",")[0][1:]) + delta)
            y.append(int(line.split(",")[1][:-2]) + delta)
        plt.scatter(x, y)


# Read tuples from a text file and plot them
def plot_debug(file_name):
    with open(file_name, 'r') as f:
        lines = f.readlines()
        x = []
        y = []
        axis = 0
        for line in lines:
            if axis == 0:
                x.append(int(line))
            else:
                y.append(int(line))

            axis = (axis + 1) % 2
        plt.scatter(x, y)

        coef = np.polyfit(x,y,1)
        poly1d_fn = np.poly1d(coef) 
        plt.plot(x,y, 'yo', x, poly1d_fn(x), '--k')

def show_output_raw(imagePath, width, height):
    #Read the raw binary data and display it
    with open(imagePath, 'rb') as f:
        image = f.read()
    image = np.frombuffer(image, dtype=np.uint8).reshape((height, width))

    #Display the image in a subplot
    plt.imshow(image, cmap='gray', label=imagePath)

def show_output_raw_stereo(imagePath, imagePath2, width, height):
    #Read the raw binary data and display it
    with open(imagePath, 'rb') as f:
        image1 = f.read()
    image1 = Image.frombytes('L', (width, height), image1)
    with open(imagePath2, 'rb') as f:
        image2 = f.read()
    image2 = Image.frombytes('L', (width, height), image2)

    new_im = Image.new('RGB', (2*width, height))

    new_im.paste(image1, (0,0))
    new_im.paste(image2, (image1.size[0],0))


    #Display the image in a subplot
    plt.imshow(new_im, cmap='gray', label=imagePath)


def show_output_value(imagePath, width, height, value):
    #Read the raw binary data and display it
    with open(imagePath, 'rb') as f:
        image = f.read()
    image = np.frombuffer(image, dtype=np.uint8).reshape((height, width))

    image2 = [[0 for i in range(width)] for j in range(height)]

    for i in range(0, width, 1):
        for j in range(0, height, 1):
            if(image[j][i] == value):
                image2[j][i] = 255
            else:
                image2[j][i] = 0

    #Display the image in a subplot
    plt.imshow(image2, cmap='gray', label=imagePath)

# Read raw binary data and display it as point cloud (read 32 bits per 32 bits, then divide by 2^16 for y and %2^16 for x)
def plot_cloud_old(file_name):
    with open(file_name, 'rb') as f:
        image = f.read()
    image = np.frombuffer(image, dtype=np.uint32)
    x = []
    y = []
    for i in range(0, len(image), 1):
        if(image[i] // 2**16 > 1000):
            continue

        x.append(image[i] % 2**16)
        y.append(image[i] // 2**16)
    plt.scatter(x, y)

def plot_cloud_32(file_name, stride):
    with open(file_name, 'rb') as f:
        image = f.read()
    image = np.frombuffer(image, dtype=np.uint32)
    x = []
    y = []
    for i in range(0, len(image), 1):
        if(image[i] // stride > 1000):
            continue

        x.append(image[i] % stride)
        y.append(image[i] // stride)
    plt.scatter(x, y)

def plot_cloud(file_name, stride, count):
    with open(file_name, 'rb') as f:
        image = f.read()
    image = np.frombuffer(image, dtype=np.uint16)
    x = []
    y = []
    for k in range(0, count, 1):
        for i in range(k*(len(image)//count), (k+1)*len(image)//count, 1):
            if(image[i] == 0xFFFF):
                continue
            
            x.append(image[i] % 128 + k*128)
            y.append(image[i] // 128)
    plt.scatter(x, y)

def plot_cloud_stereo(file_name, offsetX, count):
    with open(file_name, 'rb') as f:
        image = f.read()
    image = np.frombuffer(image, dtype=np.uint16)
    x = []
    y = []
    for k in range(0, count, 1):
        for i in range(k*(len(image)//count), (k+1)*len(image)//count, 1):
            if(image[i] == 0xFFFF):
                continue
            
            x.append(image[i] % 128 + k*128 + offsetX)
            y.append(image[i] // 128)
    plt.scatter(x, y)


def plot_cloud_score(file_name, file_name_score, stride, count):
    with open(file_name, 'rb') as f:
        image = f.read()
    image = np.frombuffer(image, dtype=np.uint16)
    with open(file_name_score, 'rb') as f:
        score = f.read()
    score = np.frombuffer(score, dtype=np.uint16)
    x = []
    y = []
    c = []
    for k in range(0, count, 1):
        for i in range(k*(len(image)//count), (k+1)*len(image)//count, 1):
            if(image[i] == 0xFFFF):
                continue
            if(score[i] == 0):
                continue
            
            x.append(image[i] % 128 + k*128)
            y.append(image[i] // 128)
            c.append(score[i])
    plt.scatter(x, y, c=c)

def plot_cloud_score_transposed(file_name, file_name_score, stride, count):
    with open(file_name, 'rb') as f:
        image = f.read()
    image = np.frombuffer(image, dtype=np.uint16)
    with open(file_name_score, 'rb') as f:
        score = f.read()
    score = np.frombuffer(score, dtype=np.uint16)
    x = []
    y = []
    c = []
    for k in range(0, count, 1):
        for i in range(k*(len(image)//count), (k+1)*len(image)//count, 1):
            if(image[i] == 0xFFFF):
                continue
            if(score[i] == 0):
                continue
            
        #     x = i*128 + j (0 <= j < 128)
        # * We want y = j*512 + i (0 <= i < 512 (no image has more height than 512))

            j = image[i] // 512
            ii = image[i] % 512
            xx = ii*128 + j

            if(score[i] > 256) :
                continue


            x.append(xx % 128 + k*128)
            y.append(xx // 128)
            c.append(score[i])
    plt.scatter(x, y, c=c)

def plot_cloud_score_transposed_stereo(file_name, file_name_score, xoffset, count):
    with open(file_name, 'rb') as f:
        image = f.read()
    image = np.frombuffer(image, dtype=np.uint16)
    with open(file_name_score, 'rb') as f:
        score = f.read()
    score = np.frombuffer(score, dtype=np.uint16)
    x = []
    y = []
    c = []
    for k in range(0, count, 1):
        for i in range(k*(len(image)//count), (k+1)*len(image)//count, 1):
            if(image[i] == 0xFFFF):
                continue
            if(score[i] == 0):
                continue
            
        #     x = i*128 + j (0 <= j < 128)
        # * We want y = j*512 + i (0 <= i < 512 (no image has more height than 512))

            j = image[i] // 512
            ii = image[i] % 512
            xx = ii*128 + j

            if(score[i] > 256) :
                continue


            x.append(xx % 128 + k*128 + xoffset)
            y.append(xx // 128)
            c.append(score[i])
    plt.scatter(x, y, c=c)


def plot_cloud_score_stereo(file_name, file_name_score, xoffset, count):
    with open(file_name, 'rb') as f:
        image = f.read()
    image = np.frombuffer(image, dtype=np.uint16)
    with open(file_name_score, 'rb') as f:
        score = f.read()
    score = np.frombuffer(score, dtype=np.uint16)
    x = []
    y = []
    c = []
    for k in range(0, count, 1):
        for i in range(k*(len(image)//count), (k+1)*len(image)//count, 1):
            if(image[i] == 0xFFFF):
                continue
            if(score[i] == 0):
                continue
            if(score[i] > 256) :
                continue


            x.append(image[i] % 128 + k*128 + xoffset)
            y.append(image[i] // 128)
            c.append(score[i])
    plt.scatter(x, y, c=c)


def plot_cloud_score_stereo_angle_old(file_name, file_name_score, file_name_angles, delta_pix_angles, xoffset, count):
    with open(file_name, 'rb') as f:
        image = f.read()
    image = np.frombuffer(image, dtype=np.uint16)
    with open(file_name_score, 'rb') as f:
        score = f.read()
    score = np.frombuffer(score, dtype=np.uint16)
    with open(file_name_angles, 'rb') as f:
        angles = f.read()
    angles = np.frombuffer(angles, dtype=np.uint8)
    x = []
    lx = []
    y = []
    ly = []
    c = []
    for k in range(0, count, 1):
        for i in range(k*(len(image)//count), (k+1)*len(image)//count, 1):
            if(image[i] == 0xFFFF):
                continue
            if(score[i] == 0):
                continue
            if(score[i] > 256) :
                continue
            if(angles[i] == 0):
                continue
                
            final_x = image[i] % 128 + k*128 + xoffset
            final_y = image[i] // 128

            encodedAngle = angles[i]
            x_bits = (encodedAngle & 0b00111000) // 8
            y_bits = encodedAngle & 0b00000111


            tan_angle = 0.0
            angle = 0.0
            if(x_bits != 0):
                tan_angle = float(y_bits) / float(x_bits)
                angle = np.arctan(tan_angle)
            else:
                angle = np.pi/2
           
            x_sign = (encodedAngle & 0b10000000) >> 7
            y_sign = (encodedAngle & 0b01000000) >> 6

            if(x_sign == 1):
                angle = np.pi - angle
            
            if(y_sign == 1):
                angle = -angle

            print(hex(encodedAngle) + " : " + str(angle))
            

            final_delta_x = -delta_pix_angles * np.cos(angle) * 5
            final_delta_y = -delta_pix_angles * np.sin(angle) * 5

            plt.arrow(final_x, final_y, final_delta_x, final_delta_y, length_includes_head=True, head_width=0.5*delta_pix_angles, head_length=1.0*delta_pix_angles, color="red")

            x.append(final_x)
            y.append(final_y)
            c.append(score[i])
    plt.scatter(x, y, c=c)

def plot_cloud_score_stereo_angle(file_name, file_name_score, file_name_angles, delta_pix_angles, xoffset, count):
    with open(file_name, 'rb') as f:
        image = f.read()
    image = np.frombuffer(image, dtype=np.uint16)
    with open(file_name_score, 'rb') as f:
        score = f.read()
    score = np.frombuffer(score, dtype=np.uint16)
    with open(file_name_angles, 'rb') as f:
        angles = f.read()
    angles = np.frombuffer(angles, dtype=np.uint16)
    x = []
    lx = []
    y = []
    ly = []
    c = []
    for k in range(0, count, 1):
        for i in range(k*(len(image)//count), (k+1)*len(image)//count, 1):
            if(image[i] == 0xFFFF):
                continue
            if(score[i] == 0):
                continue
            if(score[i] > 256) :
                continue
            if(angles[i] == 0):
                continue
                
            final_x = image[i] % 128 + k*128 + xoffset
            final_y = image[i] // 128

            encodedAngle = angles[i]
            sin_raw = (encodedAngle & 0b1111111100000000) >> 8
            cos_raw = encodedAngle & 0b0000000011111111
            
            cos_val = np.int8(cos_raw) / 64.0
            sin_val = np.int8(sin_raw) / 64.0

            print("cos: " + str(cos_val) + " sin: " + str(sin_val))

            

            final_delta_x = -delta_pix_angles * cos_val * 5
            final_delta_y = -delta_pix_angles * sin_val * 5

            plt.arrow(final_x, final_y, final_delta_x, final_delta_y, length_includes_head=True, head_width=0.5*delta_pix_angles, head_length=1.0*delta_pix_angles, color="red")

            x.append(final_x)
            y.append(final_y)
            c.append(score[i])
    plt.scatter(x, y, c=c)


# plot_tuples("test_data_2d.raw",0.25)
# plot_tuples("or_data_2d.raw",0)

# plot_debug("test_data.raw")


# plt.figure(figsize=(10, 10))
# plt.subplot(221)
# show_output_raw('tests_out/outputLevel0_Dumb.raw', 640, 480)
# plot_cloud("tests_out/points0_Dumb.raw", 640, 5)
# plt.subplot(222)
# show_output_raw('tests_out/outputLevel1_Dumb.raw', 384, 240)
# plot_cloud("tests_out/points1_Dumb.raw", 384, 3)
# plt.subplot(223)
# show_output_raw('tests_out/outputLevel2_Dumb.raw', 256, 120)
# plot_cloud("tests_out/points2_Dumb.raw", 256, 2)
# plt.subplot(224)
# show_output_raw('tests_out/outputLevel3_Dumb.raw', 128, 60)
# plot_cloud("tests_out/points3_Dumb.raw", 128, 1)

# plt.figure(figsize=(10, 10))
# plt.subplot(221)
# show_output_raw_stereo('tests_out/outputLevel0_HVX_stereo_left.raw', 'tests_out/outputLevel0_HVX_stereo_right.raw', 640, 400)
# plot_cloud_score_transposed_stereo("tests_out/points0_HVX_hnms_stereo_left.raw", "tests_out/scores0_HVX_hnms_stereo_left.raw", 0, 5)
# plot_cloud_score_transposed_stereo("tests_out/points0_HVX_hnms_stereo_right.raw", "tests_out/scores0_HVX_hnms_stereo_right.raw", 640, 5)
# plt.subplot(222)
# show_output_raw_stereo('tests_out/outputLevel1_HVX_stereo_left.raw', 'tests_out/outputLevel1_HVX_stereo_right.raw', 384, 200)
# plot_cloud_score_transposed_stereo("tests_out/points1_HVX_hnms_stereo_left.raw", "tests_out/scores1_HVX_hnms_stereo_left.raw", 0, 3)
# plot_cloud_score_transposed_stereo("tests_out/points1_HVX_hnms_stereo_right.raw", "tests_out/scores1_HVX_hnms_stereo_right.raw", 384, 3)
# plt.subplot(223)
# show_output_raw_stereo('tests_out/outputLevel2_HVX_stereo_left.raw', 'tests_out/outputLevel2_HVX_stereo_right.raw', 256, 100)
# plot_cloud_score_transposed_stereo("tests_out/points2_HVX_hnms_stereo_left.raw", "tests_out/scores2_HVX_hnms_stereo_left.raw", 0, 2)
# plot_cloud_score_transposed_stereo("tests_out/points2_HVX_hnms_stereo_right.raw", "tests_out/scores2_HVX_hnms_stereo_right.raw", 256, 2)
# plt.subplot(224)
# show_output_raw_stereo('tests_out/outputLevel3_HVX_stereo_left.raw', 'tests_out/outputLevel3_HVX_stereo_right.raw', 128, 50)
# plot_cloud_score_transposed_stereo("tests_out/points3_HVX_hnms_stereo_left.raw", "tests_out/scores3_HVX_hnms_stereo_left.raw", 0, 1)
# plot_cloud_score_transposed_stereo("tests_out/points3_HVX_hnms_stereo_right.raw", "tests_out/scores3_HVX_hnms_stereo_right.raw", 128, 1)
# plt.show()


plt.figure(figsize=(10, 10))
plt.subplot(221)
show_output_raw_stereo('tests_out/outputLevel0_HVX_stereo_left.raw', 'tests_out/outputLevel0_HVX_stereo_right.raw', 640, 400)
plot_cloud_score_stereo_angle("tests_out/points0_HVX_hnms_stereo_left.raw", "tests_out/scores0_HVX_hnms_stereo_left.raw", "tests_out/angles0_HVX_hnms_stereo_left.raw", 8, 0, 5)
plot_cloud_score_stereo_angle("tests_out/points0_HVX_hnms_stereo_right.raw", "tests_out/scores0_HVX_hnms_stereo_right.raw", "tests_out/angles0_HVX_hnms_stereo_right.raw", 8, 640, 5)
plt.subplot(222)
show_output_raw_stereo('tests_out/outputLevel1_HVX_stereo_left.raw', 'tests_out/outputLevel1_HVX_stereo_right.raw', 384, 200)
plot_cloud_score_stereo_angle("tests_out/points1_HVX_hnms_stereo_left.raw", "tests_out/scores1_HVX_hnms_stereo_left.raw", "tests_out/angles1_HVX_hnms_stereo_left.raw", 4, 0, 3)
plot_cloud_score_stereo_angle("tests_out/points1_HVX_hnms_stereo_right.raw", "tests_out/scores1_HVX_hnms_stereo_right.raw", "tests_out/angles1_HVX_hnms_stereo_right.raw", 4, 384, 3)
plt.subplot(223)
show_output_raw_stereo('tests_out/outputLevel2_HVX_stereo_left.raw', 'tests_out/outputLevel2_HVX_stereo_right.raw', 256, 100)
plot_cloud_score_stereo_angle("tests_out/points2_HVX_hnms_stereo_left.raw", "tests_out/scores2_HVX_hnms_stereo_left.raw", "tests_out/angles2_HVX_hnms_stereo_left.raw", 2, 0, 2)
plot_cloud_score_stereo_angle("tests_out/points2_HVX_hnms_stereo_right.raw", "tests_out/scores2_HVX_hnms_stereo_right.raw", "tests_out/angles2_HVX_hnms_stereo_right.raw", 2, 256, 2)
plt.subplot(224)
show_output_raw_stereo('tests_out/outputLevel3_HVX_stereo_left.raw', 'tests_out/outputLevel3_HVX_stereo_right.raw', 128, 50)
plot_cloud_score_stereo_angle("tests_out/points3_HVX_hnms_stereo_left.raw", "tests_out/scores3_HVX_hnms_stereo_left.raw", "tests_out/angles3_HVX_hnms_stereo_left.raw", 1, 0, 1)
plot_cloud_score_stereo_angle("tests_out/points3_HVX_hnms_stereo_right.raw", "tests_out/scores3_HVX_hnms_stereo_right.raw", "tests_out/angles3_HVX_hnms_stereo_right.raw", 1, 128, 1)
plt.show()

plt.show()