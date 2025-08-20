import matplotlib.pyplot as plt
import numpy as np

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
# show_output_raw('tests_out/outputLevel0_HVX.raw', 640, 400)
# plot_cloud_score_transposed("tests_out/points0_hnms_HVX.raw", "tests_out/scores0_hnms_HVX.raw", 640, 5)
# plt.subplot(222)
# show_output_raw('tests_out/outputLevel1_HVX.raw', 384, 200)
# plot_cloud_score_transposed("tests_out/points1_hnms_HVX.raw", "tests_out/scores1_hnms_HVX.raw", 384, 3)
# plt.subplot(223)
# show_output_raw('tests_out/outputLevel2_HVX.raw', 256, 100)
# plot_cloud_score_transposed("tests_out/points2_hnms_HVX.raw", "tests_out/scores2_hnms_HVX.raw", 256, 2)
# plt.subplot(224)
# show_output_raw('tests_out/outputLevel3_HVX.raw', 128, 50)
# plot_cloud_score_transposed("tests_out/points3_hnms_HVX.raw", "tests_out/scores3_hnms_HVX.raw", 128, 1)
# plt.show()

show_output_raw('test_out/image_10_640_400_0.raw', 640, 400)


plt.show()