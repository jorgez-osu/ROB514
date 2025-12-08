#!/usr/bin/env python3

# This assignment introduces you to a common task (creating a segmentation mask) and a common tool (kmeans) for
#  doing clustering of data and the difference in color spaces.

# There are no shortage of kmeans implementations out there - using scipy's
import numpy as np
from scipy.cluster.vq import kmeans, vq, whiten

# Using imageio to read in the images and skimage to do the color conversion
import imageio
from skimage.color import rgb2hsv
import matplotlib.pyplot as plt


def read_and_cluster_image(image_name, use_hsv, n_clusters):
    """ Read in the image, cluster the pixels by color (either rgb or hsv), then
    draw the clusters as an image mask, colored by both a random color and the center
    color of the cluster
    @image_name - name of image in Data
    @use_hsv - use hsv, y/n
    @n_clusters - number of clusters (up to 6)"""

    try:
        im_orig = imageio.imread("Data/" + image_name)
    except FileNotFoundError:
        im_orig = imageio.imread(image_name)

    im_orig = im_orig[:, :, 0:3]
    height, width, channels = im_orig.shape

    fig, axs = plt.subplots(1, 3, figsize=(12, 4))

    str_im_name = image_name.split('.')[0] + " "
    if use_hsv:
        str_im_name += "HSV"
    else:
        str_im_name += "RGB"
    str_im_name += f", k={n_clusters}"

    axs[0].imshow(im_orig)
    axs[0].set_title(str_im_name)
    axs[0].axis("off")

    if use_hsv:
        im_for_kmeans = rgb2hsv(im_orig.astype(np.float32) / 255.0)
    else:
        im_for_kmeans = im_orig.astype(np.float32)

    data = im_for_kmeans.reshape(-1, 3)

    whitened = whiten(data)
    centers, _ = kmeans(whitened, n_clusters)
    ids, _ = vq(whitened, centers)  

    rgb_color = np.array(
        [[255,   0,   0],   # red
         [  0, 255,   0],   # green
         [  0,   0, 255],   # blue
         [255, 255,   0],   # yellow
         [  0, 255, 255],   # cyan
         [255,   0, 255]],  # magenta
        dtype=np.float32
    )

    if n_clusters > rgb_color.shape[0]:
        raise ValueError("n_clusters must be <= 6 for the predefined colors")

    N = height * width
    mask_by_id_flat = np.zeros((N, 3), dtype=np.float32)
    mask_by_mean_flat = np.zeros((N, 3), dtype=np.float32)

    rgb_data = im_orig.reshape(-1, 3).astype(np.float32)

    for k in range(n_clusters):
        in_cluster = (ids == k)
        if not np.any(in_cluster):
            continue

        mask_by_id_flat[in_cluster] = rgb_color[k]

        mean_color = np.mean(rgb_data[in_cluster], axis=0)
        mask_by_mean_flat[in_cluster] = mean_color

    mask_by_id = mask_by_id_flat.reshape(height, width, 3).astype(im_orig.dtype)
    mask_by_mean = mask_by_mean_flat.reshape(height, width, 3).astype(im_orig.dtype)


    axs[1].imshow(mask_by_id)
    axs[1].set_title("ID colored by rgb")
    axs[1].axis("off")

    axs[2].imshow(mask_by_mean)
    axs[2].set_title("ID colored by cluster average")
    axs[2].axis("off")

    fig.tight_layout()

    return ids.reshape(height, width)



if __name__ == '__main__':
    read_and_cluster_image("real_apple.jpg", True, 4)
    read_and_cluster_image("trees.png", True, 2)
    read_and_cluster_image("trees_depth.png", False, 3)
    read_and_cluster_image("staged_apple.png", True, 3)
    read_and_cluster_image("staged_apple.png", False, 3)
    # Depending on if your mac, windows, linux, and if interactive is true, you may need to call this to get the plt
    # windows to show
    plt.show()
    print("done")
