from tensorflow import keras
import numpy as np

from tensorflow.keras.preprocessing.image import load_img

class MarineImages(keras.utils.Sequence):
    """ Helper to iterate over data (as Numpy array) """

    def __init__(self, batchSize, imageSize, input_img_paths, segmentation_img_paths):
        self.batchSize = batchSize
        self.imageSize = imageSize
        self.input_img_paths = input_img_paths
        self.segmentation_img_paths = segmentation_img_paths

    def __len__(self):
        return len(self.segmentation_img_paths) // self.batchSize

    def __getitem__(self, idx):
        """ Returns tuple (input, segmentation mask) corresponding to batch #idx """

        i = idx * self.batchSize
        batch_input_img_paths = self.input_img_paths[i : i + self.batchSize]
        batch_segmentation_img_paths = self.segmentation_img_paths[i : i + self.batchSize]

        x = np.zeros((self.batchSize,) + self.imageSize + (3,), dtype="uint8")
        print("x size = {0}".format(x.shape))

        for index, path in enumerate(batch_input_img_paths):
            img = load_img(path, target_size=self.imageSize)
            print("index = {0}, path = {1}".format(index, path))
            print("target size = {0}".format(self.imageSize))
            print("img shape = {0}".format(img.size))
            print("shape of x[{0}] = {1}".format(index, x[index].shape))
            x[index] = img

        y = np.zeros((self.batchSize,) + self.imageSize , dtype="uint8")
        for indexY, path in enumerate(batch_segmentation_img_paths):
            img = load_img(path, target_size=self.imageSize, color_mode="grayscale")
            y[indexY] = img # without expanding dimensions
            y[indexY] -= y[indexY]

        return x, y
    