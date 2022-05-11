import scipy.io
import  os
from prettytable import PrettyTable
import numpy as np
import matplotlib.pyplot as plt

dirname = os.path.dirname(__file__)
print(dirname)
#TODO learn multivariate data https://machinelearningmastery.com/multivariate-time-series-forecasting-lstms-keras/

np.random.seed(8891)

class DataLoader(object):
    def __init__(self):
        mat = scipy.io.loadmat(dirname + '/UUVV711b.mat')
        self.lonmesh = mat['lonmesh']
        self.latmesh = mat['latmesh']
        self.depth = mat['Depth']
        self.UU = mat['UU']
        self.VV = mat['VV']
        self.time = mat['T']
        self.LAT = mat['LAT']
        self.LON = mat['LON']
        print(mat.keys())
    def __call__(self, lat, lon, depth, time):

        u = self.UU[time][depth][lat][lon]
        v = self.VV[time][depth][lat][lon]
        return (u, v)
    def __repr__(self):
        x = PrettyTable()

        x.field_names = ["Variable", "shape", "minValue", "maxValue"]
        x.add_rows(
            [
                ["lonmesh", self.lonmesh.shape, min(np.min(self.lonmesh, axis=1)), max(np.max(self.lonmesh, axis=1))],
                ["latmesh", self.latmesh.shape, min(np.min(self.latmesh, axis=1)), max(np.max(self.latmesh, axis=1))],
                ["depth", self.depth.shape, np.min(self.depth), np.max(self.depth)],
                ["UU", self.UU.shape, np.shape(self.UU[:][:][0][0]), np.shape(self.UU[0][0][:][:])],
                ["VV", self.VV.shape, np.shape(self.VV[:][:][0][0]), np.shape(self.VV[0][0][:][:])],
                ["time", self.time.shape, np.min(self.time), np.max(self.time)],
            ]
        )
        return str(x)

    def get_data_indexes(self):
        N = len(self.time)
        num_train = int(0.7*N)

        indexes = np.arange(N)
        np.random.shuffle(indexes)
        train_index = indexes[:num_train]
        test_index = indexes[num_train:]
        return train_index, test_index



    def __is_valid(self, lat, lon, depth, time):
        pass

def get_data(data):
    for t in np.squeeze(data.time):
        for d, depth in enumerate(np.squeeze(data.depth)):
            for i, lat in enumerate(np.squeeze(data.LAT)):
                for j, lon in enumerate(np.squeeze(data.LON)):
                    u, v = data(i, j, d, t)
                    yield (t, d, lat, lon, u, v)

if __name__ == '__main__':
    data = DataLoader()
    print(data)
    # fig, ax = plt.subplots()
    # dataset = get_data(data)
    # dataset = np.array(list(dataset))
    # np.save('dataset.npy', dataset)
    # dataset = np.load('dataset.npy')
    # dataset = dataset.reshape((dataset.shape[0], 1, dataset.shape[1]))
    # print(dataset.shape)




    # for time, t in enumerate(data.time):
    #     ax.cla()
    #     ax.quiver(data.lonmesh, data.latmesh, data.UU[time][depth], data.VV[time][depth], units='width')
    #     plt.pause(1)
    # plt.show()



    # fig, ax = plt.subplots()
    # time = 1
    # depth = 1
    # ax.quiver(data.lonmesh, data.latmesh, data.UU[time][depth], data.VV[time][depth], units='width')
    # plt.show()
