import numpy as np
import scipy.io
def write_data(X, Y):
    def to_string(U, name):
        data = "Start_" + name + '\n'
        m, n = U.shape
        for y in range(0, m):
            for x in range(0, n):
                data += str(U[y][x]) +'\t'
            data += '\n'
        data += "End_" + name + '\n'
        return data
    with open('UUVV01', 'w+') as file:
        file.write(to_string(U, 'UU'))
        file.write(to_string(V, 'VV'))

def load(k = 0, t = 1):
    ## index 1 represents the time, k represents the depth, i and j represent the longitude and latitude

    mat = scipy.io.loadmat('UUVV711b.mat')
    lonmesh = mat['lonmesh']
    latmesh = mat['latmesh']
    m, n = lonmesh.shape  ## m and n represents the size of lonmesh and latmesh matrices

    UU = mat['UU']
    VV = mat['VV']
    print(UU.shape, VV.shape)
    U = np.zeros((lonmesh.shape))
    V = np.zeros((latmesh.shape))
    for y in range(0, m):
        for x in range(0, n):
            U[y][x] = UU[t][k][y][x]
            V[y][x] = VV[t][k][y][x]
    return U, V
if __name__ == '__main__':
    filename = 'UUVV711b.mat'
    U, V  = load(k = 0, t = 1)
    write_data(U, V)