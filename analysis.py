import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from sklearn.preprocessing import MinMaxScaler
import pandas as pd
import tensorflow as tf
from pickle import dump, load
from numpy import expand_dims
from pathlib import Path
# from bak.main_0_1 import get_model
from main_1_5 import get_model
from dataset import DataLoader
physical_devices = tf.config.list_physical_devices('GPU')
tf.config.experimental.set_memory_growth(physical_devices[0], enable=True)

def loss_plot(filename):
    dtf = pd.read_csv(filename, header=0, index_col=0)
    dtf.plot()

def prediction(time, depth):
    xx, yy = dataset.lonmesh, dataset.latmesh
    uu, vv = dataset.UU[time][depth], dataset.VV[time][depth]
    puu, pvv = np.zeros(uu.shape), np.zeros(vv.shape)
    fig, (ax1, ax2) = plt.subplots(1, 2, sharey=True)

    for i, lat in enumerate(np.squeeze(dataset.LAT)):
        X, orig = [], []
        for j, lon in enumerate(np.squeeze(dataset.LON)):
            u, v = dataset(i, j, depth, time)
            src = np.array((time, depth, lat, lon, u, v))
            data = scaler.transform(src.reshape(1, 6))
            x = data[:, :4]
            X.append(x)
            orig.append(data)
        X = np.array(X)
        # X = np.squeeze(X)
        y_pred = model.predict(X)
        scaler_input = np.hstack((np.squeeze(X), y_pred))
        y_scaled = scaler.inverse_transform(scaler_input)[:, 4:]
        puu[i][:] = y_scaled[:, 0]
        pvv[i][:] = y_scaled[:, 1]
    ax1.quiver(xx, yy, uu, vv, units='width')
    ax1.set_title('Original')
    ax2.quiver(xx, yy, puu, pvv, units='width')
    ax2.set_title('Predicted')

def search_path(dir):
    p = Path(dir)
    weight = list(p.glob('*.h5'))[0]
    scaler = list(p.glob('*.pkl'))[0]
    loss = list(p.glob('*.csv'))[0]
    return str(weight), str(scaler), str(loss)



if __name__ == '__main__':
    model = get_model()
    weight, scaler, loss = search_path('output/05')

    print(weight, scaler, loss)
    model.load_weights(weight)
    scaler = load(open(scaler, 'rb'))
    loss_plot(loss)


    dataset = DataLoader()
    depth, time = 1, 3

    prediction(time, depth)

    plt.show()
