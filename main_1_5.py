#!/home/redwan/anaconda3/envs/OceanPreadiction/bin/python
import numpy as np
from sklearn.preprocessing import MinMaxScaler
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense, Activation
from tensorflow.keras.layers import LSTM, Bidirectional
import pandas as pd
import tensorflow as tf
from pickle import dump
from time import time
import os
from prettytable import PrettyTable

tf.get_logger().setLevel('ERROR')
tf.autograph.set_verbosity(1)
np.random.seed(8891)
scaler = MinMaxScaler(feature_range=(0, 1))

#physical_devices = tf.config.list_physical_devices('GPU')
#tf.config.experimental.set_memory_growth(physical_devices[0], enable=True)
def load_data(filename):
    values = np.load(filename)
    # ensure all data is float
    values = values.astype('float32')
    # normalize features
    scaled = scaler.fit_transform(values)
    scaled = scaled.reshape((scaled.shape[0], 1, scaled.shape[1]))
    
    train_num = int(0.7*len(values))
    indexes = np.arange(len(values))
    np.random.shuffle(indexes)
    train_indexes = indexes[:train_num]
    test_indexes = indexes[train_num:]
    return (scaled[train_indexes,:, :4], np.squeeze(scaled[train_indexes,:, 4:]) ), (scaled[test_indexes,:,:4], np.squeeze(scaled[test_indexes,:,4:]))

def get_model():
    # design network
    model = Sequential()
    model.add(LSTM(32, input_shape=(1, 4), return_sequences=True))
    model.add(LSTM(32))  # return a single vector of dimension 32
    model.add(Dense(2))
    model.compile(loss='mae')
    return model

def train(dataset, name, version, folder = '05'):
    if not os.path.isdir('output/%s'%folder):
        os.mkdir('output/%s'%folder)
    weight_name = "output/{}/{}_{:.0f}.h5".format(folder, name, version)
    loss_name   = "output/{}/{}_{:.0f}.csv".format(folder, name, version)
    scaler_name = "output/{}/{}_{:.0f}.pkl".format(folder, name, version)
    os.system('clear')

    (train_X, train_y ), (test_X, test_y) = load_data(dataset)
    # print(train_X.shape, train_y.shape, test_X.shape, test_y.shape)

    x = PrettyTable()
    x.field_names = ["parameter", "value"]

    x.add_rows(
        [
            ['train_X', train_X.shape],
            ['train_y', train_y.shape],
            ['test_X', test_X.shape],
            ['test_y', test_y.shape],
            ['weight_name', weight_name],
            ['loss_name', loss_name],
            ['scaler_name', scaler_name],
        ]
    )
    print(str(x))
    model = get_model()
    history = model.fit(train_X, train_y, epochs=50, batch_size=72, validation_data=(test_X, test_y), verbose=1,
                        shuffle=False)
    model.save(weight_name)
    dtf = pd.DataFrame(history.history)
    dtf.to_csv(loss_name)
    dump(scaler, open(scaler_name, 'wb'))


if __name__ == '__main__':
    dataset = 'dataset/dataset.npy'
    train(dataset, name="roms_prediction", version=time())







