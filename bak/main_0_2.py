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

physical_devices = tf.config.list_physical_devices('GPU')
tf.config.experimental.set_memory_growth(physical_devices[0], enable=True)
scaler = MinMaxScaler(feature_range=(0, 1))
train_index = 2000000
test_index = 2800000

def load_data(filename):
    values = np.load(filename)
    # ensure all data is float
    values = values.astype('float32')
    # normalize features
    scaled = scaler.fit_transform(values)
    scaled = scaled.reshape((scaled.shape[0], 1, scaled.shape[1]))

    assert len(scaled) >= ( test_index)
    print("unused ", len(scaled) - ( test_index))
    return (scaled[:train_index,:, :4], np.squeeze(scaled[:train_index,:, 4:]) ), (scaled[train_index:test_index,:,:4], np.squeeze(scaled[train_index:test_index,:,4:]))

def get_model():
    # design network
    model = Sequential()
    model.add(Bidirectional(LSTM(50, return_sequences=True), input_shape=(1, 4)))
    model.add(Bidirectional(LSTM(50)))
    model.add(Dense(2))
    model.add(Activation('softmax'))
    model.compile(loss='mae', optimizer='adam')
    return model

def train(dataset, name, version):
    if not os.path.isdir('output'):
        os.mkdir('output')
    weight_name = "output/weight_{}_{:0}.h5".format(name, version)
    loss_name   = "output/loss_{}_{:0}.csv".format(name, version)
    scaler_name = "output/scaler_{}_{:0}.pkl".format(name, version)
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
    history = model.fit(train_X, train_y, epochs=500, batch_size=172, validation_data=(test_X, test_y), verbose=1,
                        shuffle=False)
    model.save(weight_name)
    dtf = pd.DataFrame(history.history)
    dtf.to_csv(loss_name)
    dump(scaler, open(scaler_name, 'wb'))


if __name__ == '__main__':
    dataset = 'dataset/dataset.npy'
    train(dataset, name="roms_prediction", version=time())







