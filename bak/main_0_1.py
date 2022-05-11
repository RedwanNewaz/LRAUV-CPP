import numpy as np
from sklearn.preprocessing import MinMaxScaler
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense
from tensorflow.keras.layers import LSTM
import pandas as pd
import tensorflow as tf
from pickle import dump
physical_devices = tf.config.list_physical_devices('GPU')
tf.config.experimental.set_memory_growth(physical_devices[0], enable=True)
scaler = MinMaxScaler(feature_range=(0, 1))

def load_data(filename):
    values = np.load(filename)
    # ensure all data is float
    values = values.astype('float32')
    # normalize features
    scaled = scaler.fit_transform(values)
    scaled = scaled.reshape((scaled.shape[0], 1, scaled.shape[1]))
    N = len(scaled)
    train_index = 1300000
    test_index = 1500000
    assert N >= ( test_index)
    print("unused ", N - ( test_index))
    return (scaled[:train_index,:, :4], np.squeeze(scaled[:train_index,:, 4:]) ), (scaled[train_index:test_index,:,:4], np.squeeze(scaled[train_index:test_index,:,4:]))

def get_model():
    # design network
    model = Sequential()
    model.add(LSTM(50, input_shape=(1, 4)))
    model.add(Dense(2))
    model.compile(loss='mae', optimizer='adam')
    return model

if __name__ == '__main__':

    (train_X, train_y ), (test_X, test_y) = load_data('dataset/dataset.npy')
    print(train_X.shape, train_y.shape, test_X.shape, test_y.shape)
    model = get_model()
    history = model.fit(train_X, train_y, epochs=50, batch_size=72, validation_data=(test_X, test_y), verbose=1,
                        shuffle=False)

    model.save("roms_prediction_0.h5")
    dtf = pd.DataFrame(history.history)
    dtf.to_csv('training_loss_0.csv')
    dump(scaler, open('scaler.pkl', 'wb'))



