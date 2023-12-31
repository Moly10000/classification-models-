# -*- coding: utf-8 -*-
"""BIEN KNN y PER

YlMol

#Code without libraries
"""

import pandas as pd
import numpy as np

# Load your data
path = '/content/Indicadores_municipales_sabana_DA (3).csv'
data = pd.read_csv(path, sep=",", encoding='ISO-8859-1')
data = data.fillna(data.mean(numeric_only=True))
data['Drenaje_Categoria'] = pd.cut(data['porc_vivsndren10'], bins=[0, 20, 40, 60, 80, 100], labels=[1, 2, 3, 4, 5])
data.dropna(subset=['Drenaje_Categoria'], inplace=True)

#---------
# convertir de categoricas a numericas a las columnas 'gdo_rezsoc00', 'gdo_rezsoc05', y 'gdo_rezsoc10'
# diccionario de mapeo para cada columna
#mapeo_columna_00 = {'Alto': 0, 'Muy alto': 1}
#mapeo_columna_05 = {'Alto': 0, 'Medio': 1, 'Muy alto': 2, 'Bajo': 3}
#mapeo_columna_10 = {'Alto': 0, 'Muy alto': 1, 'Medio': 2}

# Aplica el mapeo a cada columna respectiva
#data['gdo_rezsoc00'] = data['gdo_rezsoc00'].map(mapeo_columna_00)
#data['gdo_rezsoc05'] = data['gdo_rezsoc05'].map(mapeo_columna_05)
#data['gdo_rezsoc10'] = data['gdo_rezsoc10'].map(mapeo_columna_10)

#-----------------
# Extract features and target
#categorical columns 'gdo_rezsoc00', 'gdo_rezsoc05', 'gdo_rezsoc10',
X = data[['pobtot_ajustada', 'pobreza', 'pobreza_e', 'pobreza_m', 'vul_car', 'vul_ing', 'npnv', 'ic_rezedu', 'ic_asalud', 'ic_segsoc', 'ic_cv', 'ic_sbv', 'ic_ali', 'carencias', 'carencias3', 'plb', 'plb_m', 'rankin_p', 'rankin_pe', 'N_pobreza', 'N_pobreza_e', 'N_pobreza_m', 'N_vul_car', 'N_vul_ing', 'N_npnv', 'N_ic_rezedu', 'N_ic_asalud', 'N_ic_segsoc', 'N_ic_cv', 'N_ic_sbv', 'N_ic_ali', 'N_carencias', 'N_carencias3', 'N_plb', 'N_plb_m', 'perrankin_p', 'perrankin_pe', 'cppobreza', 'cppobreza_e', 'cppobreza_m', 'cpvul_car', 'cpic_rezedu', 'cpic_asalud', 'cpic_segsoc', 'cpic_cv', 'cpic_sbv', 'cpic_ali', 'cpcarencias', 'cpcarencias3', 'cpplb', 'cpplbm', 'pobtot_00', 'pobtot_05', 'pobtot_10', 'porc_pob_15_analfa00', 'porc_pob_15_analfa05', 'porc_pob_15_analfa10', 'porc_pob614_noasiste00', 'porc_pob614_noasiste05', 'porc_pob614_noasiste10', 'porc_pob15_basicainc00', 'porc_pob15_basicainc05', 'porc_pob15_basicainc10', 'porc_pob_snservsal00', 'porc_pob_snservsal05', 'porc_pob_snservsal10', 'porc_vivpisotierra00', 'porc_vivpisotierra05', 'porc_vivpisotierra10', 'porc_vivsnsan00', 'porc_vivsnsan05', 'porc_vivsnsan10', 'porc_snaguaent00', 'porc_snaguaent05', 'porc_snaguaent10', 'porc_vivsndren00', 'porc_vivsndren05', 'porc_vivsndren10', 'porc_vivsnenergia00', 'porc_vivsnenergia05', 'porc_vivsnenergia10', 'porc_vivsnlavadora00', 'porc_vivsnlavadora05', 'porc_vivsnlavadora10', 'porc_vivsnrefri00', 'porc_vivsnrefri05', 'porc_vivsnrefr10', 'irez_soc00', 'irez_soc05', 'irez_soc10', 'l_ocupnac00', 'l_ocupnac05', 'l_ocupnac10', 'p_rez_edu_90', 'p_rez_edu_00', 'p_rez_edu_10', 'p_ser_sal_00', 'p_ser_sal_10', 'p_viv_pisos_90', 'p_viv_pisos_00', 'p_viv_pisos_10', 'p_viv_muros_90', 'p_viv_muros_00', 'p_viv_muros_10', 'p_viv_techos_90', 'p_viv_techos_00', 'p_viv_techos_10', 'p_viv_hacin_90', 'p_viv_hacin_00', 'p_viv_hacin_10', 'p_viv_agu_entub_90', 'p_viv_agu_entub_00', 'p_viv_agu_entub_10', 'p_viv_dren_90', 'p_viv_dren_00', 'p_viv_dren_000', 'p_viv_elect_90', 'p_viv_elect_00', 'p_viv_elect_10', 'pobreza_alim_90', 'pobreza_alim_00', 'pobreza_alim_10', 'pobreza_cap_90', 'pobreza_cap_00', 'pobreza_cap_10', 'pobreza_patrim_90', 'pobreza_patrim_00', 'pobreza_patrim_10', 'gini_90', 'gini_00', 'gini_10']]
y = data['Drenaje_Categoria']

# Split the data into training and testing sets (80/20)
def manual_train_test_split(X, y, test_size):
    num_samples = len(X)
    split_index = int(num_samples * (1 - test_size))
    X_train, X_test = X[:split_index], X[split_index:]
    y_train, y_test = y[:split_index], y[split_index:]
    return X_train, X_test, y_train, y_test

test_size = 0.2
X_train, X_test, y_train, y_test = manual_train_test_split(X.to_numpy(), y.to_numpy(), test_size)

# Standardize the features (important for KNN)
mean = X_train.mean(axis=0)
std = X_train.std(axis=0)
X_train = (X_train - mean) / std
X_test = (X_test - mean) / std

# KNN--------------------------------------------------
def manual_knn(X_train, y_train, X_test, k):
    y_pred = []
    for test_point in X_test:
        distances = np.linalg.norm(X_train - test_point, axis=1)
        nearest_indices = np.argsort(distances)[:k]
        nearest_labels = y_train[nearest_indices]
        prediction = np.bincount(nearest_labels).argmax()
        y_pred.append(prediction)
    return np.array(y_pred)

k = 4  # number of neighbors as needed
y_pred_knn = manual_knn(X_train, y_train, X_test, k)

# Calculate the accuracy of KNN
correct_knn = (y_pred_knn == y_test).sum()
accuracy_knn = correct_knn / len(y_test)
print("Precision of KNN model:", accuracy_knn)
#print(y_pred_knn)

# Perceptron
def manual_perceptron(X_train, y_train, X_test, learning_rate, epochs):
    num_samples, num_features = X_train.shape
    weights = np.zeros(num_features)
    bias = 0
    for _ in range(epochs):
        for i in range(num_samples):
            activation = np.dot(X_train[i], weights) + bias
            if y_train[i] * activation <= 0:
                weights += learning_rate * y_train[i] * X_train[i]
                bias += learning_rate * y_train[i]
    y_pred = np.sign(np.dot(X_test, weights) + bias)
    return y_pred

learning_rate = 0.1
epochs = 100
y_pred_perceptron = manual_perceptron(X_train, y_train, X_test, learning_rate, epochs)

# Calculate the accuracy of the Perceptron
correct_perceptron = (y_pred_perceptron == y_test).sum()
accuracy_perceptron = correct_perceptron / len(y_test)
print("Precision of Perceptron model:", accuracy_perceptron)

"""#Code with libraries"""

#codigo con librerias
import pandas as pd
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from sklearn.neighbors import KNeighborsClassifier
from sklearn.linear_model import Perceptron
from sklearn.metrics import accuracy_score

# Cargar tus datos
path = '/content/Indicadores_municipales_sabana_DA (3).csv'
data = pd.read_csv(path, sep=",", encoding='ISO-8859-1')
#data = data.dropna()
data = data.fillna(data.mean(numeric_only=True))
# Crear una columna categórica basada en 'porc_vivsndren10'
data['Drenaje_Categoria'] = pd.cut(data['porc_vivsndren10'], bins=[0, 20, 40, 60, 80, 100], labels=[1, 2, 3, 4, 5])
data.dropna(subset=['Drenaje_Categoria'], inplace=True)
# Separar las características (X) y el objetivo (y)
#X = data[['porc_vivsnenergia10', 'porc_vivsnrefri05', 'pobreza_m']]
X = data[['pobtot_ajustada', 'pobreza', 'pobreza_e', 'pobreza_m', 'vul_car', 'vul_ing', 'npnv', 'ic_rezedu', 'ic_asalud', 'ic_segsoc', 'ic_cv', 'ic_sbv', 'ic_ali', 'carencias', 'carencias3', 'plb', 'plb_m', 'rankin_p', 'rankin_pe', 'N_pobreza', 'N_pobreza_e', 'N_pobreza_m', 'N_vul_car', 'N_vul_ing', 'N_npnv', 'N_ic_rezedu', 'N_ic_asalud', 'N_ic_segsoc', 'N_ic_cv', 'N_ic_sbv', 'N_ic_ali', 'N_carencias', 'N_carencias3', 'N_plb', 'N_plb_m', 'perrankin_p', 'perrankin_pe', 'cppobreza', 'cppobreza_e', 'cppobreza_m', 'cpvul_car', 'cpic_rezedu', 'cpic_asalud', 'cpic_segsoc', 'cpic_cv', 'cpic_sbv', 'cpic_ali', 'cpcarencias', 'cpcarencias3', 'cpplb', 'cpplbm', 'pobtot_00', 'pobtot_05', 'pobtot_10', 'porc_pob_15_analfa00', 'porc_pob_15_analfa05', 'porc_pob_15_analfa10', 'porc_pob614_noasiste00', 'porc_pob614_noasiste05', 'porc_pob614_noasiste10', 'porc_pob15_basicainc00', 'porc_pob15_basicainc05', 'porc_pob15_basicainc10', 'porc_pob_snservsal00', 'porc_pob_snservsal05', 'porc_pob_snservsal10', 'porc_vivpisotierra00', 'porc_vivpisotierra05', 'porc_vivpisotierra10', 'porc_vivsnsan00', 'porc_vivsnsan05', 'porc_vivsnsan10', 'porc_snaguaent00', 'porc_snaguaent05', 'porc_snaguaent10', 'porc_vivsndren00', 'porc_vivsndren05', 'porc_vivsndren10', 'porc_vivsnenergia00', 'porc_vivsnenergia05', 'porc_vivsnenergia10', 'porc_vivsnlavadora00', 'porc_vivsnlavadora05', 'porc_vivsnlavadora10', 'porc_vivsnrefri00', 'porc_vivsnrefri05', 'porc_vivsnrefr10', 'irez_soc00', 'irez_soc05', 'irez_soc10', 'l_ocupnac00', 'l_ocupnac05', 'l_ocupnac10', 'p_rez_edu_90', 'p_rez_edu_00', 'p_rez_edu_10', 'p_ser_sal_00', 'p_ser_sal_10', 'p_viv_pisos_90', 'p_viv_pisos_00', 'p_viv_pisos_10', 'p_viv_muros_90', 'p_viv_muros_00', 'p_viv_muros_10', 'p_viv_techos_90', 'p_viv_techos_00', 'p_viv_techos_10', 'p_viv_hacin_90', 'p_viv_hacin_00', 'p_viv_hacin_10', 'p_viv_agu_entub_90', 'p_viv_agu_entub_00', 'p_viv_agu_entub_10', 'p_viv_dren_90', 'p_viv_dren_00', 'p_viv_dren_000', 'p_viv_elect_90', 'p_viv_elect_00', 'p_viv_elect_10', 'pobreza_alim_90', 'pobreza_alim_00', 'pobreza_alim_10', 'pobreza_cap_90', 'pobreza_cap_00', 'pobreza_cap_10', 'pobreza_patrim_90', 'pobreza_patrim_00', 'pobreza_patrim_10', 'gini_90', 'gini_00', 'gini_10']]
 # Reemplaza 'tus_columnas_de_interes' por las columnas relevantes
y = data['Drenaje_Categoria']

# Dividir los datos en conjuntos de entrenamiento y prueba (80 20)
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

# Escalar las características (importante para KNN)
scaler = StandardScaler()
X_train = scaler.fit_transform(X_train)
X_test = scaler.transform(X_test)
# Crear un modelo KNN
knn = KNeighborsClassifier(n_neighbors=4)  # Puedes ajustar el valor de 'n_neighbors'

# Entrenar el modelo
knn.fit(X_train, y_train)

# Realizar predicciones en el conjunto de prueba
y_pred_knn = knn.predict(X_test)

# Calcular la precisión del modelo
accuracy_knn = accuracy_score(y_test, y_pred_knn)
print("Precisión del modelo KNN:", accuracy_knn)
print(y_pred_knn)

# Crear un modelo de Perceptrón
perceptron = Perceptron(max_iter=100, random_state=42)  # Puedes ajustar los parámetros según sea necesario

# Entrenar el modelo
perceptron.fit(X_train, y_train)

# Realizar predicciones en el conjunto de prueba
y_pred_perceptron = perceptron.predict(X_test)

# Calcular la precisión del modelo
accuracy_perceptron = accuracy_score(y_test, y_pred_perceptron)
print("Precisión del modelo Perceptrón:", accuracy_perceptron)
