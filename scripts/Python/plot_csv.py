# -*- coding: utf-8 -*-
# Grafica directamente con matplotlib uno o varios .csv generados por
# bag2csv.py (o por bag2csv_crazyflie.py): una figura por fichero, una
# curva por columna numérica frente a Timestamp. Pensado para una
# inspección rápida sin tener que abrir Matlab.
#
# Uso: python3 plot_csv.py fichero1.csv [fichero2.csv ...]

import csv
import sys

import matplotlib.pyplot as plt


def plot_file(path):
    with open(path, newline='') as f:
        reader = csv.reader(f)
        header = next(reader)
        rows = list(reader)

    if not rows:
        print('%s: vacío, se omite' % path)
        return

    t0 = float(rows[0][0])
    timestamps = [(float(r[0]) - t0) / 1e9 for r in rows]  # ns -> s, relativo al inicio

    numeric_cols = []
    for col in range(1, len(header)):
        try:
            float(rows[0][col])
            numeric_cols.append(col)
        except ValueError:
            pass

    if not numeric_cols:
        print('%s: no tiene columnas numéricas que graficar (¿es un topic de texto?)' % path)
        return

    plt.figure(path)
    for col in numeric_cols:
        values = [float(r[col]) for r in rows]
        plt.plot(timestamps, values, label=header[col])
    plt.xlabel('Tiempo [s]')
    plt.title(path)
    plt.legend(loc='best')
    plt.grid(True, which='both', linestyle=':')


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print('Uso: python3 plot_csv.py fichero1.csv [fichero2.csv ...]')
        sys.exit(1)

    for csv_path in sys.argv[1:]:
        plot_file(csv_path)

    plt.show()
