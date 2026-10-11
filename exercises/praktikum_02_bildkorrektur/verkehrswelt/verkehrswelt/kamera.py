"""Gemeinsame Hilfsfunktionen: Kalibrierdatei lesen, Vogelperspektive beschreiben."""
import numpy as np
import yaml


def lade_kalibrierung(pfad):
    """Liest K und D aus der Datei ost.yaml von camera_calibration."""
    with open(pfad) as f:
        d = yaml.safe_load(f)
    K = np.array(d['camera_matrix']['data'], dtype=np.float64).reshape(3, 3)
    D = np.array(d['distortion_coefficients']['data'], dtype=np.float64)
    groesse = (d['image_width'], d['image_height'])
    return K, D, groesse


def boden_nach_pixel(x, y, x_max, y_max, px_pro_m):
    """Bodenpunkt (x vorwärts, y links, in m, base_footprint) -> Pixel (Spalte, Zeile) der Vogelperspektive.
    Oben im Bild liegt x_max, links liegt y_max."""
    return (y_max - y) * px_pro_m, (x_max - x) * px_pro_m


def pixel_nach_boden(spalte, zeile, x_max, y_max, px_pro_m):
    return x_max - zeile / px_pro_m, y_max - spalte / px_pro_m
