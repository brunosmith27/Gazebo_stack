#!/usr/bin/env python3
"""Evaluate the trained avoidance model against logged data: confusion
matrices for discretized linear/angular decisions, plus regression metrics.
No ROS dependency -- run directly with plain python3:

    python3 evaluate_model.py --data ~/capstone_avoidance_data/*.csv --source fused_obstacle_avoider
"""
import argparse
import os

import numpy as np

from train_avoidance_model import load_csvs

# Discretize the continuous linear_x/angular_z into buckets that map to
# recognizable driving decisions, so the confusion matrix answers a readable
# question ("does it stop when it should stop?") instead of an opaque
# regression number.
LINEAR_BUCKETS = [
    ('REVERSE', lambda v: v < -0.1),
    ('STOP', lambda v: (v >= -0.1) & (v <= 0.1)),
    ('SLOW', lambda v: (v > 0.1) & (v < 0.6)),
    ('FULL', lambda v: v >= 0.6),
]
ANGULAR_BUCKETS = [
    ('RIGHT', lambda v: v < -0.1),
    ('STRAIGHT', lambda v: (v >= -0.1) & (v <= 0.1)),
    ('LEFT', lambda v: v > 0.1),
]


def load_model(npz_path):
    return np.load(npz_path, allow_pickle=True)


def predict_batch(d, X):
    fmean, fstd = d['feature_mean'], d['feature_std']
    lmean, lstd = d['label_mean'], d['label_std']
    Xn = (X - fmean) / fstd
    h1 = np.maximum(Xn @ d['W1'] + d['b1'], 0)
    h2 = np.maximum(h1 @ d['W2'] + d['b2'], 0)
    out = h2 @ d['W3'] + d['b3']
    return out * lstd + lmean


def bucketize(values, buckets):
    labels = np.empty(values.shape, dtype=object)
    for name, cond in buckets:
        labels[cond(values)] = name
    return labels


def confusion_matrix(actual_labels, pred_labels, class_names):
    idx = {c: i for i, c in enumerate(class_names)}
    n = len(class_names)
    mat = np.zeros((n, n), dtype=int)
    for a, p in zip(actual_labels, pred_labels):
        mat[idx[a], idx[p]] += 1
    return mat


def print_confusion_matrix(mat, class_names, title):
    print(f'\n{title}  (rows=actual, cols=predicted)')
    col_w = max(9, max(len(c) for c in class_names) + 1)
    print(' ' * 10 + ''.join(f'{c:>{col_w}s}' for c in class_names))
    for i, c in enumerate(class_names):
        print(f'{c:>10s}' + ''.join(f'{mat[i, j]:>{col_w}d}' for j in range(len(class_names))))
    total = mat.sum()
    correct = int(np.trace(mat))
    print(f'accuracy: {correct}/{total} = {100 * correct / total:.1f}%')
    # per-class recall -- how often each actual class was predicted correctly,
    # since overall accuracy alone hides a model that's great on the common
    # class (e.g. FULL speed) and useless on rare-but-critical ones (STOP).
    for i, c in enumerate(class_names):
        row_total = mat[i].sum()
        if row_total:
            print(f'  recall for actual={c}: {mat[i, i]}/{row_total} = {100 * mat[i, i] / row_total:.1f}%')


def main():
    parser = argparse.ArgumentParser(
        description='Evaluate the trained avoidance model: confusion matrices + regression metrics.'
    )
    parser.add_argument('--data', nargs='+', default=['~/capstone_avoidance_data/*.csv'],
                         help='CSV file(s) or glob pattern(s) from data_logger.py')
    parser.add_argument('--model', default=None,
                         help='Path to .npz model (default: capstone_robot/models/avoidance_mlp.npz)')
    parser.add_argument('--source', default=None,
                         help='Only evaluate on rows logged with this controller_source')
    args = parser.parse_args()

    model_path = args.model
    if model_path is None:
        this_dir = os.path.dirname(os.path.abspath(__file__))
        model_path = os.path.join(this_dir, '..', 'models', 'avoidance_mlp.npz')

    X, Y = load_csvs(args.data, source_filter=args.source)
    d = load_model(model_path)
    pred = predict_batch(d, X)

    print(f'Evaluated on {X.shape[0]} rows against {os.path.abspath(model_path)}')

    mae = np.mean(np.abs(pred - Y), axis=0)
    rmse = np.sqrt(np.mean((pred - Y) ** 2, axis=0))
    print(f'linear_x   MAE={mae[0]:.4f}  RMSE={rmse[0]:.4f}')
    print(f'angular_z  MAE={mae[1]:.4f}  RMSE={rmse[1]:.4f}')

    names_lin = [b[0] for b in LINEAR_BUCKETS]
    mat_lin = confusion_matrix(bucketize(Y[:, 0], LINEAR_BUCKETS), bucketize(pred[:, 0], LINEAR_BUCKETS), names_lin)
    print_confusion_matrix(mat_lin, names_lin, 'Linear (forward/back) confusion matrix')

    names_ang = [b[0] for b in ANGULAR_BUCKETS]
    mat_ang = confusion_matrix(bucketize(Y[:, 1], ANGULAR_BUCKETS), bucketize(pred[:, 1], ANGULAR_BUCKETS), names_ang)
    print_confusion_matrix(mat_ang, names_ang, 'Angular (steering) confusion matrix')


if __name__ == '__main__':
    main()
