#!/usr/bin/env python3
"""Train a small NumPy-only MLP to imitate the logged (state -> command) pairs
from data_logger.py. No ROS or ML framework dependency -- run directly with
plain python3, no need to source the workspace:

    python3 train_avoidance_model.py --data ~/capstone_avoidance_data/*.csv
"""
import argparse
import csv
import glob
import os

import numpy as np

FEATURE_COLS = [
    'left_dist', 'center_dist', 'right_dist',
    'person_dist', 'person_present',
    'teleop_linear_x', 'teleop_angular_z',
]
LABEL_COLS = ['cmd_linear_x', 'cmd_angular_z']


def load_csvs(patterns, source_filter=None):
    paths = []
    for pattern in patterns:
        paths.extend(sorted(glob.glob(os.path.expanduser(pattern))))
    if not paths:
        raise FileNotFoundError(f'No CSV files matched: {patterns}')

    rows = []
    skipped = 0
    for path in paths:
        with open(path) as f:
            reader = csv.DictReader(f)
            for r in reader:
                # Older logs predate the controller_source column -- treat those
                # rows as unfiltered/unknown rather than erroring out.
                source = r.get('controller_source', 'unknown')
                if source_filter is not None and source != source_filter:
                    skipped += 1
                    continue
                rows.append(r)
    print(f'Loaded {len(rows)} rows from {len(paths)} file(s): {[os.path.basename(p) for p in paths]}'
          + (f' ({skipped} rows skipped, source != "{source_filter}")' if source_filter else ''))
    if not rows:
        raise ValueError(f'No rows left after filtering for controller_source == "{source_filter}"')

    X = np.array([[float(r[c]) for c in FEATURE_COLS] for r in rows], dtype=np.float64)
    Y = np.array([[float(r[c]) for c in LABEL_COLS] for r in rows], dtype=np.float64)
    return X, Y


class MLP:
    """A minimal 2-hidden-layer MLP with manual forward/backward passes and
    Adam, implemented in plain NumPy (no autograd/framework dependency)."""

    def __init__(self, n_in, n_hidden, n_out, seed=0):
        rng = np.random.default_rng(seed)
        scale1 = np.sqrt(2.0 / n_in)
        scale2 = np.sqrt(2.0 / n_hidden)
        self.W1 = rng.normal(0, scale1, (n_in, n_hidden))
        self.b1 = np.zeros(n_hidden)
        self.W2 = rng.normal(0, scale2, (n_hidden, n_hidden))
        self.b2 = np.zeros(n_hidden)
        self.W3 = rng.normal(0, scale2, (n_hidden, n_out))
        self.b3 = np.zeros(n_out)
        self.params = ['W1', 'b1', 'W2', 'b2', 'W3', 'b3']
        self._adam_m = {p: np.zeros_like(getattr(self, p)) for p in self.params}
        self._adam_v = {p: np.zeros_like(getattr(self, p)) for p in self.params}
        self._adam_t = 0

    def forward(self, X):
        z1 = X @ self.W1 + self.b1
        h1 = np.maximum(z1, 0)  # ReLU
        z2 = h1 @ self.W2 + self.b2
        h2 = np.maximum(z2, 0)
        out = h2 @ self.W3 + self.b3  # linear output (regression)
        cache = (X, z1, h1, z2, h2)
        return out, cache

    def backward(self, cache, grad_out):
        X, z1, h1, z2, h2 = cache
        n = X.shape[0]

        gW3 = h2.T @ grad_out / n
        gb3 = grad_out.mean(axis=0)
        grad_h2 = grad_out @ self.W3.T
        grad_z2 = grad_h2 * (z2 > 0)

        gW2 = h1.T @ grad_z2 / n
        gb2 = grad_z2.mean(axis=0)
        grad_h1 = grad_z2 @ self.W2.T
        grad_z1 = grad_h1 * (z1 > 0)

        gW1 = X.T @ grad_z1 / n
        gb1 = grad_z1.mean(axis=0)

        return {'W1': gW1, 'b1': gb1, 'W2': gW2, 'b2': gb2, 'W3': gW3, 'b3': gb3}

    def adam_step(self, grads, lr, beta1=0.9, beta2=0.999, eps=1e-8):
        self._adam_t += 1
        for p in self.params:
            g = grads[p]
            self._adam_m[p] = beta1 * self._adam_m[p] + (1 - beta1) * g
            self._adam_v[p] = beta2 * self._adam_v[p] + (1 - beta2) * (g * g)
            m_hat = self._adam_m[p] / (1 - beta1 ** self._adam_t)
            v_hat = self._adam_v[p] / (1 - beta2 ** self._adam_t)
            update = lr * m_hat / (np.sqrt(v_hat) + eps)
            setattr(self, p, getattr(self, p) - update)

    def save(self, path, feature_mean, feature_std, label_mean, label_std):
        np.savez(
            path,
            W1=self.W1, b1=self.b1, W2=self.W2, b2=self.b2, W3=self.W3, b3=self.b3,
            feature_mean=feature_mean, feature_std=feature_std,
            label_mean=label_mean, label_std=label_std,
            feature_cols=np.array(FEATURE_COLS), label_cols=np.array(LABEL_COLS),
        )


def main():
    parser = argparse.ArgumentParser(description='Train the NumPy avoidance-imitation MLP.')
    parser.add_argument('--data', nargs='+', default=['~/capstone_avoidance_data/*.csv'],
                         help='CSV file(s) or glob pattern(s) from data_logger.py')
    parser.add_argument('--output', default=None,
                         help='Where to save the trained weights (default: capstone_robot/models/avoidance_mlp.npz)')
    parser.add_argument('--hidden', type=int, default=16, help='Hidden layer width')
    parser.add_argument('--epochs', type=int, default=300)
    parser.add_argument('--batch-size', type=int, default=64)
    parser.add_argument('--lr', type=float, default=1e-3)
    parser.add_argument('--val-split', type=float, default=0.15)
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--source', default=None,
                         help='Only train on rows logged with this controller_source '
                              '(e.g. "fused_obstacle_avoider"). Default: use all rows.')
    args = parser.parse_args()

    X, Y = load_csvs(args.data, source_filter=args.source)

    rng = np.random.default_rng(args.seed)
    n = X.shape[0]
    idx = rng.permutation(n)
    X, Y = X[idx], Y[idx]
    n_val = max(1, int(n * args.val_split))
    X_val, Y_val = X[:n_val], Y[:n_val]
    X_train, Y_train = X[n_val:], Y[n_val:]
    print(f'Train: {X_train.shape[0]} rows, Val: {X_val.shape[0]} rows')

    feature_mean, feature_std = X_train.mean(axis=0), X_train.std(axis=0) + 1e-8
    label_mean, label_std = Y_train.mean(axis=0), Y_train.std(axis=0) + 1e-8

    Xn_train = (X_train - feature_mean) / feature_std
    Yn_train = (Y_train - label_mean) / label_std
    Xn_val = (X_val - feature_mean) / feature_std
    Yn_val = (Y_val - label_mean) / label_std

    model = MLP(n_in=X.shape[1], n_hidden=args.hidden, n_out=Y.shape[1], seed=args.seed)

    n_train = Xn_train.shape[0]
    for epoch in range(args.epochs):
        perm = rng.permutation(n_train)
        Xn_train, Yn_train = Xn_train[perm], Yn_train[perm]
        epoch_loss = 0.0
        for start in range(0, n_train, args.batch_size):
            xb = Xn_train[start:start + args.batch_size]
            yb = Yn_train[start:start + args.batch_size]
            pred, cache = model.forward(xb)
            err = pred - yb
            loss = float(np.mean(err ** 2))
            epoch_loss += loss * xb.shape[0]
            grad_out = 2 * err / xb.shape[0]
            grads = model.backward(cache, grad_out)
            model.adam_step(grads, lr=args.lr)
        epoch_loss /= n_train

        if epoch % 20 == 0 or epoch == args.epochs - 1:
            val_pred, _ = model.forward(Xn_val)
            val_loss = float(np.mean((val_pred - Yn_val) ** 2))
            print(f'epoch {epoch:4d}  train_mse={epoch_loss:.5f}  val_mse={val_loss:.5f}')

    output_path = args.output
    if output_path is None:
        this_dir = os.path.dirname(os.path.abspath(__file__))
        output_path = os.path.join(this_dir, '..', 'models', 'avoidance_mlp.npz')
    model.save(output_path, feature_mean, feature_std, label_mean, label_std)
    print(f'Saved trained model to {os.path.abspath(output_path)}')


if __name__ == '__main__':
    main()
