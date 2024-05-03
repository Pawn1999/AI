import pandas as pd
import numpy as np


class Node:
    def __init__(self, data, label, dim, val):
        self.data = data
        self.label = label
        self.left = None
        self.right = None
        self.dim = dim
        self.val = val


def build_kd_tree(X, y, depth=0):
    if len(X) == 0:
        return None
    if len(X) == 1:
        return Node(X[0], y[0], depth, None)
    dim = depth % len(X[0])
    X = np.array(X)
    y = np.array(y)
    idx = np.argsort(X[:, dim])
    X_sorted = X[idx]
    y_sorted = y[idx]
    mid = len(X) // 2
    node = Node(X_sorted[mid], y_sorted[mid], dim, X_sorted[mid][dim])
    node.left = build_kd_tree(X_sorted[:mid], y_sorted[:mid], depth + 1)
    node.right = build_kd_tree(X_sorted[mid + 1:], y_sorted[mid + 1:], depth + 1)
    return node


def distance(x1, x2):
    x1 = np.array(x1)
    x2 = np.array(x2)
    return np.sqrt(np.sum((x1 - x2) ** 2))


def search(node, x):
    if node is None:
        return None
    if node.left is None and node.right is None:
        return node.label
    if x[node.dim] <= node.val:
        next_node = node.left
    else:
        next_node = node.right
    nn_label = search(next_node, x)
    if nn_label is not None and distance(node.data, x) < distance(next_node.data, x):
        nn_label = node.label
    return nn_label


# Load the training and test data into pandas dataframes
train_df = pd.read_csv('wine_quality_train.csv',sep='\s+')
test_df = pd.read_csv('wine_quality_test.csv',sep='\s+')

# Remove leading and trailing whitespaces from column names
train_df.columns = train_df.columns.str.strip()
test_df.columns = test_df.columns.str.strip()

# Separate features and labels
X_train = train_df.drop('quality', axis=1).values
y_train = train_df['quality'].values
X_test = test_df.drop('quality', axis=1).values
y_test = test_df['quality'].values

# Build the kd-tree using the training data
root = build_kd_tree(X_train, y_train)

# Find the nearest neighbor of each point in the test data
predictions = []
for i in range(len(X_test)):
    nn_label = search(root, X_test[i])
    predictions.append(nn_label)

# Calculate accuracy of predictions
accuracy = np.sum(predictions == y_test) / len(y_test)
print("Accuracy:", accuracy)
