# opt_2.py

#coding: utf-8
#tsp.py : 巡回セールスマン問題     
import sys
import math
import time
from pqueue import *
from unionfind import *
from Tkinter import *
# 標準入力よりデータを読み込む
def read_data():
    buff = []
    for a in sys.stdin:
        b = a.split()
        buff.append((int(b[0]), int(b[1])))
    return buff

# 距離の計算
def distance(ps):
    size = len(ps)
    table = [[0] * size for _ in xrange(size)]
    for i in xrange(size):
        for j in xrange(size):
            if i != j:
                dx = ps[i][0] - ps[j][0]
                dy = ps[i][1] - ps[j][1]
                table[i][j] = math.sqrt(dx * dx + dy * dy)
    return table

# 経路の長さ
def path_length(path):
    global distance_table
    n = 0
    i = 1
    for i in xrange(1, len(path)):
        n += distance_table[path[i - 1]][path[i]]
    n += distance_table[path[0]][path[-1]]
    return n

### 局所探索法
# 2-opt 法
def opt_2(size, path):
    global distance_table
    total = 0
    while True:
        count = 0
        for i in xrange(size - 2):
            i1 = i + 1
            for j in xrange(i + 2, size):
                if j == size - 1:
                    j1 = 0
                else:
                    j1 = j + 1
                if i != 0 or j1 != 0:
                    l1 = distance_table[path[i]][path[i1]]
                    l2 = distance_table[path[j]][path[j1]]
                    l3 = distance_table[path[i]][path[j]]
                    l4 = distance_table[path[i1]][path[j1]]
                    if l1 + l2 > l3 + l4:
                        # つなぎかえる
                        new_path = path[i1:j+1]
                        path[i1:j+1] = new_path[::-1]
                        count += 1
        total += count
        if count == 0: break
    return path, total

# or-opt 法 (簡略版)
def or_opt(size, path):
    global distance_table
    total = 0
    while True:
        count = 0
        for i in xrange(size):
            # i 番目の都市を (j) - (j1) の経路に挿入する
            i0 = i - 1
            i1 = i + 1
            if i0 < 0: i0 = size - 1
            if i1 == size: i1 = 0
            for j in xrange(size):
                j1 = j + 1
                if j1 == size: j1 = 0
                if j != i and j1 != i:
                    l1 = distance_table[path[i0]][path[i]]  # i0 - i - i1
                    l2 = distance_table[path[i]][path[i1]]
                    l3 = distance_table[path[j]][path[j1]]  # j - j1
                    l4 = distance_table[path[i0]][path[i1]] # i0 - i1
                    l5 = distance_table[path[j]][path[i]]   # j - i - j1
                    l6 = distance_table[path[i]][path[j1]] 
                    if l1 + l2 + l3 > l4 + l5 + l6:
                        # つなぎかえる
                        p = path[i]
                        path[i:i + 1] = []
                        if i < j:
                            path[j:j] = [p]
                        else:
                            path[j1:j1] = [p]
                        count += 1
        total += count
        if count == 0: break
    return path, total

# 組み合わせ
def optimize1(size, path):
    while True:
        path, _ = opt_2(size, path)
        path, flag = or_opt(size, path)
        if flag == 0: return path

def optimize2(size, path):
    while True:
        path, _ = or_opt(size, path)
        path, flag = opt_2(size, path)
        if flag == 0: return path

### 単純な欲張り法 (Nearest Neighbor 法)
def greedy0(path):
    global distance_table
    size = len(path)
    for i in xrange(size - 1):
        min_len = 1000000
        min_pos = 0
        for j in xrange(i + 1, size):
            l = distance_table[path[i]][path[j]]
            if l < min_len:
                min_len = l
                min_pos = j
        path[i + 1], path[min_pos] = path[min_pos], path[i + 1]
    return path

### クラスカルのアルゴリズムの変形版
# 辺の定義

class Edge:
    def __init__(self, p1, p2, weight):
        self.p1 = p1
        self.p2 = p2
        self.weight = weight

    def __cmp__(x, y):
        return x.weight - y.weight


# 辺のデータを作成
def make_edge(size):
    global distance_table
    edges = PQueue()
    for i in xrange(0, size - 1):
        for j in xrange(i + 1, size):
            e = Edge(i, j, distance_table[i][j])
            edges.push(e)
    return edges

# 辺から経路へ
def edge_to_path(edges, size):
    def search_edge(x):
        r = []
        for i in xrange(size):
            if edges[i].p1 == x:
                r.append(edges[i].p2)
            elif edges[i].p2 == x:
                r.append(edges[i].p1)
        return r
    #
    path = [0] * size
    for i in xrange(size - 1):
        x, y = search_edge(path[i])
        if i == 0:
            path[i + 1] = x
            path[-1] = y
        elif path[i - 1] == x:
            path[i + 1] = y
        else:
            path[i + 1] = x
    return path

# 探索
def greedy1(size):
    edges = make_edge(size)
    edge_count = [0] * size
    u = UnionFind(size)
    i = 0
    select_edge = []
    while i < size:
        e = edges.pop()
        if edge_count[e.p1] < 2 and edge_count[e.p2] < 2 and (u.find(e.p1) != u.find(e.p2) or i == size - 1):
            u.union(e.p1, e.p2)
            edge_count[e.p1] += 1
            edge_count[e.p2] += 1
            select_edge.append(e)
            i += 1
    return edge_to_path(select_edge, size)

### データの入力
###
point_table = read_data()
point_size = len(point_table)
distance_table = distance(point_table)

### 実行
###
s = time.clock()
if sys.argv[1] == 'tsp1':
    path = greedy0(range(point_size))
elif sys.argv[1] == 'tsp2':
    path = greedy1(point_size)
#else:
#   path = divide_merge(range(point_size))
print path_length(path)
print time.clock() - s

# 局所探索法 (逐次改善法)
if len(sys.argv) > 2:
    s = time.clock()
    if sys.argv[2] == '2-opt':
        path, _ = opt_2(point_size, path)
    elif sys.argv[2] == 'or-opt':
        path, _ = or_opt(point_size, path)
    elif sys.argv[2] == 'opt1':
        path = optimize1(point_size, path)
    else:
        path = optimize2(point_size, path)
    print path_length(path)
    print time.clock() - s



import os
for f in os.listdir('/home/shuukasei/tfidf/tfidf2/wsj'):
        m=open('/home/shuukasei/tfidf/tfidf2/wsj/'+f,'r').read()
#       s=r.findall(m)
#       for text in s:
        print m
