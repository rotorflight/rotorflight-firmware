#!/bin/env python3

import numpy as np
import matplotlib.pyplot as plt


X = [ -1, -0.5, 0, 0.5, 1 ]
Y = [ -1, -0.5, 0, 0.5, 1 ]

Y[0] *= 0.98
Y[1] *= 1.10
Y[3] *= 0.95
Y[4] *= 1.21


def gen_spline(x,y):
    u = [ 0, 0, 0, 0, 0 ]
    v = [ 0, 0, 0, 0, 0 ]

    for i in [1,2,3]:
        S = (x[i]-x[i-1]) / (x[i+1]-x[i-1])
        P = S * v[i-1] + 2.0
        v[i] = (S - 1.0) / P
        print(f'v[{i}] = {v[i]} P = {P}')

        U = ((y[i+1] - y[i]) / (x[i+1] - x[i])) - ((y[i] - y[i-1]) / (x[i] - x[i-1]))
        u[i] = (6.0 * U / (x[i+1] - x[i-1]) - S * u[i-1]) / P

    for k in [3,2,1,0]:
        v[k] = v[k] * v[k+1] + u[k]

    for k in [0,1,2,3,4]:
        print(f'k={k}:: v[{k}] = {v[k]}')

    return v


def test1_spline(x,y):
    u = [ 0, 0, 0, 0, 0 ]
    v = [ 0, -0.25, -0.26666666666666666, -0.26666666666666666, 0 ]
    p = [ 0, 4.0, 3.75, 3.7333333333333334, 0 ]

    for i in [1,2,3]:
        u[i] = ( 24*(y[i+1] - 2*y[i] + y[i-1]) - u[i-1] ) / p[i]

    for k in [3,2,1,0]:
        v[k] = v[k] * v[k+1] + u[k]

    for k in [0,1,2,3,4]:
        print(f'k={k}:: v[{k}] = {v[k]}')

    return v

def test2_spline(x,y):
    u = [ 0, 0, 0, 0, 0 ]
    v = [ 0, 0, 0, 0, 0 ]

    u[0] = 0
    u[1] = ((y[2] - 2*y[1] + y[0])        ) / 4
    u[2] = ((y[3] - 2*y[2] + y[1]) - u[1] ) / 3.75
    u[3] = ((y[4] - 2*y[3] + y[2]) - u[2] ) / 3.7333333333333334
    u[4] = 0

    v[4] = u[4]
    v[3] = u[3]
    v[2] = -0.26666666666666666 * v[3] + u[2]
    v[1] = -0.25 * v[2] + u[1]
    v[0] = u[0]

    for k in [0,1,2,3,4]:
        print(f'k={k}:: v[{k}] = {v[k]}')

    return v

def test3_spline(x,y):
    u = [ 0, 0, 0, 0, 0 ]

    u[0] = 0
    u[1] = ((y[2] - 2*y[1] + y[0])        ) / 4
    u[2] = ((y[3] - 2*y[2] + y[1]) - u[1] ) / 3.75
    u[3] = ((y[4] - 2*y[3] + y[2]) - u[2] ) / 3.7333333333333334
    u[4] = 0

    u[2] = -0.26666666666666666 * u[3] + u[2]
    u[1] = -0.25 * u[2] + u[1]

    for k in [0,1,2,3,4]:
        print(f'k={k}:: v[{k}] = {u[k] * 24}')

    return u


def speval(x,y,v,X,j):
    i = j - 1
    h = x[j] - x[i]
    a = (x[j] - X) / h
    b = (X - x[i]) / h
    return a * y[i] + b * y[j] + ((a*a*a - a) * v[i] + (b*b*b - b) * v[j]) * (h*h) / 6.0

def spline(x,y,v,X):
    for i in [1,2,3]:
        if X < x[i]:
            return speval(x,y,v,X,i)
    return speval(x,y,v,X,4)


def speval2(x,y,v,X,j):
    i = j - 1
    a = 2 * (x[j] - X)
    b = 1 - a
    return a * y[i] + b * y[j] + (a*a*a - a) * v[i] + (b*b*b - b) * v[j]

def spline2(x,y,v,X):
    for i in [1,2,3]:
        if X < x[i]:
            return speval2(x,y,v,X,i)
    return speval2(x,y,v,X,4)


###

XX = np.linspace(-1.25, 1.25, 1000)
YY = np.zeros_like(XX)
ZZ = np.zeros_like(XX)

V = gen_spline(X,Y)
W = test3_spline(X,Y)

for i in range(1000):
    YY[i] = spline(X,Y,V,XX[i])
    ZZ[i] = spline2(X,Y,W,XX[i])

plt.plot(XX,YY,'g')
plt.plot(XX,ZZ,'b')
plt.plot(X,Y,'ro')
plt.show()

