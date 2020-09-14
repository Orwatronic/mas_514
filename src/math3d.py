from numpy import array, cos, sin, tan, identity

def Rx(phi):
    return array([
        [1, 0       , 0       ],
        [0, cos(phi),-sin(phi)],
        [0, sin(phi), cos(phi)]
    ])

def Ry(theta):
    return array([
        [ cos(theta), 0, sin(theta)],
        [ 0         , 1, 0         ],
        [-sin(theta), 0, cos(theta)]
    ])

def Rz(psi):
    return array([
        [cos(psi),-sin(psi), 0],
        [sin(psi), cos(psi), 0],
        [0       , 0       , 1]
    ])

def Hrx(phi):
    return array([
        [1, 0       , 0       , 0],
        [0, cos(phi),-sin(phi), 0],
        [0, sin(phi), cos(phi), 0],
        [0, 0       , 0       , 1]
    ])

def Hry(theta):
    return array([
        [ cos(theta), 0, sin(theta), 0],
        [ 0         , 1, 0         , 0],
        [-sin(theta), 0, cos(theta), 0],
        [ 0         , 0, 0         , 1]
    ])

def Hrz(psi):
    return array([
        [cos(psi),-sin(psi), 0, 0],
        [sin(psi), cos(psi), 0, 0],
        [0       , 0       , 1, 0],
        [0       , 0       , 0, 1]
    ])

def Htx(x):
    return array([
        [1, 0, 0, x],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

def Hty(y):
    return array([
        [1, 0, 0, 0],
        [0, 1, 0, y],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

def Htz(z):
    return array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, z],
        [0, 0, 0, 1]
    ])

def inv(H):
    '''
    Inverse homogenous transformation
    '''
    R = H[0:3,0:3]
    d = H[0:3,3]

    invH = identity(4)

    invH[0:3,0:3] = R.transpose()
    invH[0:3,3] = -R.transpose().dot(d)

    return invH

def DH(theta, d, a, alpha):
    '''
    Deneavit-Hartenberg transformation matrix
    ''' 
    return array([
        [ cos(theta),-sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta)],
        [ sin(theta), cos(theta)*cos(alpha),-cos(theta)*sin(alpha), a*sin(theta)],
        [ 0         , sin(alpha)           , cos(alpha)           , d           ],
        [ 0         , 0                    , 0                    , 1           ]
    ])
