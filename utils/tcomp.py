from numpy import array, sin, cos, pi
import numpy as np
from utils.AngleWrap import AngleWrap

def tcomp(tab,tbc):
    '''
    Composition of transformations given by poses
    '''
    if (tab.shape[0] != 3):
        raise Exception('TCOMP: tab is not a valid transformation!')
    if (tbc.shape[0] != 3):
        raise Exception('TCOMP: tbc is not a valid transformation!')

    ang = tab[2, 0]+tbc[2, 0]
    
    if ang > pi or ang <= -pi:
        ang = AngleWrap(ang)

    s = sin(tab[2, 0])
    c = cos(tab[2, 0])
    tac = np.vstack((
        tab[0:2] + array([[c, -s],[s, c]]) @ tbc[0:2],
        ang
        ))
   
    return tac

def inv_tcomp(toa,tob):
    '''
    Inverse Composition of transformations given by poses
    '''
    if (toa.shape[0] != 3):
        raise Exception('TCOMP: tab is not a valid transformation!')
    if (tob.shape[0] != 3):
        raise Exception('TCOMP: tbc is not a valid transformation!')

    ang = tob[2, 0]-toa[2, 0]
    
    if ang > pi or ang <= -pi:
        ang = AngleWrap(ang)

    s = sin(toa[2, 0])
    c = cos(toa[2, 0])

    diff = tob - toa
    tab = np.vstack((
        array([[c, s],[-s, c]]) @ diff[0:2],
        ang
        ))
   
    return tab

def inv_pose(pose):
    ang = pose[2, 0]
    if ang > pi or ang <= -pi:
        ang = AngleWrap(ang)
    s = sin(ang)
    c = cos(ang)

    return np.vstack((
        array([[-c, -s],[s, -c]]) @ pose[0:2],
        -ang
    ))
