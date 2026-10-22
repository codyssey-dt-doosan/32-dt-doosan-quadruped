import math
import unittest
import cv2
import numpy as np
from native_gauge_reader import read_native_gauges


def cluster(angles):
    im=np.full((480,640,3),45,np.uint8)
    for x,phi in zip([160,320,480],angles):
        c=(x,240);r=45
        cv2.circle(im,c,r,(220,220,220),-1,cv2.LINE_AA)
        for degree in range(0,360,30):
            a=math.radians(degree)
            cv2.line(im,(int(x+.85*r*math.sin(a)),int(240-.85*r*math.cos(a))),(int(x+.98*r*math.sin(a)),int(240-.98*r*math.cos(a))),(20,20,20),2)
        if phi is not None:
            a=math.radians(phi)
            cv2.line(im,c,(int(x+.75*r*math.sin(a)),int(240-.75*r*math.cos(a))),(10,10,10),3,cv2.LINE_AA)
            cv2.circle(im,c,4,(10,10,10),-1)
    return im


class NativeGaugeTests(unittest.TestCase):
    def test_three_black_needles_and_original_scale(self):
        readings,_=read_native_gauges(cluster([225,0,135]))
        self.assertEqual(len(readings),3)
        for reading,expected in zip(readings,[0,50,100]):
            self.assertTrue(reading['valid'])
            self.assertAlmostEqual(reading['bar'],expected,delta=1.5)

    def test_unreadable_needle_is_not_fabricated(self):
        readings,_=read_native_gauges(cluster([225,None,135]))
        self.assertEqual(len(readings),3)
        self.assertFalse(readings[1]['valid'])
        self.assertIsNone(readings[1]['bar'])

    def test_partial_cluster_is_rejected(self):
        im=cluster([225,0,135]);im[:,410:]=45
        self.assertEqual(read_native_gauges(im)[0],[])

    def test_no_gauges_is_rejected(self):
        self.assertEqual(read_native_gauges(np.full((480,640,3),45,np.uint8))[0],[])


if __name__=='__main__':unittest.main()
