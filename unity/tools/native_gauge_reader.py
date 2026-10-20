"""Three original dark-needle dials, read only from camera pixels.
Angles use twelve o'clock=0 degrees, clockwise. Original face texture is marked 0-100 bar.
"""
import math
import cv2
import numpy as np
from gauge_ocr.reader import Dial, GaugeSpec, rectify, phi_to_value


def locate_dials(bgr):
    h,w=bgr.shape[:2];gray=cv2.cvtColor(bgr,cv2.COLOR_BGR2GRAY)
    # The inspection camera centers the mapped instrument cluster; walls outside it are excluded.
    x0,y0,x1,y1=int(w*.08),int(h*.18),int(w*.92),int(h*.82)
    roi=gray[y0:y1,x0:x1]
    _,mask=cv2.threshold(roi,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))
    contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    found=[]
    for c in contours:
        area=cv2.contourArea(c)
        if len(c)<5 or area<80 or area>w*h*.05:continue
        (cx,cy),(a,b),angle=cv2.fitEllipse(c)
        if min(a,b)/max(a,b)<.55 or min(a,b)<12:continue
        fill=area/(math.pi*a*b/4)
        if not .75<fill<1.15:continue
        found.append(Dial(cx+x0,cy+y0,a,b,angle))
    # Three similarly sized coplanar gauges are required; no substitute reading on a partial cluster.
    best=None
    import itertools
    for group in itertools.combinations(found,3):
        group=sorted(group,key=lambda d:d.cx)
        sizes=np.array([max(d.w,d.h) for d in group])
        ys=np.array([d.cy for d in group]);gaps=np.diff([d.cx for d in group])
        if sizes.max()/sizes.min()>1.35 or np.ptp(ys)>sizes.mean()*.35:continue
        if gaps.min()<sizes.mean()*.8 or gaps.max()/gaps.min()>1.5:continue
        score=float(sizes.mean()-ys.std())
        if best is None or score>best[0]:best=(score,group)
    return best[1] if best else []


def dark_needle(face):
    gray=cv2.cvtColor(face,cv2.COLOR_BGR2GRAY)
    _,mask=cv2.threshold(gray,0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
    r=face.shape[0]/2;n=720
    polar=cv2.warpPolar(mask,(int(r),n),(r,r),r,cv2.WARP_POLAR_LINEAR)
    band=polar[:,int(r*.18):int(r*.55)]
    score=band.mean(axis=1)/255
    score=np.convolve(np.r_[score[-3:],score,score[:3]],np.ones(7)/7,'valid')
    i=int(score.argmax());opposite=score[(i+n//2)%n]
    confidence=float(score[i])
    if confidence<.30 or confidence-opposite<.12:return float('nan'),confidence
    # The first maximum biases thick needles toward one edge; average the local peak on the circle.
    indices=(i+np.arange(-40,41))%n
    weights=score[indices]**4
    angles=indices*2*math.pi/n
    theta=math.atan2(float(np.dot(weights,np.sin(angles))),float(np.dot(weights,np.cos(angles))))
    return (math.degrees(theta)+90)%360,confidence


def read_native_gauges(bgr):
    results=[];out=bgr.copy()
    for i,d in enumerate(locate_dials(bgr)):
        phi,confidence=dark_needle(rectify(bgr,d))
        valid=math.isfinite(phi)
        value=phi_to_value(phi,GaugeSpec(v_max=100.)) if valid else None
        results.append({'id':'G'+str(i+1),'angle_deg':phi if valid else None,'bar':value,'confidence':confidence,'valid':valid})
        cv2.ellipse(out,((d.cx,d.cy),(d.w,d.h),d.angle),(0,255,0) if valid else (0,180,255),2)
        label=f'G{i+1}: {value:.1f} bar' if valid else f'G{i+1}: unreadable'
        cv2.putText(out,label,(max(0,int(d.cx-d.w*.65)),max(18,int(d.cy-d.h*.7))),cv2.FONT_HERSHEY_SIMPLEX,.42,(0,255,255),1,cv2.LINE_AA)
        if valid:
            a=math.radians(phi);r=max(d.w,d.h)*.4
            cv2.line(out,(int(d.cx),int(d.cy)),(int(d.cx+math.sin(a)*r),int(d.cy-math.cos(a)*r)),(255,0,255),1)
    cv2.putText(out,'Original equipment gauges | scale 0-100 bar',(12,bgr.shape[0]-15),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,255,255),1)
    return results,out
