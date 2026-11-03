"""Map identity and localization checks. No ROS, serial or motion imports."""
import hashlib
import json
import math


def finite(value):
    return type(value) in (int,float) and math.isfinite(value)


def angle(value):
    return math.atan2(math.sin(value),math.cos(value))


def checked_pose(values):
    if not isinstance(values,(list,tuple)) or len(values)!=3 or not all(map(finite,values)):
        raise ValueError('Expected finite x,y,yaw')
    if abs(values[0])>100 or abs(values[1])>100 or abs(values[2])>math.pi:
        raise ValueError('Initial pose outside bounded test area/units')
    return list(values)


def stationary_certificate(reference,pose,discontinuities,v,w):
    if (reference is None or type(discontinuities) is not int or discontinuities<0
            or not all(map(finite,(v,w)))):return False
    old=reference['pose'];checked_pose(pose);checked_pose(old)
    return (reference['discontinuities']==discontinuities and abs(v)<.002 and abs(w)<.01
        and math.hypot(pose[0]-old[0],pose[1]-old[1])<.002 and abs(angle(pose[2]-old[2]))<.003)


def stability_update(count,last_key,ready,pose_stamp,scan_stamp,stationary_revalidated):
    """Recover after occlusion using distinct live scans only for a valid stop certificate."""
    key=('scan',scan_stamp) if stationary_revalidated else ('pose',pose_stamp)
    if not ready:return 0,key
    return min(5,count+1) if key!=last_key else count,key


def classify(data):
    return [0 if 0<=v<=25 else 100 if v>=65 else -1 for v in data]


def pgm(raw):
    """Strict 8-bit binary PGM parser, preserving whitespace-valued first pixel."""
    offset=0;tokens=[]
    while len(tokens)<4:
        while offset<len(raw) and raw[offset:offset+1] in b' \r\n\t':offset+=1
        if raw[offset:offset+1]==b'#':
            offset=raw.index(b'\n',offset)+1
            continue
        start=offset
        while offset<len(raw) and raw[offset:offset+1] not in b' \r\n\t':offset+=1
        if start==offset:raise ValueError('Incomplete PGM header')
        tokens.append(raw[start:offset])
    if tokens[0]!=b'P5' or tokens[3]!=b'255':raise ValueError('Expected 8-bit P5')
    width,height=int(tokens[1]),int(tokens[2])
    if not 0<width*height<=4000000:raise ValueError('Invalid map size')
    if raw[offset:offset+2]==b'\r\n':offset+=2
    elif raw[offset:offset+1] in (b' ',b'\n',b'\r',b'\t'):offset+=1
    else:raise ValueError('Missing PGM pixel separator')
    pixels=raw[offset:]
    if len(pixels)!=width*height:raise ValueError('Map pixel count differs')
    cells=[]
    for y in range(height-1,-1,-1):
        cells.extend(0 if v==254 else 100 if v==0 else -1 for v in pixels[y*width:(y+1)*width])
    return width,height,cells


def map_identity(info,data):
    # map_saver writes metric metadata with six decimal places.
    stable=dict(width=info['width'],height=info['height'],resolution=round(info['resolution'],6),
                origin=[round(v,6) for v in info['origin']],cells=classify(data))
    return hashlib.sha256(json.dumps(stable,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()


def pose_quality(pose,covariance,source_age,receive_age,after_initial,scan_score,stationary_revalidated=False):
    checked_pose(pose)
    if (len(covariance)!=36 or not all(map(finite,covariance))
            or any(covariance[k]<0 for k in (0,7,35))):raise ValueError('Invalid AMCL covariance')
    reasons=[]
    if (not finite(source_age) or not finite(receive_age) or source_age<-.05 or receive_age<0
            or (not stationary_revalidated and (source_age>1.8 or receive_age>1.8))):
        reasons.append('AMCL pose stale/future')
    if not after_initial:reasons.append('No new localization after explicit initial pose')
    if max(covariance[0],covariance[7])>.10**2:reasons.append('Position uncertainty above 10cm')
    if covariance[35]>math.radians(5)**2:reasons.append('Heading uncertainty above 5deg')
    if scan_score.get('used',0)<30 or scan_score.get('near_wall_ratio',0)<.55:
        reasons.append('Insufficient live laser/map agreement')
    return dict(ready=not reasons,reasons=reasons,position_std_m=[math.sqrt(covariance[k]) for k in (0,7)],
                yaw_std_deg=math.degrees(math.sqrt(covariance[35])),scan_score=scan_score,
                amcl_source_age_s=source_age,
                evidence_mode='stationary_laser_and_wheel_revalidated' if stationary_revalidated else 'new_amcl_pose')


def scan_match(info,cells,ranges,angle_min,increment,range_min,range_max,pose):
    """Independent endpoint consistency; unknown/outside points are not matches."""
    x,y,yaw=checked_pose(pose);ox,oy,oa=info['origin'];res=info['resolution']
    width,height=info['width'],info['height'];used=matched=0
    radius=max(1,int(math.ceil(.10/res)))
    ca,sa=math.cos(oa),math.sin(oa)
    for i in range(0,len(ranges),max(1,len(ranges)//180)):
        r=ranges[i]
        if not math.isfinite(r) or not max(range_min,.30)<=r<=min(range_max,8.):continue
        a=angle_min+i*increment
        bx,by=.13+r*math.cos(a),r*math.sin(a)
        wx=x+math.cos(yaw)*bx-math.sin(yaw)*by-ox
        wy=y+math.sin(yaw)*bx+math.cos(yaw)*by-oy
        gx,gy=int(math.floor((ca*wx+sa*wy)/res)),int(math.floor((-sa*wx+ca*wy)/res))
        used+=1
        if any(0<=gx+dx<width and 0<=gy+dy<height and
               cells[(gy+dy)*width+gx+dx]>=65 for dx in range(-radius,radius+1)
               for dy in range(-radius,radius+1) if dx*dx+dy*dy<=radius*radius):matched+=1
    return dict(used=used,matched=matched,near_wall_ratio=matched/used if used else 0.,tolerance_m=.10)
