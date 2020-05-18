#!/usr/bin/python
#
# script to compare poses from odom and transforms from two different bags
#
# The first bag is the reference bag, the second one the one to be tested
# against the first one.
#
# example run:
#
# ./compare_poses.py odom/body_rig ~/.ros/t1.bag ~/.ros/t2.bag  -f2 body_
#

import rosbag, rospy, numpy as np
import tf
import argparse
import sys
import tf_conversions.posemath as pm
from collections import defaultdict

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class PoseStats():
    def __init__(self):
        self.missing, self.extra, self.common = (0,0,0)
        self.angle_error, self.position_error = (0.0, 0.0)
        self.angle_error_max = 0
        self.angle_error_max_time = 0
        self.angle_error_max_topic = ''
        self.position_error_max = 0
        self.position_error_max_time = 0
        self.position_error_max_topic = ''
    def print_stats(self):
        print 'missing: ', self.missing
        print 'extra: ',   self.extra
        print 'common: ',   self.common
        print 'total angle error: ', self.angle_error
        print 'total position error: ', self.position_error
        if self.common > 0:
            print 'per-pose angle error: ', self.angle_error / self.common
            print 'per-pose position error: ', self.position_error/self.common
            print 'max position error: ', self.position_error_max, \
                ' ', self.position_error_max_time, \
                ' ', self.position_error_max_topic
            print 'max angle error: ', self.angle_error_max, \
                ' ', self.angle_error_max_time, \
                ' ', self.angle_error_max_topic
    def good(self, thresh):
        if self.missing !=0 or self.extra != 0 :
            return False
        if self.common > 0:
            if (self.angle_error /self.common > thresh or
                self.position_error / self.common > thresh):
                return False
        return True

def compare_transforms(t1, p1, t2, p2):
    stats = PoseStats()
    ts1 = set(t1)
    ts2 = set(t2)
    for t in ts1 & ts2: # common time stamp
        ps1 = set(p1[t].keys())
        ps2 = set(p2[t].keys())
        tf_common = ps1 & ps2 # common transform labels
        for tfc in tf_common:
            d = np.matmul(np.linalg.inv(p1[t][tfc][2]),p2[t][tfc][2])
            dx = np.linalg.norm(d[0:3,3])
            angle, n, pt = tf.transformations.rotation_from_matrix(d)
            stats.angle_error = stats.angle_error + angle
            stats.position_error = stats.position_error + dx
            stats.common = stats.common + 1
        stats.missing = stats.missing + len(ps1.difference(ps2))
        stats.extra = stats.extra + len(ps2.difference(ps1))
    for t in ts1.difference(ts2):
        stats.missing = stats.missing + len(p1[t].keys())
    for t in ts2.difference(ts1):
        stats.extra = stats.extra + len(p2[t].keys())
    return stats

def compare_odom(t1, p1, t2, p2):
    stats = PoseStats()
    ts1 = set(t1)
    ts2 = set(t2)
    for t in ts1 & ts2: # common time stamp
        ps1 = set(p1[t].keys())
        ps2 = set(p2[t].keys())
        tp_common = ps1 & ps2 # common topics
        for tp in tp_common:
            p1inv = p1[t][tp].Inverse()
            delta = p1inv * p2[t][tp]
            da = delta.M.GetRotAngle()[0]
            dx = delta.p.Norm()
            stats.angle_error = stats.angle_error + da
            stats.position_error = stats.position_error + dx
            stats.common = stats.common + 1
            if da > stats.angle_error_max:
                stats.angle_error_max = da
                stats.angle_error_max_time = t
                stats.angle_error_max_topic = tp
            if dx > stats.position_error_max:
                stats.position_error_max = dx
                stats.position_error_max_time = t
                stats.position_error_max_topic = tp
        stats.missing = stats.missing + len(ps1.difference(ps2))
        stats.extra = stats.extra + len(ps2.difference(ps1))
    for t in ts1.difference(ts2):
        stats.missing = stats.missing + len(p1[t].keys())
    for t in ts2.difference(ts1):
        stats.extra = stats.extra + len(p2[t].keys())
    return stats

def read_odom(fname, verbose=False):
    bag = rosbag.Bag(fname, 'r')
    if not bag:
        raise 'cannot open bag' + fname
    iterator = bag.read_messages()
    t = []
    p = defaultdict(dict)
    for (topic, msg, time) in iterator:
        if msg._type == 'nav_msgs/Odometry':
            T = pm.fromMsg(msg.pose.pose)
            t.append(msg.header.stamp)
            p[msg.header.stamp][topic] = T
    if verbose:
        print 'read %d poses from %s' % (len(t), fname)
    return np.array(t), p

def read_tf(fname, verbose=False):
    bag = rosbag.Bag(fname, 'r')
    if not bag:
        raise 'cannot open bag' + fname
    iterator = bag.read_messages(topics='/tf')
    t = []
    p = defaultdict(dict)
    for (topic, msg, time) in iterator:
        if msg._type == 'tf/tfMessage':
            t.append(time)
            for i in msg.transforms:
                tr = i.transform.translation
                q  = i.transform.rotation
                m  = tf.transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
                m[0:3,3] = [tr.x, tr.y, tr.z]
                key = i.header.frame_id + '->' + i.child_frame_id
                p[time][key]=(i.header.frame_id, i.child_frame_id, m)
    if verbose:
        print 'read %d transforms from %s' % (len(t), fname)
    return np.array(t), p

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='compare transforms and odom from two different bags.')
    parser.add_argument('--verbose',  type=bool, action='store', default=False,
                        help='verbose printout')
    parser.add_argument('--threshold',  type=float, action='store', default=1e-6,
                        help='average error threshold')

    parser.add_argument('bagfile1')
    parser.add_argument('bagfile2')
    args = parser.parse_args()

    t1, p1 = read_odom(args.bagfile1, args.verbose)
    t2, p2 = read_odom(args.bagfile2, args.verbose)
    pose_stats  = compare_odom(t1, p1, t2, p2)
    
    t1, p1 = read_tf(args.bagfile1, args.verbose)
    t2, p2 = read_tf(args.bagfile2, args.verbose)
    tf_stats = compare_transforms(t1, p1, t2, p2)
    if (args.verbose or not pose_stats.good(args.threshold)
        or not tf_stats.good(args.threshold)):
        print '----- odom ------'
        pose_stats.print_stats()
        print '----- transforms ----'
        tf_stats.print_stats()
    if pose_stats.good(args.threshold) and tf_stats.good(args.threshold):
        sys.exit(0)
    print 'TEST FAILED!'
    sys.exit(-1)
