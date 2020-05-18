#!/usr/bin/env python
#
# 2019 Bernd Pfrommer
#
""" Class for synchronizing stamped messages

  usage example:


  from flex_sync import Sync

  class SyncListener:
    def __init__(self):
      pass
    def callback(self, msgvec):
      print "got messages: ", len(msgvec)

  listener  = SyncListener()
  topics=['/bar', '/foo'])
  sync      = Sync(topics, listener)
  bag       = rosbag.Bag(bagfile, 'r')
  iterator  = bag.read_messages(topics=topics)
  for (topic, msg, time) in iterator:
      sync.process(topic, msg)

"""

from collections import defaultdict
from threading import Lock

class Sync:
    def __init__(self, a_topics, l):
        self.topics   = a_topics
        self.listener = l
        self.lock = Lock()
        self.reset()

    def reset(self):
        self.lock.acquire()
        try:
            # msgMap maintains a queue for each topic
            self.msgMap   = defaultdict(lambda: defaultdict(lambda: {}))
            # msgCount counts how many messages have arrived for
            # a given timestamp
            self.msgCount = defaultdict(lambda: 0)
        finally:
            self.lock.release()

        
    def process(self, topic, msg):
        """ returns True if sync was complete, and callback made. """
        try:
            self.lock.acquire()
            if not hasattr(msg, 'header'):
                return False # filter anything that has no header
            t = msg.header.stamp
            # insert map into message array for this time stamp
            self.msgMap[topic][t] = msg
            #  bump the number of messages arrived for this
            #  time stamp
            self.msgCount[t]      = self.msgCount[t] + 1
            if self.msgCount[t] == len(self.topics):
                self.currentTime = t
                mvec = self.makeVec(t) # cleans out msgMap as well!
                self.listener(mvec)
                # now clean out the msgCount array
                for t_del in list(self.msgCount):
                    if t_del <= t:
                        del self.msgCount[t_del]
                return True
        finally:
            self.lock.release()

        return False

    def makeVec(self, t):
        mvec = []
        for topic in self.topics:
            mmap = self.msgMap[topic] # queue for this topic
            # erase all old messages in this queue
            # and append the current one to the vector
            for t_del in sorted(mmap):
                if t_del < t:
                    del mmap[t_del]
                else:
                    mvec.append(mmap[t_del])
                    del mmap[t_del]
                    break
        return mvec
