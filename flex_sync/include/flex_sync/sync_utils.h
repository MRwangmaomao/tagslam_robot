/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#ifndef FLEX_SYNC_SYNC_UTILS_H
#define FLEX_SYNC_SYNC_UTILS_H

#include <ros/ros.h>

#include <boost/shared_ptr.hpp>

#include <string>
#include <vector>
#include <map>
#include <stdexcept>

namespace flex_sync {
  // helper function to make vector from elements in queue,
  // and clear out the queue. 
  template<typename T>
  static std::vector<boost::shared_ptr<T const>>
  make_vec(const ros::Time &t,
           const std::vector<std::string> &topics,
           std::map<std::string,
           std::map<ros::Time, boost::shared_ptr<T const> >> *topicToQueue) {
    std::vector<boost::shared_ptr<T const>> mvec;
    for (const auto &topic: topics) {
      auto &t2m = (*topicToQueue)[topic]; // time to message
      if (t2m.empty()) {
        throw std::runtime_error(topic + " has empty queue!");
      }
      while (!t2m.empty() && t2m.begin()->first < t) {
        t2m.erase(t2m.begin());
      }
      if (t2m.empty()) {
        ROS_ERROR_STREAM(topic << " queue is empty for time: " << t);
        throw std::runtime_error(topic + " queue is empty!");
      }
      if (t2m.begin()->first != t) {
        throw std::runtime_error(topic + " has wrong time stamp!");
      }
      mvec.push_back(t2m.begin()->second);
      t2m.erase(t2m.begin());
    }
    return (mvec);
  }

  // helper function to update the message count in map
  static inline std::map<ros::Time, int>::iterator update_count(
    const ros::Time &t, std::map<ros::Time, int> *msgCountMap) {
    // check if we have this time stamp
    std::map<ros::Time, int>::iterator it = msgCountMap->find(t);
    if (it == msgCountMap->end()) { // no messages received for this tstamp
      msgCountMap->insert(std::map<ros::Time,int>::value_type(t, 1));
      it = msgCountMap->find(t);
    } else {
      it->second++; // bump number of received messages
    }
    return (it);
  }

  // helper function to drop message
  static inline void decrease_count(const ros::Time &t,
                             std::map<ros::Time, int> *msgCountMap) {
    // check if we have this time stamp
    std::map<ros::Time, int>::iterator it = msgCountMap->find(t);
    if (it == msgCountMap->end()) {
      // no messages received for this tstamp, should not happen!
      ROS_ERROR_STREAM("no count for " << t);
      return;
    } else {
      it->second--;
      if (it->second <= 0) {
        msgCountMap->erase(it);
      }
    }
  }

}

#endif
