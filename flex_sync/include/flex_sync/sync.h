/* -*-c++-*--------------------------------------------------------------------
 * 2018 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#ifndef FLEX_SYNC_SYNC_H
#define FLEX_SYNC_SYNC_H

#include "flex_sync/sync_utils.h"
#include <ros/ros.h>
#include <functional>
#include <boost/shared_ptr.hpp>
#include <string>
#include <vector>
#include <map>
#include <mutex>


/*
 * Class for synchronizing across variable number of messages
 */

namespace flex_sync {
  class SyncBase {
  public:
    using string = std::string;
    using Time   = ros::Time;
    typedef std::map<Time, int> CountMap;
    // these are so we can use vector and map as shorthand
    template<class F> using vector = std::vector<F>;
    template<class F, class K> using map = std::map<F, K>;

    SyncBase(int numTopics, int qs) :
      maxQueueSize_(qs),
      topicsVec_(numTopics)
      {
    }

    virtual ~SyncBase() {};

    const Time getCurrentTime() const {
      std::unique_lock<std::mutex> lock(mutex_);
      return (currentTime_);
    }

    void setMaxQueueSize(int qs) {
      maxQueueSize_ = qs;
    }

    unsigned int qs() const { return (maxQueueSize_); }
    unsigned int getNumberDropped() const { return (numDropped_); }

    void clearNumberDropped() { numDropped_ = 0; }

    virtual void publishMessages(const Time &t) = 0;


  protected:
    template<typename P>
    bool addTopic(const std::string &topic, int idx,
                  map<string, map<Time, P>> *msgMap) {
      if (msgMap->count(topic) == 0) {
        (*msgMap)[topic] = map<Time, P>();
        topics_.push_back(topic);
        while (topicsVec_.size() <= (unsigned int)idx) {
          topicsVec_.push_back(std::vector<std::string>());
        }
        topicsVec_[idx].push_back(topic);
        return (true);
      }
      ROS_WARN_STREAM("duplicate sync topic added: " << topic);
      return (false);
    }

    template<typename P>
    void process(const std::string &topic, P msg,
                 map<string, map<Time, P>> *msgMap) {
      std::unique_lock<std::mutex> lock(mutex_);
      const Time &t = msg->header.stamp;
      auto &q   = (*msgMap)[topic];
      auto qit = q.find(t);
      // store message in a per-topic queue
      if (qit != q.end()) {
        ROS_WARN_STREAM("duplicate on topic " << topic << " ignored, t=" << t);
        return;
      }
      if (q.size() >= maxQueueSize_) {
        auto it = q.begin();
        decrease_count(it->first, &msgCount_);
        q.erase(it);
        numDropped_++;
      }
      q.insert(typename map<Time, P>::value_type(t, msg));
      // update the map that counts how many
      // messages we've received for that time slot
      auto it = update_count(t, &msgCount_);
      if (it->second > (int) topics_.size()) {
        ROS_WARN_STREAM("flex_sync: " << topic << " has " << it->second
                        << " msgs for " << (int)topics_.size() << " topics");
      }
      if (it->second >= (int) topics_.size()) {
        // got a full set of messages for that time slot
        currentTime_ = t;
        // publishMessages also cleans out old messages from the queues
        publishMessages(t);
        // clean out old entries from the message counting map
        it++;
        msgCount_.erase(msgCount_.begin(), it);
      }
    }
    // ------------ variables -----------
    unsigned int    maxQueueSize_{0};
    vector<string>  topics_;
    vector<vector<string>>  topicsVec_;
    Time            currentTime_{0.0};
    CountMap        msgCount_;
    unsigned int    numDropped_{0};
    mutable std::mutex      mutex_;
  };

  // declare variadic template arguments,
  // then specialize below 
  template <typename ... Ts> class Sync{};

  template <typename T1>
  class Sync<T1>: public SyncBase {
    typedef boost::shared_ptr<T1 const> T1ConstPtr;
    typedef map<string, map<Time, T1ConstPtr>> MsgMap1;
  public:
    typedef std::function<void(const vector<T1ConstPtr> &)> Callback;
    
    Sync(const vector<vector<string>> &topics, const Callback &callback,
         unsigned int qs = 5) : SyncBase(1, qs),
                                callback_(callback)  {
      // initialize time-to-message maps for each topic
      if (topics.size() != 1) {
        ROS_ERROR_STREAM("topics vector must have size 1!");
        return;
      }
      for (const auto &topic: topics[0]) {
        SyncBase::addTopic(topic, 0, &msgMap1_);
      }
    }

    void addTopic(const std::string &topic) {
      SyncBase::addTopic(topic, 0, &msgMap1_);
    }

    void process(const std::string &topic, const T1ConstPtr &msg) {
      SyncBase::process(topic, msg, &msgMap1_);
    }

  private:
    void publishMessages(const Time &t) override {
      vector<T1ConstPtr> mvec = make_vec(t, topics_, &msgMap1_);
      callback_(mvec);
    }

    MsgMap1   msgMap1_;
    Callback  callback_;
  };
  
  template <class T1, class T2>
  class Sync<T1,T2>: public SyncBase {
    using string = std::string;
    using Time   = ros::Time;
    template<class F> using vector = std::vector<F>;
    template<class F, class K> using map = std::map<F, K>;
  public:
    typedef T1 Type1;
    typedef T2 Type2;
    typedef boost::shared_ptr<T1 const> T1ConstPtr;
    typedef boost::shared_ptr<T2 const> T2ConstPtr;
    typedef std::function<void(const vector<T1ConstPtr> &,
                               const vector<T2ConstPtr> &)> Callback;
    typedef map<string, map<Time, T1ConstPtr>> MsgMap1;
    typedef map<string, map<Time, T2ConstPtr>> MsgMap2;

    Sync(const vector<vector<string>> &topics,
          const Callback &callback,
          unsigned int maxQueueSize = 5) :
      SyncBase(2, maxQueueSize),
      callback_(callback) {
      // initialize time-to-message maps for each topic
      if (topics.size() != 2) {
        ROS_ERROR_STREAM("topics vector must have size 2!");
        return;
      }
      for (const auto &topic: topics[0]) {
        addTopic1(topic);
      }
      for (const auto &topic: topics[1]) {
        addTopic2(topic);
      }
    }

    void addTopic1(const std::string &topic) {
      SyncBase::addTopic(topic, 0, &msgMap1_);
    }
    void addTopic2(const std::string &topic) {
      SyncBase::addTopic(topic, 1, &msgMap2_);
    }

    void process(const std::string &topic, const T1ConstPtr &msgPtr) {
      SyncBase::process(topic, msgPtr, &msgMap1_);
    }

    void process(const std::string &topic, const T2ConstPtr &msgPtr) {
      SyncBase::process(topic, msgPtr, &msgMap2_);
    }

  private:
    void publishMessages(const Time &t) override {
      vector<T1ConstPtr> mvec1 = make_vec<T1>(t, topicsVec_[0], &msgMap1_);
      vector<T2ConstPtr> mvec2 = make_vec<T2>(t, topicsVec_[1], &msgMap2_);
      callback_(mvec1, mvec2);
    }

    Callback        callback_;
    MsgMap1         msgMap1_;
    MsgMap2         msgMap2_;
  };
  
  /* -------------------------- 3 different types ----------------- */
  
  template <typename T1, typename T2, typename T3>
  class Sync<T1,T2,T3>: public SyncBase {
  public:
    typedef T1 Type1;
    typedef T2 Type2;
    typedef T3 Type3;
    typedef boost::shared_ptr<T1 const> T1ConstPtr;
    typedef boost::shared_ptr<T2 const> T2ConstPtr;
    typedef boost::shared_ptr<T3 const> T3ConstPtr;
    typedef std::function<void(const vector<T1ConstPtr> &,
                               const vector<T2ConstPtr> &,
                               const vector<T3ConstPtr> &)> Callback;
    typedef map<string, map<Time, T1ConstPtr>> MsgMap1;
    typedef map<string, map<Time, T2ConstPtr>> MsgMap2;
    typedef map<string, map<Time, T3ConstPtr>> MsgMap3;

    Sync(const vector<vector<string>> &topics,
          const Callback &callback,
          unsigned int maxQueueSize = 5) :
      SyncBase(3, maxQueueSize),
      callback_(callback) {
      // initialize time-to-message maps for each topic
      if (topics.size() != 3) {
        ROS_ERROR_STREAM("topics vector must have size 3!");
        return;
      }
      for (const auto &topic: topics[0]) {
        addTopic1(topic);
      }
      for (const auto &topic: topics[1]) {
        addTopic2(topic);
      }
      for (const auto &topic: topics[2]) {
        addTopic3(topic);
      }
    }

    void addTopic1(const std::string &topic) {
      SyncBase::addTopic(topic, 0, &msgMap1_);
    }
    void addTopic2(const std::string &topic) {
      SyncBase::addTopic(topic, 1, &msgMap2_);
    }
    void addTopic3(const std::string &topic) {
      SyncBase::addTopic(topic, 2, &msgMap3_);
    }

    void process(const std::string &topic, const T1ConstPtr &msgPtr) {
      SyncBase::process(topic, msgPtr, &msgMap1_);
    }

    void process(const std::string &topic, const T2ConstPtr &msgPtr) {
      SyncBase::process(topic, msgPtr, &msgMap2_);
    }

    void process(const std::string &topic, const T3ConstPtr &msgPtr) {
      SyncBase::process(topic, msgPtr, &msgMap3_);
    }

  private:
    void publishMessages(const Time &t) override {
      vector<T1ConstPtr> mvec1 = make_vec<T1>(t, topicsVec_[0], &msgMap1_);
      vector<T2ConstPtr> mvec2 = make_vec<T2>(t, topicsVec_[1], &msgMap2_);
      vector<T3ConstPtr> mvec3 = make_vec<T3>(t, topicsVec_[2], &msgMap3_);
      callback_(mvec1, mvec2, mvec3);
    }

    Callback        callback_;
    MsgMap1         msgMap1_;
    MsgMap2         msgMap2_;
    MsgMap3         msgMap3_;
  };
}

#endif
