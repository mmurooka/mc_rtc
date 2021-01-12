/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/log/utils.h>
#include <mc_rtc/logging.h>
#include <mc_rtc/utils_api.h>

#include <memory>
#include <unordered_map>

namespace mc_rtc
{

struct LoggerImpl;
/*! \brief Logs controller data to disk
 *
 * The user can select a desired logging policy that will impact the behaviour
 * of this class.
 *
 * See mc_rtc::Logger::Policy documentation for details on available
 * policies
 */
struct MC_RTC_UTILS_DLLAPI Logger
{
public:
  /** Magic number used to identify binary logs */
  static const uint8_t magic[4];
  /** A function that fills LogData vectors */
  typedef std::function<void(mc_rtc::MessagePackBuilder &)> serialize_fn;
  /*! \brief Defines available policies for the logger */
  enum struct Policy
  {
    /*! \brief Non-threaded policy
     *
     * Using this policy, the logging disk operations are done in the same
     * thread as the global controller running thread, this is well suited
     * for simulations and environments where you can guarantee a very fast
     * access to the disk (e.g. logging to a ramdisk)
     */
    NON_THREADED = 0,
    /*! \brief Threaded policy
     *
     * Using this policy, the logging disk operations are done in a separate
     * thread from the global controller running thread. As a result, some
     * buffering occurs and you might lose some data if the controller
     * crashes. This is intended for real-time environments.
     */
    THREADED = 1
  };

public:
  /*! \brief Constructor
   *
   * \param policy The chosen logging policy
   *
   * \param directory Path to the directory where log files will be stored
   *
   * \param tmpl Log file template
   */
  Logger(const Policy & policy, const std::string & directory, const std::string & tmpl);

  /*! \brief Destructor */
  ~Logger();

  /*! \brief Setup the constructor configuration
   *
   * \param policy The chosen logging policy
   *
   * \param directory Path to the directory where log files will be stored
   *
   * \param tmpl Log file template
   */
  void setup(const Policy & policy, const std::string & directory, const std::string & tmpl);

  /*! \brief Start logging
   *
   * Print the file header to the log. This should be called at initialization
   * or when a controller switch occurs
   *
   * \param ctl_name Name of the running controller
   *
   * \param timestep Time increment for the log time entry
   *
   * \param resume If true, start the time entry at the current value,
   * otherwise, start at 0
   */
  void start(const std::string & ctl_name, double timestep, bool resume = false);

  /*! \brief Log controller's data
   *
   * Print controller data to the log.
   *
   */
  void log();

  /** Add a log entry into the log with the provided source
   *
   * This function only accepts callable objects that returns a l/rvalue to a
   * serializable object.
   *
   * The log entry can be removed by calling:
   * - \ref removeLogEntry with the same name
   * - \ref removeLogEntries with the same source
   *
   * The later remove all other log entries from the same source
   *
   * \param name Name of the log entry, this should be unique at any given time
   * but the same key can be re-used during the logger's life
   *
   * \param source Source of the log entry
   *
   * \param get_fn A function that provides data that should be logged
   *
   */
  template<typename CallbackT,
           typename SourceT = void,
           typename std::enable_if<mc_rtc::log::callback_is_serializable<CallbackT>::value, int>::type = 0>
  void addLogEntry(const std::string & name, const SourceT * source, CallbackT && get_fn)
  {
    using ret_t = decltype(get_fn());
    using base_t = typename std::decay<ret_t>::type;
    if(log_entries_.count(name))
    {
      log::error("Already logging an entry named {}", name);
      return;
    }
    log_entries_changed_ = true;
    log_entries_[name] = {source, [get_fn](mc_rtc::MessagePackBuilder & builder) mutable {
                            mc_rtc::log::LogWriter<base_t>::write(get_fn(), builder);
                          }};
  }

  /** Add a log entry from a source and a pointer to member
   *
   * This is an helper above the source + callback version
   *
   * \param name Name of the log entry
   *
   * \param source Source of the log entry
   *
   * \param member Member pointer for the source data
   *
   */
  template<typename SourceT,
           typename MemberT,
           typename std::enable_if<mc_rtc::log::is_serializable<MemberT>::value, int>::type = 0>
  void addLogEntry(const std::string & name, const SourceT * source, MemberT SourceT::*member)
  {
    addLogEntry(name, source, [source, member]() -> const MemberT & { return source->*member; });
  }

  /** Add a log entry from a source and a pointer to method
   *
   * This is an helper above the source + callback version
   *
   * \param name Name of the log entry
   *
   * \param source Source of the log entry
   *
   * \param method Method pointer for the source data
   */
  template<
      typename SourceT,
      typename MethodT,
      typename std::enable_if<mc_rtc::log::is_serializable<typename std::decay<MethodT>::type>::value, int>::type = 0>
  void addLogEntry(const std::string & name, const SourceT * source, MethodT (SourceT::*method)() const)
  {
    addLogEntry(name, source, [source, method]() -> MethodT { return (source->*method)(); });
  }

  /** Add a log entry into the log with no source
   *
   * This function only accepts callable objects that returns a l/rvalue to a
   * serializable object.
   *
   * The log entry can only be removed by calling \ref removeLogEntry with the same name
   *
   * \param name Name of the log entry, this should be unique at any given time
   * but the same key can be re-used during the logger's life
   *
   * \param get_fn A function that provides data that should be logged
   *
   */
  template<typename T, typename std::enable_if<mc_rtc::log::callback_is_serializable<T>::value, int>::type = 0>
  void addLogEntry(const std::string & name, T && get_fn)
  {
    addLogEntry(name, static_cast<const void *>(nullptr), std::forward<T>(get_fn));
  }

  /** Remove a log entry from the log
   *
   * This has no effect if the log entry does not exist.
   *
   * \param name Name of the entry
   *
   */
  void removeLogEntry(const std::string & name);

  /** Remove all log entries from a given source
   *
   * This has no effect if no log entry is associated to the given source.
   *
   * \param source Source whose entries should be removed
   *
   */
  void removeLogEntries(const void * source);

  /** Return the time elapsed since the controller start */
  double t() const;

  /** Access the file opened by this Logger
   *
   * \note This is empty before \ref start has been called
   */
  const std::string & path() const;

  /** Flush the log data to disk (only implemented in the synchronous method) */
  void flush();

private:
  /** Hold information about a log entry stored in this instance */
  struct LogEntry
  {
    /** What is the data source (can be nullptr) */
    const void * source;
    /** Callback to log data */
    serialize_fn log_cb;
  };
  /** Store implementation detail related to the logging policy */
  std::shared_ptr<LoggerImpl> impl_ = nullptr;
  /** Set to true when log entries are added or removed */
  bool log_entries_changed_ = false;
  /** Contains all the log entries */
  std::unordered_map<std::string, LogEntry> log_entries_;
};

} // namespace mc_rtc
