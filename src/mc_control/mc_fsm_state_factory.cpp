#include <mc_control/mc_fsm_state_factory.h>

#include <mc_rtc/Configuration.h>

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

namespace mc_control
{

FSMStateFactory::FSMStateFactory(const std::vector<std::string> & paths,
                                 const std::vector<std::string> & files,
                                 bool verbose)
: mc_rtc::ObjectLoader<FSMState>("MC_RTC_FSM_STATE",
                                 {},
                                 false,
                                 verbose)
{
  load_libraries(paths);
  load_files(files);
}

void FSMStateFactory::load_libraries(const std::vector<std::string> & paths)
{
  mc_rtc::ObjectLoader<FSMState>::load_libraries(paths,
                                                 [this](const std::string & cn,
                                                        lt_dlhandle h)
                                                 { update(cn, h); });
}

namespace
{

// The code in this namespace allows to deal with scattered states
// information. For example if State B depends on State A but we attempt to
// load State B before State A. It also detects cycle (State B depends on
// State A, State A depends on State B).

/** Undefined state */
struct UDState
{
  std::string state;
  std::string base;
  mc_rtc::Configuration config;
};

/** One pass of resolution
 *
 * For each undefined state, if the base exists we create the state and
 * remove the undefined state from the states' vector
 *
 */
void resolve_pass(FSMStateFactory & factory, std::vector<UDState> & states)
{
  for(auto it = states.begin(); it != states.end();)
  {
    auto & uds = *it;
    if(factory.hasState(uds.base))
    {
      factory.load(uds.state, uds.base, uds.config);
      it = states.erase(it);
    }
    else
    {
      ++it;
    }
  }
}

/** Full resolution
 *
 * Do passes until a pass does nothing or all states have been created
 *
 */
void resolve(FSMStateFactory & factory, std::vector<UDState> & states)
{
  size_t prev_size = 0;
  while(states.size() != 0)
  {
    resolve_pass(factory, states);
    if(states.size() == prev_size)
    {
      break;
    }
    prev_size = states.size();
  }
  if(states.size() != 0)
  {
    LOG_ERROR("Some states could not be loaded as their base is not available, check for typos or cycles")
    for(const auto & s : states)
    {
      LOG_WARNING("- " << s.state << " (base: " << s.base << ")")
    }
  }
}

/** Build up a list of undefined states */
void load_ud(FSMStateFactory & factory, const std::map<std::string, mc_rtc::Configuration> & states, std::vector<UDState> & ud_states)
{
  for(const auto & s : states)
  {
    const auto & config = s.second;
    std::string base = config("base", std::string(""));
    if(base.empty())
    {
      LOG_ERROR("Attempted to load state " << s.first << " but no base is specified in the configuration")
      continue;
    }
    if(factory.hasState(base))
    {
      factory.load(s.first, base, config);
    }
    else
    {
      ud_states.push_back({s.first, base, config});
    }
  }
}

void load_file(FSMStateFactory & factory,
               const std::string & file,
               std::vector<UDState> & ud_states)
{
  std::map<std::string, mc_rtc::Configuration> states = mc_rtc::Configuration(file);
  load_ud(factory, states, ud_states);
}

void load_dir(FSMStateFactory & factory,
              const std::string & dir,
              std::vector<UDState> & ud_states)
{
  bfs::directory_iterator dit(dir), endit;
  std::vector<bfs::path> drange;
  std::copy(dit, endit, std::back_inserter(drange));
  for(const auto & p : drange)
  {
    if(bfs::is_regular_file(p))
    {
      load_file(factory, p.string(), ud_states);
    }
    else if(bfs::is_directory(p))
    {
      load_dir(factory, p.string(), ud_states);
    }
  }
}

}

void FSMStateFactory::load_files(const std::vector<std::string> & files)
{
  std::vector<UDState> ud_states;
  for(const auto & f : files)
  {
    if(bfs::is_directory(f))
    {
      LOG_INFO("Looking for .json state files in " << f)
      load_dir(*this, f, ud_states);
    }
    else
    {
      if(bfs::exists(f) && bfs::is_regular_file(f))
      {
        load_file(*this, f, ud_states);
      }
      else
      {
        LOG_WARNING("State file " << f << " does not exist")
      }
    }
  }
  if(ud_states.size())
  {
    resolve(*this, ud_states);
  }
}

void FSMStateFactory::load(const std::map<std::string, mc_rtc::Configuration> & states)
{
  std::vector<UDState> ud_states;
  load_ud(*this, states, ud_states);
  if(ud_states.size())
  {
    resolve(*this, ud_states);
  }
}

void FSMStateFactory::load(const std::string & name,
                           const std::string & base,
                           const mc_rtc::Configuration & config)
{
  if(!hasState(base))
  {
    LOG_ERROR_AND_THROW(std::runtime_error, "Cannot create a state using a base " << base << " that does not exist")
  }
  if(hasState(name))
  {
    LOG_ERROR_AND_THROW(std::runtime_error, "State " << name << " already exists")
  }
  states_.push_back(name);
  states_factories_[name] = [config, base](FSMStateFactory & f)
  {
    auto ret = f.create(base);
    ret->configure(config);
    return ret;
  };
  outputs_[name] = outputs_[base];
}

FSMStatePtr FSMStateFactory::create(const std::string & state,
                                    FSMController & ctl,
                                    const mc_rtc::Configuration & config)
{
  FSMStatePtr ret = create(state);
  if(!ret)
  {
    LOG_ERROR("Creation of " << state << " state failed")
    return nullptr;
  }
  ret->configure(config);
  ret->start(ctl);
  return ret;
}

FSMStatePtr FSMStateFactory::create(const std::string & state)
{
  if(!hasState(state))
  {
    LOG_ERROR("Attempted to create unavailable state " << state)
    return nullptr;
  }
  if(has_object(state))
  {
    return create_object(state);
  }
  else
  {
    return states_factories_[state](*this);
  }
}

bool FSMStateFactory::hasState(const std::string & state) const
{
  return std::find(states_.begin(), states_.end(), state) != states_.end();
}

const std::vector<std::string> & FSMStateFactory::states() const
{
  return states_;
}

const std::vector<std::string> & FSMStateFactory::stateOutputs(const std::string & state) const
{
  return outputs_.at(state);
}

bool FSMStateFactory::isValidOutput(const std::string & state,
                                    const std::string & output) const
{
  return hasState(state) &&
    std::find(outputs_.at(state).begin(),
              outputs_.at(state).end(),
              output) != outputs_.at(state).end();
}

void FSMStateFactory::update(const std::string & cn, lt_dlhandle handle)
{
  void * sym = lt_dlsym(handle, "outputs");
  if(sym == nullptr)
  {
    LOG_ERROR_AND_THROW(mc_rtc::LoaderException,
                        "Symbol outputs not found in " <<
                        lt_dlgetinfo(handle)->filename << std::endl <<
                        lt_dlerror())
  }
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wpedantic"
  auto outputs_fn = (std::vector<std::string>(*)(const std::string&))(sym);
  #pragma GCC diagnostic pop
  outputs_[cn] = outputs_fn(cn);
  if(outputs_[cn].size() == 0)
  {
    LOG_WARNING("No outputs loaded for state " << cn)
  }
  states_.push_back(cn);
}

}
