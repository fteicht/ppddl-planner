#ifndef RDDL_CLIENT_H
#define RDDL_CLIENT_H

#include <vector>
#include <map>
#include <string>
#include <cstring>

#include "planners/rddl_planner.h"

#include "formulas.h"
#include "actions.h"

#include "system_utils.h"
#include "string_utils.h"
//#include "strxml.h"

class CompletePlanner;
class XMLNode;

class IPPCClient {
public:
    IPPCClient(RddlPlanner* _planner,
               std::string _hostName,
               unsigned short _port,
               std::map<std::string, int> const& _stateVariableIndices,
               std::vector<std::vector<std::string> > const& _stateVariableValues) :
          planner(_planner),
          hostName(_hostName),
          port(_port),
          socket(-1),
          numberOfRounds(-1),
          remainingTime(0),
          stateVariableIndices(_stateVariableIndices),
          stateVariableValues(_stateVariableValues) {}

public:
    void run(std::string const& problemName);
    long get_time_left() {return remainingTime;}
private:
    void initConnection();
    int connectToServer();
    void closeConnection();
//    void stateTovaluemap( std::vector<double>& nextState, ValueMap& values);

    void initSession(std::string const& rddlProblem);
    void finishSession();

    void initRound(AtomSet& initialState, double& immediateReward);
    void finishRound(XMLNode const* node, double& immediateReward);

    const std::string getServerAction(std::string name, std::vector<std::string>& arguments);
    bool sendAction(const Action* action,
                    AtomSet& nextState,
                    double& immediateReward);

    void readState(XMLNode const* node,
                   AtomSet& nextState,
                   double& immediateReward);
    const Atom*  readVariable(XMLNode const* node);

    RddlPlanner* planner;
    std::string hostName;
    unsigned short port;
    int socket;

    int numberOfRounds;

    long remainingTime;

    std::map<std::string, int> stateVariableIndices;
    std::vector<std::vector<std::string> > stateVariableValues;
};

#endif
