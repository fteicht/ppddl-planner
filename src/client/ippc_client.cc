#include "ippc_client.h"

#include <iostream>
#include <sstream>
#include <cstdlib>
#include <sys/socket.h>
#include <netdb.h>
#include <unistd.h>

using namespace std;

void IPPCClient::run(string const& problemName) {
    // Init connection to the rddlsim server
    initConnection();
    // planner.xml_client_ = this

    // Request round
    initSession(problemName);

    AtomSet atoms;
    ValueMap values;
    vector<double> nextState(stateVariableIndices.size());
    double immediateReward = 0.0;

   // Builds up the values
    // {
    //     const Function* reward_function = planner->get_problem().domain().functions().find_function("reward");

    //     if (! reward_function)
    //         std::cerr << "WARNING!!! domain contains no reward function" << std::endl;

    //     ValueMap::iterator reward_fluent = values.insert(std::make_pair(Fluent::fluent(*reward_function), Rational(0))).first;
    // }



    // Main loop 
    // execute the specified number of rounds; NB: we ignore round_time 
    for (unsigned int currentRound = 0; currentRound < numberOfRounds; ++currentRound) {
        // Gets the reward and the initial state
        initRound(atoms, immediateReward);

        // stateTovaluemap(nextState, values);
       
        planner->initRound(numberOfRounds, remainingTime);

        while (true) {
            // AA: Gets the current state, hashes it
            // planner->initStep(nextState, remainingTime);
         
            const Action *a = planner->decideAction(atoms, values, remainingTime);

            if ( !sendAction(a, atoms, immediateReward) )
                break;

             planner->endRound();
        }
    }

    // Get end of session message and print total result
    finishSession();

    // Close connection to the rddlsim server
    closeConnection();
}

/******************************************************************************
                               Server Communication
******************************************************************************/

void IPPCClient::initConnection() {
    // assert (socket == -1);
    try {
        socket = connectToServer();
        if (socket <= 0) {
            std::cerr << "Error: couldn't connect to server." << std::endl;
            exit(socket);
        }
    } catch (const exception& e) {
        std::cerr << "Error: couldn't connect to server." << std::endl; 
        exit(socket);       
    } catch (...) {
        std::cerr << "Error: couldn't connect to server." << std::endl;
        exit(socket);       
    }
}

int IPPCClient::connectToServer() {
    struct hostent* host = ::gethostbyname(hostName.c_str());
    if (!host) {
        return -1;
    }

    int res = ::socket(PF_INET, SOCK_STREAM, 0);
    if (res == -1) {
        return -1;
    }

    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr = *((struct in_addr*) host->h_addr);
    memset(&(addr.sin_zero), '\0', 8);

    if (::connect(res, (struct sockaddr*) &addr, sizeof(addr)) == -1) {
        return -1;
    }
    return res;
}

void IPPCClient::closeConnection() {
    if (socket == -1) {
        std::cerr <<"Error: couldn't disconnect from server." << std::endl;
        exit(-1);
    }
    close(socket);
}

/******************************************************************************
                     Session and rounds management
******************************************************************************/

void IPPCClient::initSession(string const& rddlProblem ) {
    stringstream os;
 os << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>" << "<session-request>"
       << "<problem-name>elevators_inst_mdp__4</problem-name>"
       << "<client-name>" << "prost" << "</client-name>"
       << "<no-header/>" << "</session-request>" << '\0';
    if (write(socket, os.str().c_str(), os.str().length()) == -1) {
        std::cerr <<"Error: writing to socket failed." << std::endl;
        exit(-1);
    } //" << rddlProblem << "

    const XMLNode* serverResponse = read_node(socket);
    if (!serverResponse) {
        std::cerr <<"Error: initializing session failed on " << rddlProblem << std::endl;
        exit(-1);
    }

    string s;
    if (!serverResponse->dissect("num-rounds", s)) {
        std::cerr <<"Error: server response insufficient." << std::endl;
        exit(-1);
    }
    numberOfRounds = atoi(s.c_str());

    if (!serverResponse->dissect("time-allowed", s)) {
        std::cerr <<"Error: server response insufficient." << std::endl;
        exit(-1);
    }
    remainingTime = atoi(s.c_str());

    delete serverResponse;

    // AA: Initialise le # de rounds et les vecteurs des actions et des rewards
    //planner->initSession(numberOfRounds, remainingTime);
}

void IPPCClient::finishSession() {
    XMLNode const* sessionEndResponse = read_node(socket);
    
    if (sessionEndResponse->getName() != "session-end") {
        std::cerr <<"Error: session end message insufficient." << std::endl;
        exit(-1);
    }

    string s;
    if (!sessionEndResponse->dissect("total-reward", s)) {
        std::cerr <<"Error: session end message insufficient." << std::endl;
        exit(-1);
    }

    delete sessionEndResponse;

    // AA: Something to do?
    //     double totalReward = atof(s.c_str());
    // planner->finishSession(totalReward);
}

void IPPCClient::initRound(AtomSet& initialState, double& immediateReward) {
    stringstream os;
    os.str("");
    os << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
       << "<round-request/>" << '\0';

    if (write(socket, os.str().c_str(), os.str().length()) == -1) {
        std::cerr <<"Error: writing to socket failed." << std::endl;
        exit(-1);
    }

    XMLNode const* serverResponse = read_node(socket);

    if (!serverResponse || serverResponse->getName() != "round-init") {
        std::cerr <<"Error: round-request response insufficient." << std::endl;
        exit(-1);
    }

    string s;
    if (!serverResponse->dissect("time-left", s)) {
        std::cerr <<"Error: round-request response insufficient." << std::endl;
        exit(-1);
    }
    remainingTime = atoi(s.c_str());

    delete serverResponse;

    serverResponse = read_node(socket);
    // std::cout << "-----------Server Response---------\n";
    // std::cout << serverResponse << std::endl << "-----------------\n";

    readState(serverResponse, initialState, immediateReward);
    // assert(MathUtils::doubleIsEqual(immediateReward, 0.0));

    delete serverResponse;
}

void IPPCClient::finishRound(XMLNode const* node, double& immediateReward) {
    // TODO: Move immediate rewards
    string s;
    if (!node->dissect("immediate-reward", s)) {
        std::cerr <<"Error: round end message insufficient."<< std::endl;
        exit(-1);
    }
    immediateReward = atof(s.c_str());

    if (!node->dissect("round-reward", s)) {
        std::cerr <<"Error: server communication failed." << std::endl;
        exit(-1);
    }
    double roundReward = atof(s.c_str());
    
    // AA: What to do with the eventual  reward?

    //  planner->finishStep(immediateReward);
    planner->endRound(roundReward);
}

/******************************************************************************
                         Submission of actions
******************************************************************************/


/* Sends an action on the given stream. */
bool IPPCClient::sendAction(const Action* action, 
                            AtomSet& nextState,
                            double& immediateReward) {
    stringstream os;
    std::string noop ("noop");

    std::vector<string> arguments;
    string action_name = getServerAction( action->name(), arguments);

     std::cout << "Sending action: " <<  action_name  << std::endl;

    os << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n" << "<actions>";

    // If action is "noop" don't send anything
    if (action_name.compare(noop) != 0) {
        os << "<action><action-name>" << action_name << "</action-name>";
        if (arguments.size() == 0)
            os << "<action-arg></action-arg>";
        else
            for (std::vector<string>::iterator oi = arguments.begin() ; oi != arguments.end(); ++oi)
            {
                os << "<action-arg>" << *oi << "</action-arg>";
            }
        os << "<action-value>true</action-value></action>";
    }   
    os << "</actions>" << '\0';
    
    // std::cout << os.rdbuf() << std::endl;

    if (write(socket, os.str().c_str(), os.str().length()) == -1) {
        return false;
    }

    XMLNode const* serverResponse = read_node(socket);
    
    bool roundContinues = true;
    if (serverResponse->getName() == "round-end") {
        finishRound(serverResponse, immediateReward);
        roundContinues = false;
    } else {
        readState(serverResponse, nextState, immediateReward);
    }

    delete serverResponse;
    return roundContinues;
}


/******************************************************************************
                             Receiving of states
******************************************************************************/

void IPPCClient::readState(XMLNode const* node, AtomSet& nextState, double& immediateReward) {
    if (node->getName() != "turn")
    {
        std::cerr << "Error in receiving the turn.\n" << std::endl;   
        exit(-1);
    }

    if (node->size() == 2 &&
        node->getChild(1)->getName() == "no-observed-fluents") {
        std::cerr << "No state/observations received.\n" << std::endl;   
        exit(-1);
    }
    // AA: Vedere l'ordine dei vari nodi... magari time-left e imm-rew vengono prima
    map<string, string> newValues;

    string s;
    if (!node->dissect("time-left", s)) {
        std::cerr <<"Error: turn response message insufficient."<< std::endl;   
        exit(-2);
    }
    remainingTime = atoi(s.c_str());

    if (!node->dissect("immediate-reward", s)) {
        std::cerr << "Error: turn response message insufficient."<< std::endl;   
        exit(-2);
    }
    immediateReward = atof(s.c_str());

    std::cout << " remaining time : " << remainingTime << ", imm. Reward: " << immediateReward << std::endl;   

    for (int i = 0; i < node->size(); i++) {
        XMLNode const* child = node->getChild(i);
        if (child->getName() == "observed-fluent") {
            const Atom *a = readVariable(child);
            if (a != NULL) {
                nextState.insert( a );
                // std::cout << "Atoms: "<< std::endl << (*a).predicate() << " " << nextState.size() << std::endl;           
            }
        }
    }

    // for(map<string,string>::iterator it = newValues.begin(); it != newValues.end(); ++it) {
    //     string varName = it->first;
    //     string value = it->second;

    //     // If the variable has no parameters, its name is different from the one
    //     // that is used by PROST internally where no parents are used (afaik,
    //     // this changed at some point in rddlsim, and I am not sure if it will
    //     // change back which is why this hacky solution is fine for the moment).
    //     if(varName[varName.length()-2] == '(') {
    //         varName = varName.substr(0,varName.length()-2);
    //     }

    //     if(stateVariableIndices.find(varName) != stateVariableIndices.end()) {
    //         if(stateVariableValues[stateVariableIndices[varName]].empty()) {
    //             // TODO: This should be a numerical variable without
    //             // value->index mapping, but it can also be a boolean one atm.
    //             if(value =="true") {
    //                 nextState[stateVariableIndices[varName]] = 1.0;
    //             } else if(value == "false") {
    //                 nextState[stateVariableIndices[varName]] = 0.0;
    //             } else {
    //                 nextState[stateVariableIndices[varName]] = atof(value.c_str());
    //             }
    //         } else {
    //             for(unsigned int i = 0; i < stateVariableValues[stateVariableIndices[varName]].size(); ++i) {
    //                 if(stateVariableValues[stateVariableIndices[varName]][i] == value) {
    //                     nextState[stateVariableIndices[varName]] = i;
    //                     break;
    //                 }
    //             }
    //         }
    //     }
    // }
}

const Atom* IPPCClient::readVariable(XMLNode const* node) {
    string name;
    std::ostringstream os;
    os.str("");

    if (!node || node->getName() != "observed-fluent") {
             std::cerr<<"<client> ERROR: There is something wrong with the atom node"<<std::endl;
             exit(-1);
    }

    if (!node->dissect("fluent-name", name)) {
        std::cout<<"<client> ERROR: There is something wrong with the fluent's name"<<std::endl;
        exit(-1);
    }

    // to convert the predicate name to the planner's internal representation, 
    // replace a;; '-' with '_'.
    for (size_t i = 0; i < name.size(); i++)
    {
        if (name[i] == '-')
        {
            name[i] = '_';
        }
    }
 
  // In the planner's naming convention, an atom is characterized by a string 
  // name__arg1_arg2, where "name" is the name of th e predicate, and arg1, arg2,
  // etc. are the names of the constants to which the predicate is applied. 
  // The code below takes an XML message containing the name of a predicate and
  // its arguments, and puts them into the above form.
    os << name;
    bool val = false;
    bool bFirstArg = true;
    for( int i = 0; i < node->size(); ++i )
    {
        const XMLNode* termNode = node->getChild( i );
        if( !termNode )
	{
            continue;
	}
        else if (termNode->getName() == "fluent-arg")
	{
            std::string term_name = termNode->getText();

            if (bFirstArg)
	    {
                bFirstArg = false;
                os << "__";    
	    }
            else
	    {
                os << "_";
	    }

            os << term_name;
	}
        else if (termNode != 0 && termNode->getName() == "fluent-value")
	{
            val = (termNode->getText() == "true");
	}
    }

    std::cout << "-------- collected value : " << val << " " << os.str() << std::endl;

    if (val)
    {
    // using the atom's description, find the atom's identifier
    Predicate p = *planner->get_problem().domain().predicates().find_predicate( os.str() );

    // AA: Peut-etre à mettre avant de chercher le predicate dans la table
    if ( PredicateTable::static_predicate(p) )
        return NULL;

    //  if (p != NULL)
    
        TermList terms;
        const Atom& a = Atom::make( p, terms );
        RCObject::ref(&a);

        Atom::AtomTable::const_iterator af = Atom::atom_table().find( &a );
        if ( af == Atom::atom_table().end())
        {
            std::cerr << " Error, cannot find Atom " <<  os.str() << std::endl;
            exit(-1);
        }

        RCObject::destructive_deref(&a);
         return *af;
    }
    // negative atom
    else return NULL;

}


// std::make_pair(Fluent::fluent(*planner->get_problem().domain().functions().find_function( fluentName ) ), PARAMS ) );


// = Fluent::fluent(*(pb.domain().functions().find_function("reward")));



  // // PROST Parsing
  // name = name.substr(0, name.length() - 1);
  //   // AA.
  //   std::cout << "               " << name << std::endl;

  //   vector<string> params;
  //   string value;
  //   string fluentName;

  //   for (int i = 0; i < node->size(); i++) {
  //       XMLNode const* paramNode = node->getChild(i);
  //       if (!paramNode) {
  //           assert(false);
  //           continue;
  //       } else if (paramNode->getName() == "fluent-arg") {
  //           string param = paramNode->getText();
  //           params.push_back(param.substr(0, param.length() - 1));
  //       } else if (paramNode->getName() == "fluent-value") {
  //           value = paramNode->getText();
  //           value = value.substr(0, value.length() - 1);
  //       }
  //   }
  //   name += "(";
  //   for (unsigned int i = 0; i < params.size(); ++i) {
  //       name += params[i];
  //       if (i != params.size() - 1) {
  //           name += ", ";
  //       }
  //   }
  //   name += ")";
  //   assert(result.find(name) == result.end());
  //   result[name] = value;



// void IPPCClient::stateTovaluemap(nextState, values){

//     for(map<string,string>::iterator it = nextState.begin(); it != nextState.end(); ++it) {
//         string fluentName = it->first;
//         string param = it->second;

//         values.insert(  std::make_pair(Fluent::fluent(*planner->get_problem().domain().functions().find_function( fluentName ) ), PARAMS ) );
//     }

// }

const string IPPCClient::getServerAction(string action_name, std::vector<string>& arguments)
{
    // In the planner's naming convention, an action is characterized by a string 
    // name__arg1_arg2__num, where "name" is the name of th e predicate, and arg1, arg2,
    // etc. are the names of the constants to which the predicate is applied, and
    // "num" is the number of conditional effect translated in action.
    // The server takes name[arg1].
    // The code below takes an XML message containing the name of a predicate and
    // its arguments, and puts them into the above form.

    std::string delimiter = "_";
    size_t pos = 0;

    while ((pos = action_name.find(delimiter)) != std::string::npos) {
        action_name[pos] = '-';
    }

    delimiter = "--";
    pos = 0;     
    // Gets the arguments
      while ((pos =  action_name.find(delimiter)) != std::string::npos) {
          arguments.push_back( action_name.substr(0, pos) );
          action_name.erase(0, pos + delimiter.length());
    }

      action_name = arguments[0];
      arguments.erase( arguments.begin() );
    
    for (int i = 0; i < arguments.size(); ++i)
        std::cout << arguments[i] << ", ";
    std::cout << std::endl;

    return action_name;
}
