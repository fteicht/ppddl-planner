/* -*-C++-*- */
/*
 * XML Clients.
 *
 * Copyright 2003-2005 Carnegie Mellon University and Rutgers University
 * Copyright 2007 H�kan Younes
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef _CLIENT_H
#define _CLIENT_H

#include "problems.h"
#include "formulas.h"
#include "expressions.h"
#include "strxml.h"
#include <string>

class XMLClient;

/* ====================================================================== */
/* Planner */

/*
 * An abstract planner.
 */
struct Planner {
  /* Constructs a planner. */
  Planner(const Problem& problem) : _problem(problem) {}

  /* Deletes this planner. */
  virtual ~Planner() {}

  /* Called to initialize a round. */
  virtual void initRound() = 0;

  /* Called to return an action for the given state. */
  virtual const Action* decideAction(const AtomSet& atoms,
                                     const ValueMap& values) = 0;

  /* Called to finalize a round. */
  virtual void endRound() = 0;

protected:
  /* Problem to solve. */
  const Problem& _problem;
  
  /* XML client */
  const XMLClient* xml_client_;
  friend class XMLClient;
};


/* ====================================================================== */
/* XMLClient */

/*
 * An XML client.
 */
struct XMLClient {
  /* Constructs an XML client. */
  XMLClient(Planner& planner, const Problem& problem, const std::string& name,
            int fd);
  
  static bool sessionRequestInfo(const XMLNode* node,
          int& rounds, long& time, int& turns);
  
  static bool roundRequestInfo(const XMLNode* node,
            int& round_number, long& time_left, int& rounds_left);
  
  static const Atom* getAtom(const Problem& problem, const XMLNode* atomNode);
  static const Fluent* getFluent(const Problem& problem, const XMLNode* appNode);
  
  static bool getState(AtomSet& atoms, ValueMap& values,
                       const Problem& problem, const XMLNode* stateNode);
  
  static void sendAction(std::ostream& os, const Action* action);
  
  int get_total_rounds() const {return total_rounds_;}
  int get_round_turns() const {return round_turns_;}
  long get_round_time() const {return round_time_;}
  
  int get_round_number() const {return round_number_;}
  long get_time_left() const {return time_left_;}
  int get_rounds_left() const {return rounds_left_;}
  
protected :
  int total_rounds_;
  int round_turns_;
  long round_time_;
  
  int round_number_;
  long time_left_;
  int rounds_left_;
};


#endif /* _CLIENT_H */
