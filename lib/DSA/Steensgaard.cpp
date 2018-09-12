//===- Steensgaard.cpp - Context Insensitive Data Structure Analysis ------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file was developed by the LLVM research group and is distributed under
// the University of Illinois Open Source License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This pass computes a context-insensitive data analysis graph.  It does this
// by computing the local analysis graphs for all of the functions, then merging
// them together into a single big graph without cloning.
//
//===----------------------------------------------------------------------===//

#define DEBUG_TYPE "steens"

#include "dsa/DataStructure.h"
#include "dsa/DSGraph.h"
#include "llvm/Analysis/AliasAnalysis.h"
#include "llvm/Analysis/Passes.h"
#include "llvm/IR/Module.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/FormattedStream.h"
#include <ostream>

using namespace llvm;

SteensgaardDataStructures::~SteensgaardDataStructures() { }

void
SteensgaardDataStructures::releaseMemory() {
  delete ResultGraph; 
  ResultGraph = 0;
  DataStructures::releaseMemory();
}

// print - Implement the Pass::print method...
void
SteensgaardDataStructures::print(llvm::raw_ostream &O, const Module *M) const {
  assert(ResultGraph && "Result graph has not yet been computed!");
  ResultGraph->writeGraphToFile(O, "steensgaards");
}

/// run - Build up the result graph, representing the pointer graph for the
/// program.
///
bool
SteensgaardDataStructures::runOnModule(Module &M) {
  DS = &getAnalysis<StdLibDataStructures>();
  init(&M.getDataLayout());
  return runOnModuleInternal(M);
}

bool
SteensgaardDataStructures::runOnModuleInternal(Module &M) {
  assert(ResultGraph == 0 && "Result graph already allocated!");
  
  // Get a copy for the globals graph.
  DSGraph * GG = DS->getGlobalsGraph();
  GlobalsGraph = new DSGraph(GG, GG->getGlobalECs(), *TypeSS);

  // Create a new, empty, graph...
  ResultGraph = new DSGraph(GG->getGlobalECs(), getDataLayout(), *TypeSS);
  ResultGraph->setGlobalsGraph(GlobalsGraph);
  // ResultGraph->spliceFrom(DS->getGlobalsGraph());

  
  // Loop over the rest of the module, merging graphs for non-external functions
  // into this graph.
  //
  for (Module::iterator I = M.begin(), E = M.end(); I != E; ++I) {
    if (!I->isDeclaration()) {
      ResultGraph->spliceFrom(DS->getDSGraph(*I));
    }
  }

  ResultGraph->removeTriviallyDeadNodes();
  ResultGraph->maskIncompleteMarkers();
  ResultGraph->markIncompleteNodes(DSGraph::MarkFormalArgs | DSGraph::IgnoreGlobals);

  // Now that we have all of the graphs inlined, we can go about eliminating
  // call nodes...
  //

  // Start with a copy of the original call sites.
  std::list<DSCallSite> & Calls = ResultGraph->getFunctionCalls();

  for (std::list<DSCallSite>::iterator CI = Calls.begin(), E = Calls.end();
       CI != E;) {
    DSCallSite &CurCall = *CI++;

    // Loop over the called functions, eliminating as many as possible...
    std::vector<const Function*> CallTargets;
    if (CurCall.isDirectCall())
      CallTargets.push_back(CurCall.getCalleeFunc());
    else
      CurCall.getCalleeNode()->addFullFunctionList(CallTargets);

    for (unsigned c = 0; c != CallTargets.size(); ) {
      // If we can eliminate this function call, do so!
      const Function *F = CallTargets[c];
      if (!F->isDeclaration()) {
        ResolveFunctionCall(F, CurCall, ResultGraph->getReturnNodes()[F]);
        CallTargets[c] = CallTargets.back();
        CallTargets.pop_back();
      } else if (Function::getRealLinkageName(F->getName()) == "qsort" &&
                 CurCall.getNumPtrArgs() == 4) {
        // Handle qsort specially
        // TODO: generalize this for other library functions
        assert(CurCall.getNumPtrArgs() == 4 && "Unexpected arguments to qsort");
        DSNodeHandle &Base = CurCall.getPtrArg(0);
        DSNodeHandle &Fun = CurCall.getPtrArg(3);
        std::vector<const Function*> CompareFuns;
        Fun.getNode()->addFullFunctionList(CompareFuns);
        for (const Function *F : CompareFuns) {
          for (const Argument &A : F->args()) {
            ResultGraph->getNodeForValue(&A).mergeWith(Base);
          }
        }
        ++c;                    // Still can't eliminate this call
      } else if (Function::getRealLinkageName(F->getName()) == "pthread_create" &&
                 CurCall.getNumPtrArgs() == 4) {
        // Handle pthread_create specially
        DSNodeHandle &Data = CurCall.getPtrArg(3);
        DSNodeHandle &Fun = CurCall.getPtrArg(2);
        std::vector<const Function*> CompareFuns;
        Fun.getNode()->addFullFunctionList(CompareFuns);
        for (const Function *F : CompareFuns) {
          for (const Argument &A : F->args()) {
            ResultGraph->getNodeForValue(&A).mergeWith(Data);
          }
        }
        ++c;                    // Still can't eliminate this call
      } else
        ++c;  // Cannot eliminate this call, skip over it...
    }

    if (CallTargets.empty()) {        // Eliminated all calls?
      std::list<DSCallSite>::iterator I = CI;
      Calls.erase(--I);               // Remove entry
    }
  }

  // Remove our knowledge of what the return values of the functions are, except
  // for functions that are externally visible from this module (e.g. main).  We
  // keep these functions so that their arguments are marked incomplete.
  for (DSGraph::ReturnNodesTy::iterator I =
         ResultGraph->getReturnNodes().begin(),
         E = ResultGraph->getReturnNodes().end(); I != E; )
    if (I->first->hasInternalLinkage())
      ResultGraph->getReturnNodes().erase(I++);
    else
      ++I;

  // Update the "incomplete" markers on the nodes, ignoring unknownness due to
  // incoming arguments...
  ResultGraph->maskIncompleteMarkers();
  ResultGraph->markIncompleteNodes(DSGraph::MarkFormalArgs | DSGraph::IgnoreGlobals);

  // Remove any nodes that are dead after all of the merging we have done...

  ResultGraph->removeDeadNodes(DSGraph::KeepUnreachableGlobals);

  GlobalsGraph->removeTriviallyDeadNodes();
  GlobalsGraph->maskIncompleteMarkers();

  // Mark external globals incomplete.
  GlobalsGraph->markIncompleteNodes(DSGraph::IgnoreGlobals);

  formGlobalECs();

  // Clone the global nodes into this graph.
  cloneGlobalsInto(ResultGraph, DSGraph::DontCloneCallNodes |
                              DSGraph::DontCloneAuxCallNodes);

  DEBUG(ResultGraph->AssertGraphOK());
  DEBUG(GlobalsGraph->AssertGraphOK());
  return false;
}

/// ResolveFunctionCall - Resolve the actual arguments of a call to function F
/// with the specified call site descriptor.  This function links the arguments
/// and the return value for the call site context-insensitively.
///
void
SteensgaardDataStructures::ResolveFunctionCall(const Function *F, 
                                                const DSCallSite &Call,
                                                DSNodeHandle &RetVal) {

  assert(ResultGraph != 0 && "Result graph not allocated!");

  std::vector<DSNodeHandle> Args;
  ResultGraph->getFunctionArgumentsForCall(F, Args);

  // Handle the return value and VA args of the function.
  Call.getRetVal().mergeWith(Args[0]);
  Call.getVAVal().mergeWith(Args[1]);

  // Loop over all arguments, resolving them to their provided pointers
  for (unsigned i = 0; i < Call.getNumPtrArgs() && (i+2) < Args.size(); i++) {
    Call.getPtrArg(i).mergeWith(Args[i+2]);
  }
}

char SteensgaardDataStructures::ID = 0;

// Register the pass...
static RegisterPass<SteensgaardDataStructures> X
("dsa-steens",
 "Context-insensitive Data Structure Analysis");
