//===- Steensgaard.cpp - Context Insensitive Alias Analysis ---------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file was developed by the LLVM research group and is distributed under
// the University of Illinois Open Source License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This pass uses the data structure graphs to implement a simple context
// insensitive alias analysis.  It does this by computing the local analysis
// graphs for all of the functions, then merging them together into a single big
// graph without cloning.
//
//===----------------------------------------------------------------------===//

#include "dsa/DataStructure.h"
#include "dsa/DSGraph.h"
#include "llvm/Analysis/AliasAnalysis.h"
#include "llvm/Analysis/Passes.h"
#include "llvm/Analysis/TargetLibraryInfo.h"
#include "llvm/IR/Module.h"
#include "llvm/Support/Debug.h"
#include <ostream>
using namespace llvm;

namespace {
class SteensAAResult : public AAResultBase<SteensAAResult> {
  DSGraph * ResultGraph;
  SteensAAResult *Result;
public:
  static char ID;

  SteensAAResult(const TargetLibraryInfo &TLI, DSGraph *ResultGraph = nullptr)
      : AAResultBase(TLI) {}

  SteensAAResult(SteensAAResult &&Arg) 
      : AAResultBase(std::move(Arg)), ResultGraph(Arg.ResultGraph) {}
  ~SteensAAResult() {}

  //------------------------------------------------
  // Implement the AliasAnalysis API
  //
  AliasResult alias(const MemoryLocation &LocA, const MemoryLocation &LocB);

  ModRefInfo getModRefInfo(ImmutableCallSite CS, const MemoryLocation &Loc);
};

class SteensAAWrapperPass : public ModulePass {
  std::unique_ptr<SteensAAResult> Result;

public:
  static char ID;

  SteensAAWrapperPass() : ModulePass(ID) {}

  SteensAAResult &getResult() { return *Result; }
  const SteensAAResult &getResult() const { return *Result; }

  bool runOnModule(Module &M) override;
  bool doFinalization(Module &M) override;

  void getAnalysisUsage(AnalysisUsage &AU) const override;

};

AliasResult SteensAAResult::alias(const MemoryLocation &LocA, const MemoryLocation &LocB) {
  const Value *V1 = LocA.Ptr;
  uint64_t V1Size = LocA.Size;
  const Value *V2 = LocB.Ptr;
  uint64_t V2Size = LocB.Size;

  assert(ResultGraph && "Result graph has not been computed yet!");

  DSGraph::ScalarMapTy &GSM = ResultGraph->getScalarMap();

  DSGraph::ScalarMapTy::iterator I = GSM.find(const_cast<Value*>(V1));
  DSGraph::ScalarMapTy::iterator J = GSM.find(const_cast<Value*>(V2));
  if (I != GSM.end() && !I->second.isNull() &&
      J != GSM.end() && !J->second.isNull()) {
    DSNodeHandle &V1H = I->second;
    DSNodeHandle &V2H = J->second;

    // If at least one of the nodes is complete, we can say something about
    // this.  If one is complete and the other isn't, then they are obviously
    // different nodes.  If they are both complete, we can't say anything
    // useful.
    if (I->second.getNode()->isCompleteNode() ||
        J->second.getNode()->isCompleteNode()) {
      // If the two pointers point to different data structure graph nodes, they
      // cannot alias!
      if (V1H.getNode() != V2H.getNode())
        return NoAlias;

      // See if they point to different offsets...  if so, we may be able to
      // determine that they do not alias...
      if (V1Size != MemoryLocation::UnknownSize && V2Size != MemoryLocation::UnknownSize) {
        unsigned O1 = I->second.getOffset(), O2 = J->second.getOffset();
        if (O1 != O2) {
          if (O2 < O1) {    // Ensure that O1 <= O2
            std::swap(V1, V2);
            std::swap(O1, O2);
            std::swap(V1Size, V2Size);
          }

          if (O1+V1Size <= O2)
            return NoAlias;
        }
      }
    }
  }

  // If we cannot determine alias properties based on our graph, fall back on
  // some other AA implementation.
  //
  return AAResultBase::alias(LocA, LocB);
}

ModRefInfo
SteensAAResult::getModRefInfo(ImmutableCallSite CS, const MemoryLocation &Loc) {
  const Value *P = Loc.Ptr;
  uint64_t Size = Loc.Size;
  ModRefInfo Result = MRI_ModRef;

  // Find the node in question.
  DSGraph::ScalarMapTy &GSM = ResultGraph->getScalarMap();
  DSGraph::ScalarMapTy::iterator I = GSM.find(P);

  if (I != GSM.end() && !I->second.isNull()) {
    DSNode *N = I->second.getNode();
    if (N->isCompleteNode()) {
      // If this is a direct call to an external function, and if the pointer
      // points to a complete node, the external function cannot modify or read
      // the value (we know it's not passed out of the program!).
      if (const Function *F = CS.getCalledFunction())
        if (F->isDeclaration())
          return MRI_NoModRef;

      // Otherwise, if the node is complete, but it is only M or R, return this.
      // This can be useful for globals that should be marked const but are not.
      if (!N->isModifiedNode())
        Result = (ModRefInfo)(Result & ~MRI_Mod);
      if (!N->isReadNode())
        Result = (ModRefInfo)(Result & ~MRI_Ref);
    }
  }

  return (ModRefInfo)(Result & getModRefInfo(CS, MemoryLocation(P, Size)));
}

void SteensAAWrapperPass::getAnalysisUsage(AnalysisUsage &AU) const {
  AU.setPreservesAll();                    // Does not transform code...
  AU.addRequired<AAResultsWrapperPass>();   
  AU.addRequired<SteensgaardDataStructures>();   // Uses steensgaard dsgraph
  AU.addRequired<TargetLibraryInfoWrapperPass>();
}

char SteensAAWrapperPass::ID = 0;
RegisterPass<SteensAAWrapperPass> X("steens-aa",
                                    "Steensgaard's alias analysis (DSGraph based)");
}

bool SteensAAWrapperPass::runOnModule(Module &M) {
  Result.reset(
      new SteensAAResult(getAnalysis<TargetLibraryInfoWrapperPass>().getTLI(), 
      getAnalysis<SteensgaardDataStructures>().getResultGraph()));
  return false;
}

bool SteensAAWrapperPass::doFinalization(Module &M) {
  Result.reset();
  return false;
}
