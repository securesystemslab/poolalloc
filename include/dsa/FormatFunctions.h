//===- FormatFunctions.h - Create a list of format functions ----*- C++ -*-===//

#ifndef LLVM_ANALYSIS_FORMAT_FUNCTIONS_H
#define LLVM_ANALYSIS_FORMAT_FUNCTIONS_H

#include "llvm/Pass.h"
#include "llvm/IR/Value.h"
#include "llvm/IR/Function.h"
#include "llvm/ADT/DenseSet.h"

namespace llvm {

class FormatFunctions : public ModulePass {
  DenseSet<StringRef> FormatFunctionsSet;
  
public:
  static char ID;
  FormatFunctions() : ModulePass(ID) {};
  
  virtual void getAnalysisUsage(AnalysisUsage &AU) const override {
    AU.setPreservesAll();
  }

  virtual bool runOnModule(Module &M) override;

  bool isFormatFunction(const Function *F) const {
    const StringRef Name = Function::getRealLinkageName(F->getName());
    return  FormatFunctionsSet.count(Name);
  }

  bool isFormatFunction(const Value *V) const {
    if (const Function *F = dyn_cast<Function>(V->stripPointerCasts())) {
      return isFormatFunction(F);
    }
    return false;
  }
};

}

#endif /* LLVM_ANALYSIS_FORMAT_FUNCTIONS_H */
