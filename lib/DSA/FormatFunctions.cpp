//===- FormatFunctions.cpp - Create a list of format functions ------------===//

#include "dsa/FormatFunctions.h"

using namespace llvm;

char FormatFunctions::ID = 0;

bool FormatFunctions::runOnModule(Module &M) {
  FormatFunctionsSet.insert("printf");
  FormatFunctionsSet.insert("fprintf");
  FormatFunctionsSet.insert("sprintf");
  FormatFunctionsSet.insert("snprintf");
  FormatFunctionsSet.insert("syslog");
  return false;
}

static RegisterPass<FormatFunctions> X("format-functions", "Create a list of format functions");
