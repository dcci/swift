//===--- replv2.cpp - Swift REPL v2  --------------------------------------===//
//
// This source file is part of the Swift.org open source project
//
// Copyright (c) 2014 - 2019 Apple Inc. and the Swift project authors
// Licensed under Apache License v2.0 with Runtime Library Exception
//
// See https://swift.org/LICENSE.txt for license information
// See https://swift.org/CONTRIBUTORS.txt for the list of Swift project authors
//
//===----------------------------------------------------------------------===//
// This is a host-side tool to implement the REPL.
//===----------------------------------------------------------------------===//

#include "llvm/ExecutionEngine/Orc/LLJIT.h"
#include "llvm/ExecutionEngine/Orc/JITTargetMachineBuilder.h"
#include "llvm/IRReader/IRReader.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/Error.h"
#include "llvm/Support/InitLLVM.h"
#include "llvm/Support/SourceMgr.h"
#include "llvm/Support/TargetSelect.h"

using namespace llvm;
using namespace llvm::orc;

ExitOnError ExitOnErr;

int main(int argc, char *argv[]) {
  InitLLVM X(argc, argv);

  InitializeNativeTarget();
  InitializeNativeTargetAsmPrinter();

  cl::ParseCommandLineOptions(argc, argv, "REPL");
  ExitOnErr.setBanner(std::string(argv[0]) + ": ");

  auto JTMB = ExitOnErr(JITTargetMachineBuilder::detectHost());
  auto J = ExitOnErr(LLJIT::Create(JTMB,
                                   ExitOnErr(JTMB.getDefaultDataLayoutForTarget())));

  SMDiagnostic Err;
  auto Ctx = llvm::make_unique<LLVMContext>();
  std::string Source = 
    R"(
      define i32 @add1(i32 %x) {
        entry:
          %r = add nsw i32 %x, 1
          ret i32 %r
      }
  )";
  auto M = parseIR(MemoryBufferRef(Source, "foo.ll"), Err, *Ctx);
  if (!M) {
    std::string ErrMsg;
    {
      raw_string_ostream ErrStream(ErrMsg);
      Err.print("", ErrStream);
    }
    llvm::errs() << ErrMsg << "\n";
  }

  // Thread safe initialization.
  ExitOnErr(J->addIRModule(ThreadSafeModule(std::move(M), std::move(Ctx))));
  auto Add1Sym = ExitOnErr(J->lookup("add1"));
  int (*Add1)(int) = (int (*)(int))Add1Sym.getAddress();
  int Result = Add1(42);
  llvm::outs() << Result << "\n";

  return 0;
}
