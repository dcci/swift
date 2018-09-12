//===--- SCCP.cpp - Constant fold and diagnose overflows ------------------===//
//
// This source file is part of the Swift.org open source project
//
// Copyright (c) 2014 - 2018 Apple Inc. and the Swift project authors
// Licensed under Apache License v2.0 with Runtime Library Exception
//
// See https://swift.org/LICENSE.txt for license information
// See https://swift.org/CONTRIBUTORS.txt for the list of Swift project authors
//
//===----------------------------------------------------------------------===//

#define DEBUG_TYPE "sccp"
#include "swift/SIL/SILBuilder.h"
#include "swift/SIL/SILVisitor.h"
#include "swift/SILOptimizer/PassManager/Passes.h"
#include "swift/SILOptimizer/PassManager/Transforms.h"
#include "swift/SILOptimizer/Utils/Local.h"
#include "swift/SILOptimizer/Utils/SILOptFunctionBuilder.h"
#include "llvm/ADT/APFloat.h"
#include "llvm/ADT/APSInt.h"

using namespace swift;

namespace {

class LatticeValue {
  enum Type {
    // We haven't looked at this SSA value yet. This is what the original
    // paper calls "undefined", but the term is overloaded enough that we
    // prefer unresolved instead.
    unresolved,

    // The value is believed to be constant at this point.
    constant,

    // The value is known to be non-constant at all program points.
    overdefined
  };

  // The type of this lattice value.
  Type Ty;

  // If the SSA value is constant, the constant.
  llvm::Optional<APInt> ConstantValue;

public:
  LatticeValue() : Ty(Type::unresolved), ConstantValue(llvm::None) {}

  // Accessors to check the state of the lattice value.
  bool isUnresolved() { return Ty == Type::unresolved; }
  bool isConstant() { return Ty == Type::constant; }
  bool isOverdefined() { return Ty == Type::overdefined; }

  APInt getConstant() {
    assert(isConstant() && "the lattice value is not constant");
    return *ConstantValue;
  }

  // Mark this value as constant (according to the lattice meet
  // operator). If we lowered the lattice value, return true, and
  // false otherwise.
  bool markConstant(APInt C) {
    if (isOverdefined())
      return false;
    if (isConstant())
      return C != ConstantValue ? markOverdefined() : false;
    Ty = Type::constant;
    ConstantValue = C;
    return true;
  }

  bool markOverdefined() {
    if (isOverdefined())
      return false;
    Ty = Type::overdefined;
    ConstantValue = llvm::None;
    return true;
  }
};

class SCCPSolver : public SILInstructionVisitor<SCCPSolver, void> {
public:
  SCCPSolver() = default;

  /// Mark the block passed as argument as executable.
  /// Returns true if the block was already marked as reachable,
  /// false otherwise.
  bool markExecutable(SILBasicBlock *BB) {
    if (!ExecutableBBs.insert(BB).second)
      return false;
    BBWorklist.push_back(BB);
    return true;
  }

  /// The main solver.
  void solve();

  void addToWorklist(SILInstruction *I) { InstructionWorklist.push_back(I); }

  // Utilities for dealing with lattice values in the solver.
  void markSingleValueConstant(SingleValueInstruction *I, APInt C) {
    auto Res = I->getResults();
    assert(Res.size() == 1 &&
           "single value instruction produced more than a value");
    SILValue V = Res[0];
    LatticeValue &LV = SSAValueState[V];
    bool Changed = LV.markConstant(C);
    if (Changed)
      addToWorklist(I);
  }

  void markOverdefined(SILInstruction *I) {
    auto Res = I->getResults();
    bool Changed = false;
    for (auto R : Res) {
      LatticeValue &LV = SSAValueState[R];
      Changed |= LV.markOverdefined();
    }
    if (Changed)
      addToWorklist(I);
  }

  // Instruction visitors.
  void visitIntegerLiteralInst(IntegerLiteralInst *ILI) {
    LLVM_DEBUG(llvm::dbgs() << "Visiting integer literal inst: "; ILI->dump());
    markSingleValueConstant(ILI, ILI->getValue());
  }

  void visitBuiltinInst(BuiltinInst *BI) {
    const BuiltinInfo &Builtin = BI->getBuiltinInfo();
    OperandValueArrayRef Args = BI->getArguments();

    // For now we only handle builtins which produce a single value.
    auto Res = BI->getResults();
    if (Res.size() > 1)
      return markOverdefined(BI);

    // At this point we can just assume this is a SingleValueInstruction.
    switch (Builtin.ID) {
    case BuiltinValueKind::URem: {
      auto Num = Args[0];
      auto Denum = Args[1];
      LatticeValue &LV1 = SSAValueState[Num];
      LatticeValue &LV2 = SSAValueState[Denum];
      if (LV1.isConstant() && LV2.isConstant()) {
        APInt LHS = LV1.getConstant();
        APInt RHS = LV2.getConstant();
        APInt Rem = LHS.urem(RHS);
        markSingleValueConstant(BI, Rem);
      }
      break;
    }
    default:
      // We don't know what to do with this intrinsic.
      return markOverdefined(BI);
    }
  }

  void visitSILInstruction(SILInstruction *I) {
    LLVM_DEBUG(llvm::dbgs() << "Visiting instruction: "; I->dump());
    // We don't know what to do with this instruction, conservatively send
    // it to overdefined.
    markOverdefined(I);
    return;
  }

  bool isBlockExecutable(SILBasicBlock *BB) { return ExecutableBBs.count(BB); }

  LatticeValue getLatticeValue(SILValue &V) {
    auto LVI = SSAValueState.find(V);
    assert(LVI != SSAValueState.end() && "missing lattice value");
    return LVI->second;
  }

private:
  // Worklist of SILBasicBlocks that need to be examined.
  SmallVector<SILBasicBlock *, 64> BBWorklist;

  // Map of lattice values associated to instructions.
  llvm::DenseMap<SILValue, LatticeValue> SSAValueState;

  // Worklist of instructions that need to be examined.
  SmallVector<SILInstruction *, 8> InstructionWorklist;

  // Set of blocks that are executable.
  SmallPtrSet<SILBasicBlock *, 8> ExecutableBBs;
};

void SCCPSolver::solve() {
  bool Done = BBWorklist.empty() && InstructionWorklist.empty();
  while (!Done) {
    while (!BBWorklist.empty()) {
      SILBasicBlock *BB = BBWorklist.back();
      BBWorklist.pop_back();
      for (SILInstruction &I : *BB)
        visit(&I);
    }
    while (!InstructionWorklist.empty()) {
      // If the instruction got into this worklist, that's because
      // we lowered the lattice value associated with it either
      // unresolved -> constant or constant -> overdefined.
      // We have to update all the users of the results it produced
      // (if they live in blocks that are marked as executable).
      SILInstruction *I = InstructionWorklist.back();
      InstructionWorklist.pop_back();
      for (auto R : I->getResults())
        for (auto *U : R->getUses())
          if (auto *UI = dyn_cast<SILInstruction>(U->getUser()))
            if (ExecutableBBs.count(UI->getParent()))
              visit(UI);
    }
    Done = BBWorklist.empty() && InstructionWorklist.empty();
  }

#ifndef NDEBUG
  // Dump the lattice state after the algorithm finished.
  for (auto State : SSAValueState) {
    auto Value = State.first;
    auto LV = State.second;
    LLVM_DEBUG(Value->dump());
    if (LV.isUnresolved())
      LLVM_DEBUG(llvm::dbgs() << "unresolved\n");
    else if (LV.isConstant())
      LLVM_DEBUG(llvm::dbgs() << "constant: " << LV.getConstant() << "\n");
    else if (LV.isOverdefined())
      LLVM_DEBUG(llvm::dbgs() << "overdefined\n");
    else
      llvm_unreachable("invalid lattice state!");
  }
#endif
}

class SCCP : public SILFunctionTransform {
public:
  SCCP() = default;

private:
  bool tryToReplaceWithConstant(SCCPSolver &S, SILInstruction &SI,
                                SILValue &V) {
    LatticeValue LV = S.getLatticeValue(V);
    assert(!LV.isUnresolved() && "looking at an unresolved lattice value");
    if (LV.isOverdefined())
      return false;
    assert(LV.isConstant() && "the lattice value must be constant here");
    // FIXME: maybe we should pass a SingleValueInstruction to this function.
    SILBuilderWithScope B(&SI);
    auto *SVI = cast<SingleValueInstruction>(&SI);
    auto C =
        B.createIntegerLiteral(SI.getLoc(), SVI->getType(), LV.getConstant());
    V->replaceAllUsesWith(C);
    return true;
  }

  /// The entry point to the transformation.
  void run() override {
    SILFunction *F = getFunction();
    SCCPSolver S;

    // Mark entry block as reachable.
    S.markExecutable(&F->front());
    S.solve();

    // TODO: remove basic blocks if they're unreachable.
    for (SILBasicBlock &BB : *F) {
      if (!S.isBlockExecutable(&BB))
        continue;
      for (SILBasicBlock::iterator BI = BB.begin(), E = BB.end(); BI != E;) {
        SILInstruction *I = &*BI++;
        auto Res = I->getResults();
        for (auto R : Res)
          tryToReplaceWithConstant(S, *I, R);
        // Throw away dead instructions.
        recursivelyDeleteTriviallyDeadInstructions(I);
      }
    }
  }
};

} // end anonymous namespace

SILTransform *swift::createSCCP() { return new SCCP(); }
