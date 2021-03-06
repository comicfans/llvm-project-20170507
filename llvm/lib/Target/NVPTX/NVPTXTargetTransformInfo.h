//===-- NVPTXTargetTransformInfo.h - NVPTX specific TTI ---------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
/// \file
/// This file a TargetTransformInfo::Concept conforming object specific to the
/// NVPTX target machine. It uses the target's detailed information to
/// provide more precise answers to certain TTI queries, while letting the
/// target independent and default TTI implementations handle the rest.
///
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_NVPTX_NVPTXTARGETTRANSFORMINFO_H
#define LLVM_LIB_TARGET_NVPTX_NVPTXTARGETTRANSFORMINFO_H

#include "NVPTX.h"
#include "NVPTXTargetMachine.h"
#include "llvm/Analysis/TargetTransformInfo.h"
#include "llvm/CodeGen/BasicTTIImpl.h"
#include "llvm/Target/TargetLowering.h"

namespace llvm {

class NVPTXTTIImpl : public BasicTTIImplBase<NVPTXTTIImpl> {
  typedef BasicTTIImplBase<NVPTXTTIImpl> BaseT;
  typedef TargetTransformInfo TTI;
  friend BaseT;

  const NVPTXSubtarget *ST;
  const NVPTXTargetLowering *TLI;

  const NVPTXSubtarget *getST() const { return ST; };
  const NVPTXTargetLowering *getTLI() const { return TLI; };

public:
  explicit NVPTXTTIImpl(const NVPTXTargetMachine *TM, const Function &F)
      : BaseT(TM, F.getParent()->getDataLayout()), ST(TM->getSubtargetImpl()),
        TLI(ST->getTargetLowering()) {}

  bool hasBranchDivergence() { return true; }

  bool isSourceOfDivergence(const Value *V);

  unsigned getFlatAddressSpace() const {
    return AddressSpace::ADDRESS_SPACE_GENERIC;
  }

  // Increase the inlining cost threshold by a factor of 5, reflecting that
  // calls are particularly expensive in NVPTX.
  unsigned getInliningThresholdMultiplier() { return 5; }

  int getArithmeticInstrCost(
      unsigned Opcode, Type *Ty,
      TTI::OperandValueKind Opd1Info = TTI::OK_AnyValue,
      TTI::OperandValueKind Opd2Info = TTI::OK_AnyValue,
      TTI::OperandValueProperties Opd1PropInfo = TTI::OP_None,
      TTI::OperandValueProperties Opd2PropInfo = TTI::OP_None,
      ArrayRef<const Value *> Args = ArrayRef<const Value *>());

  void getUnrollingPreferences(Loop *L, ScalarEvolution &SE,
                               TTI::UnrollingPreferences &UP);
  bool hasVolatileVariant(Instruction *I, unsigned AddrSpace) {
    // Volatile loads/stores are only supported for shared and global address
    // spaces, or for generic AS that maps to them.
    if (!(AddrSpace == llvm::ADDRESS_SPACE_GENERIC ||
          AddrSpace == llvm::ADDRESS_SPACE_GLOBAL ||
          AddrSpace == llvm::ADDRESS_SPACE_SHARED))
      return false;

    switch(I->getOpcode()){
    default:
      return false;
    case Instruction::Load:
    case Instruction::Store:
      return true;
    }
  }
};

} // end namespace llvm

#endif
