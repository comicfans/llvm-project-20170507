; NOTE: Assertions have been autogenerated by utils/update_llc_test_checks.py
; RUN: llc < %s -mtriple=i686-unknown-unknown | FileCheck %s -check-prefix=X32
; RUN: llc < %s -mtriple=x86_64-unknown-unknown | FileCheck %s -check-prefix=X64

; It's not necessary to zero-extend the arg because it is specified 'zeroext'. 
define void @bar1(i1 zeroext %v1) nounwind ssp {
; X32-LABEL: bar1:
; X32:       # BB#0:
; X32-NEXT:    movzbl {{[0-9]+}}(%esp), %eax
; X32-NEXT:    pushl %eax
; X32-NEXT:    calll foo1
; X32-NEXT:    addl $4, %esp
; X32-NEXT:    retl
;
; X64-LABEL: bar1:
; X64:       # BB#0:
; X64-NEXT:    xorl %eax, %eax
; X64-NEXT:    jmp foo1 # TAILCALL
  %conv = zext i1 %v1 to i32
  %call = tail call i32 (...) @foo1(i32 %conv) nounwind
  ret void
}

; Check that on x86-64 the arguments are simply forwarded.
define void @bar2(i8 zeroext %v1) nounwind ssp {
; X32-LABEL: bar2:
; X32:       # BB#0:
; X32-NEXT:    movzbl {{[0-9]+}}(%esp), %eax
; X32-NEXT:    pushl %eax
; X32-NEXT:    calll foo1
; X32-NEXT:    addl $4, %esp
; X32-NEXT:    retl
;
; X64-LABEL: bar2:
; X64:       # BB#0:
; X64-NEXT:    xorl %eax, %eax
; X64-NEXT:    jmp foo1 # TAILCALL
  %conv = zext i8 %v1 to i32
  %call = tail call i32 (...) @foo1(i32 %conv) nounwind
  ret void
}

; Check that i1 return values are not zero-extended.
define zeroext i1 @bar3() nounwind ssp {
; X32-LABEL: bar3:
; X32:       # BB#0:
; X32-NEXT:    calll foo2
; X32-NEXT:    retl
;
; X64-LABEL: bar3:
; X64:       # BB#0:
; X64-NEXT:    pushq %rax
; X64-NEXT:    callq foo2
; X64-NEXT:    popq %rcx
; X64-NEXT:    retq
  %call = call i1 @foo2() nounwind
  ret i1 %call
}

declare i32 @foo1(...)
declare zeroext i1 @foo2()

