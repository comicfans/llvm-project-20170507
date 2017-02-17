; NOTE: Assertions have been autogenerated by utils/update_test_checks.py
; RUN: opt < %s -instcombine -S | FileCheck %s
target datalayout = "e-p:64:64:64-i1:8:8-i8:8:8-i16:16:16-i32:32:32-i64:64:64-f32:32:32-f64:64:64-v64:64:64-v128:128:128-a0:0:64-s0:64:64-f80:128:128-n8:16:32:64"

; Instcombine should be able to eliminate all of these ext casts.

declare void @use(i32)

define i64 @test1(i64 %a) {
; CHECK-LABEL: @test1(
; CHECK-NEXT:    [[B:%.*]] = trunc i64 %a to i32
; CHECK-NEXT:    [[C:%.*]] = and i64 %a, 15
; CHECK-NEXT:    call void @use(i32 [[B]])
; CHECK-NEXT:    ret i64 [[C]]
;
  %b = trunc i64 %a to i32
  %c = and i32 %b, 15
  %d = zext i32 %c to i64
  call void @use(i32 %b)
  ret i64 %d
}

define i64 @test2(i64 %a) {
; CHECK-LABEL: @test2(
; CHECK-NEXT:    [[B:%.*]] = trunc i64 %a to i32
; CHECK-NEXT:    [[D1:%.*]] = shl i64 %a, 36
; CHECK-NEXT:    [[D:%.*]] = ashr exact i64 [[D:%.*]]1, 36
; CHECK-NEXT:    call void @use(i32 [[B]])
; CHECK-NEXT:    ret i64 [[D]]
;
  %b = trunc i64 %a to i32
  %c = shl i32 %b, 4
  %q = ashr i32 %c, 4
  %d = sext i32 %q to i64
  call void @use(i32 %b)
  ret i64 %d
}

define i64 @test3(i64 %a) {
; CHECK-LABEL: @test3(
; CHECK-NEXT:    [[B:%.*]] = trunc i64 %a to i32
; CHECK-NEXT:    [[C:%.*]] = and i64 %a, 8
; CHECK-NEXT:    call void @use(i32 [[B]])
; CHECK-NEXT:    ret i64 [[C]]
;
  %b = trunc i64 %a to i32
  %c = and i32 %b, 8
  %d = zext i32 %c to i64
  call void @use(i32 %b)
  ret i64 %d
}

define i64 @test4(i64 %a) {
; CHECK-LABEL: @test4(
; CHECK-NEXT:    [[B:%.*]] = trunc i64 %a to i32
; CHECK-NEXT:    [[C:%.*]] = and i64 %a, 8
; CHECK-NEXT:    [[X:%.*]] = xor i64 [[C]], 8
; CHECK-NEXT:    call void @use(i32 [[B]])
; CHECK-NEXT:    ret i64 [[X]]
;
  %b = trunc i64 %a to i32
  %c = and i32 %b, 8
  %x = xor i32 %c, 8
  %d = zext i32 %x to i64
  call void @use(i32 %b)
  ret i64 %d
}

define i32 @test5(i32 %A) {
; CHECK-LABEL: @test5(
; CHECK-NEXT:    [[C:%.*]] = lshr i32 %A, 16
; CHECK-NEXT:    ret i32 [[C]]
;
  %B = zext i32 %A to i128
  %C = lshr i128 %B, 16
  %D = trunc i128 %C to i32
  ret i32 %D
}

define i32 @test6(i64 %A) {
; CHECK-LABEL: @test6(
; CHECK-NEXT:    [[C:%.*]] = lshr i64 %A, 32
; CHECK-NEXT:    [[D:%.*]] = trunc i64 [[C]] to i32
; CHECK-NEXT:    ret i32 [[D]]
;
  %B = zext i64 %A to i128
  %C = lshr i128 %B, 32
  %D = trunc i128 %C to i32
  ret i32 %D
}

define i92 @test7(i64 %A) {
; CHECK-LABEL: @test7(
; CHECK-NEXT:    [[B:%.*]] = zext i64 %A to i92
; CHECK-NEXT:    [[C:%.*]] = lshr i92 [[B]], 32
; CHECK-NEXT:    ret i92 [[C]]
;
  %B = zext i64 %A to i128
  %C = lshr i128 %B, 32
  %D = trunc i128 %C to i92
  ret i92 %D
}

define i64 @test8(i32 %A, i32 %B) {
; CHECK-LABEL: @test8(
; CHECK-NEXT:    [[TMP38:%.*]] = zext i32 %A to i64
; CHECK-NEXT:    [[TMP32:%.*]] = zext i32 %B to i64
; CHECK-NEXT:    [[TMP33:%.*]] = shl nuw i64 [[TMP32]], 32
; CHECK-NEXT:    [[INS35:%.*]] = or i64 [[TMP33]], [[TMP38]]
; CHECK-NEXT:    ret i64 [[INS35]]
;
  %tmp38 = zext i32 %A to i128
  %tmp32 = zext i32 %B to i128
  %tmp33 = shl i128 %tmp32, 32
  %ins35 = or i128 %tmp33, %tmp38
  %tmp42 = trunc i128 %ins35 to i64
  ret i64 %tmp42
}

define i8 @test9(i32 %X) {
; CHECK-LABEL: @test9(
; CHECK-NEXT:    [[TMP1:%.*]] = trunc i32 %X to i8
; CHECK-NEXT:    [[Z:%.*]] = and i8 [[TMP1]], 42
; CHECK-NEXT:    ret i8 [[Z]]
;
  %Y = and i32 %X, 42
  %Z = trunc i32 %Y to i8
  ret i8 %Z
}

; rdar://8808586
define i8 @test10(i32 %X) {
; CHECK-LABEL: @test10(
; CHECK-NEXT:    [[Y:%.*]] = trunc i32 %X to i8
; CHECK-NEXT:    [[Z:%.*]] = and i8 [[Y]], 42
; CHECK-NEXT:    ret i8 [[Z]]
;
  %Y = trunc i32 %X to i8
  %Z = and i8 %Y, 42
  ret i8 %Z
}

; PR25543
; https://llvm.org/bugs/show_bug.cgi?id=25543
; This is an extractelement.

define i32 @trunc_bitcast1(<4 x i32> %v) {
; CHECK-LABEL: @trunc_bitcast1(
; CHECK-NEXT:    [[EXT:%.*]] = extractelement <4 x i32> %v, i32 1
; CHECK-NEXT:    ret i32 [[EXT]]
;
  %bc = bitcast <4 x i32> %v to i128
  %shr = lshr i128 %bc, 32
  %ext = trunc i128 %shr to i32
  ret i32 %ext
}

; A bitcast may still be required.

define i32 @trunc_bitcast2(<2 x i64> %v) {
; CHECK-LABEL: @trunc_bitcast2(
; CHECK-NEXT:    [[BC1:%.*]] = bitcast <2 x i64> %v to <4 x i32>
; CHECK-NEXT:    [[EXT:%.*]] = extractelement <4 x i32> [[BC1]], i32 2
; CHECK-NEXT:    ret i32 [[EXT]]
;
  %bc = bitcast <2 x i64> %v to i128
  %shr = lshr i128 %bc, 64
  %ext = trunc i128 %shr to i32
  ret i32 %ext
}

; The right shift is optional.

define i32 @trunc_bitcast3(<4 x i32> %v) {
; CHECK-LABEL: @trunc_bitcast3(
; CHECK-NEXT:    [[EXT:%.*]] = extractelement <4 x i32> %v, i32 0
; CHECK-NEXT:    ret i32 [[EXT]]
;
  %bc = bitcast <4 x i32> %v to i128
  %ext = trunc i128 %bc to i32
  ret i32 %ext
}

define i32 @trunc_shl_31_i32_i64(i64 %val) {
; CHECK-LABEL: @trunc_shl_31_i32_i64(
; CHECK-NEXT:    [[VAL_TR:%.*]] = trunc i64 %val to i32
; CHECK-NEXT:    [[TRUNC:%.*]] = shl i32 [[VAL_TR]], 31
; CHECK-NEXT:    ret i32 [[TRUNC]]
;
  %shl = shl i64 %val, 31
  %trunc = trunc i64 %shl to i32
  ret i32 %trunc
}

define i32 @trunc_shl_nsw_31_i32_i64(i64 %val) {
; CHECK-LABEL: @trunc_shl_nsw_31_i32_i64(
; CHECK-NEXT:    [[VAL_TR:%.*]] = trunc i64 %val to i32
; CHECK-NEXT:    [[TRUNC:%.*]] = shl i32 [[VAL_TR]], 31
; CHECK-NEXT:    ret i32 [[TRUNC]]
;
  %shl = shl nsw i64 %val, 31
  %trunc = trunc i64 %shl to i32
  ret i32 %trunc
}

define i32 @trunc_shl_nuw_31_i32_i64(i64 %val) {
; CHECK-LABEL: @trunc_shl_nuw_31_i32_i64(
; CHECK-NEXT:    [[VAL_TR:%.*]] = trunc i64 %val to i32
; CHECK-NEXT:    [[TRUNC:%.*]] = shl i32 [[VAL_TR]], 31
; CHECK-NEXT:    ret i32 [[TRUNC]]
;
  %shl = shl nuw i64 %val, 31
  %trunc = trunc i64 %shl to i32
  ret i32 %trunc
}

define i32 @trunc_shl_nsw_nuw_31_i32_i64(i64 %val) {
; CHECK-LABEL: @trunc_shl_nsw_nuw_31_i32_i64(
; CHECK-NEXT:    [[VAL_TR:%.*]] = trunc i64 %val to i32
; CHECK-NEXT:    [[TRUNC:%.*]] = shl i32 [[VAL_TR]], 31
; CHECK-NEXT:    ret i32 [[TRUNC]]
;
  %shl = shl nsw nuw i64 %val, 31
  %trunc = trunc i64 %shl to i32
  ret i32 %trunc
}

define i16 @trunc_shl_15_i16_i64(i64 %val) {
; CHECK-LABEL: @trunc_shl_15_i16_i64(
; CHECK-NEXT:    [[VAL_TR:%.*]] = trunc i64 %val to i16
; CHECK-NEXT:    [[TRUNC:%.*]] = shl i16 [[VAL_TR]], 15
; CHECK-NEXT:    ret i16 [[TRUNC]]
;
  %shl = shl i64 %val, 15
  %trunc = trunc i64 %shl to i16
  ret i16 %trunc
}

define i16 @trunc_shl_15_i16_i32(i32 %val) {
; CHECK-LABEL: @trunc_shl_15_i16_i32(
; CHECK-NEXT:    [[VAL_TR:%.*]] = trunc i32 %val to i16
; CHECK-NEXT:    [[TRUNC:%.*]] = shl i16 [[VAL_TR]], 15
; CHECK-NEXT:    ret i16 [[TRUNC]]
;
  %shl = shl i32 %val, 15
  %trunc = trunc i32 %shl to i16
  ret i16 %trunc
}

define i8 @trunc_shl_7_i8_i64(i64 %val) {
; CHECK-LABEL: @trunc_shl_7_i8_i64(
; CHECK-NEXT:    [[VAL_TR:%.*]] = trunc i64 %val to i8
; CHECK-NEXT:    [[TRUNC:%.*]] = shl i8 [[VAL_TR]], 7
; CHECK-NEXT:    ret i8 [[TRUNC]]
;
  %shl = shl i64 %val, 7
  %trunc = trunc i64 %shl to i8
  ret i8 %trunc
}

define i2 @trunc_shl_1_i2_i64(i64 %val) {
; CHECK-LABEL: @trunc_shl_1_i2_i64(
; CHECK-NEXT:    [[SHL:%.*]] = shl i64 %val, 1
; CHECK-NEXT:    [[TRUNC:%.*]] = trunc i64 [[SHL]] to i2
; CHECK-NEXT:    ret i2 [[TRUNC]]
;
  %shl = shl i64 %val, 1
  %trunc = trunc i64 %shl to i2
  ret i2 %trunc
}

define i32 @trunc_shl_1_i32_i64(i64 %val) {
; CHECK-LABEL: @trunc_shl_1_i32_i64(
; CHECK-NEXT:    [[VAL_TR:%.*]] = trunc i64 %val to i32
; CHECK-NEXT:    [[TRUNC:%.*]] = shl i32 [[VAL_TR]], 1
; CHECK-NEXT:    ret i32 [[TRUNC]]
;
  %shl = shl i64 %val, 1
  %trunc = trunc i64 %shl to i32
  ret i32 %trunc
}

define i32 @trunc_shl_16_i32_i64(i64 %val) {
; CHECK-LABEL: @trunc_shl_16_i32_i64(
; CHECK-NEXT:    [[VAL_TR:%.*]] = trunc i64 %val to i32
; CHECK-NEXT:    [[TRUNC:%.*]] = shl i32 [[VAL_TR]], 16
; CHECK-NEXT:    ret i32 [[TRUNC]]
;
  %shl = shl i64 %val, 16
  %trunc = trunc i64 %shl to i32
  ret i32 %trunc
}

define i32 @trunc_shl_33_i32_i64(i64 %val) {
; CHECK-LABEL: @trunc_shl_33_i32_i64(
; CHECK-NEXT:    ret i32 0
;
  %shl = shl i64 %val, 33
  %trunc = trunc i64 %shl to i32
  ret i32 %trunc
}

define i32 @trunc_shl_32_i32_i64(i64 %val) {
; CHECK-LABEL: @trunc_shl_32_i32_i64(
; CHECK-NEXT:    ret i32 0
;
  %shl = shl i64 %val, 32
  %trunc = trunc i64 %shl to i32
  ret i32 %trunc
}

; TODO: Should be able to handle vectors
define <2 x i32> @trunc_shl_16_v2i32_v2i64(<2 x i64> %val) {
; CHECK-LABEL: @trunc_shl_16_v2i32_v2i64(
; CHECK-NEXT:    [[SHL:%.*]] = shl <2 x i64> %val, <i64 16, i64 16>
; CHECK-NEXT:    [[TRUNC:%.*]] = trunc <2 x i64> [[SHL]] to <2 x i32>
; CHECK-NEXT:    ret <2 x i32> [[TRUNC]]
;
  %shl = shl <2 x i64> %val, <i64 16, i64 16>
  %trunc = trunc <2 x i64> %shl to <2 x i32>
  ret <2 x i32> %trunc
}

define <2 x i32> @trunc_shl_nosplat_v2i32_v2i64(<2 x i64> %val) {
; CHECK-LABEL: @trunc_shl_nosplat_v2i32_v2i64(
; CHECK-NEXT:    [[SHL:%.*]] = shl <2 x i64> %val, <i64 15, i64 16>
; CHECK-NEXT:    [[TRUNC:%.*]] = trunc <2 x i64> [[SHL]] to <2 x i32>
; CHECK-NEXT:    ret <2 x i32> [[TRUNC]]
;
  %shl = shl <2 x i64> %val, <i64 15, i64 16>
  %trunc = trunc <2 x i64> %shl to <2 x i32>
  ret <2 x i32> %trunc
}

define void @trunc_shl_31_i32_i64_multi_use(i64 %val, i32 addrspace(1)* %ptr0, i64 addrspace(1)* %ptr1) {
; CHECK-LABEL: @trunc_shl_31_i32_i64_multi_use(
; CHECK-NEXT:    [[SHL:%.*]] = shl i64 %val, 31
; CHECK-NEXT:    [[TRUNC:%.*]] = trunc i64 [[SHL]] to i32
; CHECK-NEXT:    store volatile i32 [[TRUNC]], i32 addrspace(1)* %ptr0, align 4
; CHECK-NEXT:    store volatile i64 [[SHL]], i64 addrspace(1)* %ptr1, align 8
; CHECK-NEXT:    ret void
;
  %shl = shl i64 %val, 31
  %trunc = trunc i64 %shl to i32
  store volatile i32 %trunc, i32 addrspace(1)* %ptr0
  store volatile i64 %shl, i64 addrspace(1)* %ptr1
  ret void
}

define i32 @trunc_shl_lshr_infloop(i64 %arg) {
; CHECK-LABEL: @trunc_shl_lshr_infloop(
; CHECK-NEXT:    [[TMP0:%.*]] = lshr i64 %arg, 1
; CHECK-NEXT:    [[TMP1:%.*]] = shl i64 [[TMP0]], 2
; CHECK-NEXT:    [[TMP2:%.*]] = trunc i64 [[TMP1]] to i32
; CHECK-NEXT:    ret i32 [[TMP2]]
;
  %tmp0 = lshr i64 %arg, 1
  %tmp1 = shl i64 %tmp0, 2
  %tmp2 = trunc i64 %tmp1 to i32
  ret i32 %tmp2
}

define i32 @trunc_shl_ashr_infloop(i64 %arg) {
; CHECK-LABEL: @trunc_shl_ashr_infloop(
; CHECK-NEXT:    [[TMP0:%.*]] = ashr i64 %arg, 3
; CHECK-NEXT:    [[TMP1:%.*]] = shl nsw i64 [[TMP0]], 2
; CHECK-NEXT:    [[TMP2:%.*]] = trunc i64 [[TMP1]] to i32
; CHECK-NEXT:    ret i32 [[TMP2]]
;
  %tmp0 = ashr i64 %arg, 3
  %tmp1 = shl i64 %tmp0, 2
  %tmp2 = trunc i64 %tmp1 to i32
  ret i32 %tmp2
}

define i32 @trunc_shl_shl_infloop(i64 %arg) {
; CHECK-LABEL: @trunc_shl_shl_infloop(
; CHECK-NEXT:    [[ARG_TR:%.*]] = trunc i64 %arg to i32
; CHECK-NEXT:    [[TMP2:%.*]] = shl i32 [[ARG_TR]], 3
; CHECK-NEXT:    ret i32 [[TMP2]]
;
  %tmp0 = shl i64 %arg, 1
  %tmp1 = shl i64 %tmp0, 2
  %tmp2 = trunc i64 %tmp1 to i32
  ret i32 %tmp2
}

define i32 @trunc_shl_lshr_var(i64 %arg, i64 %val) {
; CHECK-LABEL: @trunc_shl_lshr_var(
; CHECK-NEXT:    [[TMP0:%.*]] = lshr i64 %arg, %val
; CHECK-NEXT:    [[TMP0_TR:%.*]] = trunc i64 [[TMP0]] to i32
; CHECK-NEXT:    [[TMP2:%.*]] = shl i32 [[TMP0_TR]], 2
; CHECK-NEXT:    ret i32 [[TMP2]]
;
  %tmp0 = lshr i64 %arg, %val
  %tmp1 = shl i64 %tmp0, 2
  %tmp2 = trunc i64 %tmp1 to i32
  ret i32 %tmp2
}

define i32 @trunc_shl_ashr_var(i64 %arg, i64 %val) {
; CHECK-LABEL: @trunc_shl_ashr_var(
; CHECK-NEXT:    [[TMP0:%.*]] = ashr i64 %arg, %val
; CHECK-NEXT:    [[TMP0_TR:%.*]] = trunc i64 [[TMP0]] to i32
; CHECK-NEXT:    [[TMP2:%.*]] = shl i32 [[TMP0_TR]], 2
; CHECK-NEXT:    ret i32 [[TMP2]]
;
  %tmp0 = ashr i64 %arg, %val
  %tmp1 = shl i64 %tmp0, 2
  %tmp2 = trunc i64 %tmp1 to i32
  ret i32 %tmp2
}

define i32 @trunc_shl_shl_var(i64 %arg, i64 %val) {
; CHECK-LABEL: @trunc_shl_shl_var(
; CHECK-NEXT:    [[TMP0:%.*]] = shl i64 %arg, %val
; CHECK-NEXT:    [[TMP0_TR:%.*]] = trunc i64 [[TMP0]] to i32
; CHECK-NEXT:    [[TMP2:%.*]] = shl i32 [[TMP0_TR]], 2
; CHECK-NEXT:    ret i32 [[TMP2]]
;
  %tmp0 = shl i64 %arg, %val
  %tmp1 = shl i64 %tmp0, 2
  %tmp2 = trunc i64 %tmp1 to i32
  ret i32 %tmp2
}

define <8 x i16> @trunc_shl_v8i15_v8i32_15(<8 x i32> %a) {
; CHECK-LABEL: @trunc_shl_v8i15_v8i32_15(
; CHECK-NEXT:    [[SHL:%.*]] = shl <8 x i32> %a, <i32 15, i32 15, i32 15, i32 15, i32 15, i32 15, i32 15, i32 15>
; CHECK-NEXT:    [[CONV:%.*]] = trunc <8 x i32> [[SHL]] to <8 x i16>
; CHECK-NEXT:    ret <8 x i16> [[CONV]]
;
  %shl = shl <8 x i32> %a, <i32 15, i32 15, i32 15, i32 15, i32 15, i32 15, i32 15, i32 15>
  %conv = trunc <8 x i32> %shl to <8 x i16>
  ret <8 x i16> %conv
}

define <8 x i16> @trunc_shl_v8i16_v8i32_16(<8 x i32> %a) {
; CHECK-LABEL: @trunc_shl_v8i16_v8i32_16(
; CHECK-NEXT:    ret <8 x i16> zeroinitializer
;
  %shl = shl <8 x i32> %a, <i32 16, i32 16, i32 16, i32 16, i32 16, i32 16, i32 16, i32 16>
  %conv = trunc <8 x i32> %shl to <8 x i16>
  ret <8 x i16> %conv
}

define <8 x i16> @trunc_shl_v8i16_v8i32_17(<8 x i32> %a) {
; CHECK-LABEL: @trunc_shl_v8i16_v8i32_17(
; CHECK-NEXT:    ret <8 x i16> zeroinitializer
;
  %shl = shl <8 x i32> %a, <i32 17, i32 17, i32 17, i32 17, i32 17, i32 17, i32 17, i32 17>
  %conv = trunc <8 x i32> %shl to <8 x i16>
  ret <8 x i16> %conv
}

define <8 x i16> @trunc_shl_v8i16_v8i32_4(<8 x i32> %a) {
; CHECK-LABEL: @trunc_shl_v8i16_v8i32_4(
; CHECK-NEXT:    [[SHL:%.*]] = shl <8 x i32> %a, <i32 4, i32 4, i32 4, i32 4, i32 4, i32 4, i32 4, i32 4>
; CHECK-NEXT:    [[CONV:%.*]] = trunc <8 x i32> [[SHL]] to <8 x i16>
; CHECK-NEXT:    ret <8 x i16> [[CONV]]
;
  %shl = shl <8 x i32> %a, <i32 4, i32 4, i32 4, i32 4, i32 4, i32 4, i32 4, i32 4>
  %conv = trunc <8 x i32> %shl to <8 x i16>
  ret <8 x i16> %conv
}

; FIXME: trunc (shuffle X, C, Mask) --> shuffle (trunc X), C', Mask

define <4 x i8> @shuf1(<4 x i32> %x) {
; CHECK-LABEL: @shuf1(
; CHECK-NEXT:    [[SHUF:%.*]] = shufflevector <4 x i32> %x, <4 x i32> <i32 undef, i32 3634, i32 90, i32 undef>, <4 x i32> <i32 1, i32 5, i32 6, i32 2>
; CHECK-NEXT:    [[TRUNC:%.*]] = trunc <4 x i32> [[SHUF]] to <4 x i8>
; CHECK-NEXT:    ret <4 x i8> [[TRUNC]]
;
  %shuf = shufflevector <4 x i32> %x, <4 x i32> <i32 35, i32 3634, i32 90, i32 -1>, <4 x i32> <i32 1, i32 5, i32 6, i32 2>
  %trunc = trunc <4 x i32> %shuf to <4 x i8>
  ret <4 x i8> %trunc
}

; TODO: Shuffle with constant operand should be canonicalized to operand 1?
; FIXME: trunc (shuffle C, X, Mask) --> shuffle C', (trunc X), Mask

define <4 x i8> @shuf2(<4 x i32> %x) {
; CHECK-LABEL: @shuf2(
; CHECK-NEXT:    [[SHUF:%.*]] = shufflevector <4 x i32> <i32 -3500, i32 undef, i32 undef, i32 -1>, <4 x i32> %x, <4 x i32> <i32 3, i32 6, i32 6, i32 0>
; CHECK-NEXT:    [[TRUNC:%.*]] = trunc <4 x i32> [[SHUF]] to <4 x i8>
; CHECK-NEXT:    ret <4 x i8> [[TRUNC]]
;
  %shuf = shufflevector <4 x i32> <i32 -3500, i32 3634, i32 90, i32 -1>, <4 x i32> %x, <4 x i32> <i32 3, i32 6, i32 6, i32 0>
  %trunc = trunc <4 x i32> %shuf to <4 x i8>
  ret <4 x i8> %trunc
}

