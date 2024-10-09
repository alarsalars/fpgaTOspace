; ModuleID = 'C:/Users/TAlars/Documents/vivado_projects_tests/fpgaTOspace/HLS/wb_s_uart_fd/wb_s_uart_fd/solution1/.autopilot/db/a.g.ld.5.gdce.bc'
source_filename = "llvm-link"
target datalayout = "e-m:e-i64:64-i128:128-i256:256-i512:512-i1024:1024-i2048:2048-i4096:4096-n8:16:32:64-S128-v16:16-v24:32-v32:32-v48:64-v96:128-v192:256-v256:256-v512:512-v1024:1024"
target triple = "fpga64-xilinx-none"

%"struct.ap_uint<8>" = type { %"struct.ap_int_base<8, false>" }
%"struct.ap_int_base<8, false>" = type { %"struct.ssdm_int<8, false>" }
%"struct.ssdm_int<8, false>" = type { i8 }

; Function Attrs: noinline
define void @apatb_top_ir(%"struct.ap_uint<8>"* nocapture readonly %adr, i1 zeroext %we, i1 zeroext %cyc, i1 zeroext %stb, %"struct.ap_uint<8>"* nocapture readonly %wb_in, i1 zeroext %rx, i1* noalias nocapture nonnull dereferenceable(1) %tx, i1* noalias nocapture nonnull dereferenceable(1) %ack, %"struct.ap_uint<8>"* noalias nocapture nonnull dereferenceable(1) %uart_out) local_unnamed_addr #0 {
entry:
  %tx_copy = alloca i1, align 512
  %ack_copy = alloca i1, align 512
  %uart_out_copy = alloca i8, align 512
  call fastcc void @copy_in(i1* nonnull %tx, i1* nonnull align 512 %tx_copy, i1* nonnull %ack, i1* nonnull align 512 %ack_copy, %"struct.ap_uint<8>"* nonnull %uart_out, i8* nonnull align 512 %uart_out_copy)
  call void @apatb_top_hw(%"struct.ap_uint<8>"* %adr, i1 %we, i1 %cyc, i1 %stb, %"struct.ap_uint<8>"* %wb_in, i1 %rx, i1* %tx_copy, i1* %ack_copy, i8* %uart_out_copy)
  call void @copy_back(i1* %tx, i1* %tx_copy, i1* %ack, i1* %ack_copy, %"struct.ap_uint<8>"* %uart_out, i8* %uart_out_copy)
  ret void
}

; Function Attrs: argmemonly noinline norecurse
define internal fastcc void @copy_in(i1* noalias readonly "unpacked"="0", i1* noalias align 512 "unpacked"="1", i1* noalias readonly "unpacked"="2", i1* noalias align 512 "unpacked"="3", %"struct.ap_uint<8>"* noalias readonly "unpacked"="4", i8* noalias nocapture align 512 "unpacked"="5.0.0.0") unnamed_addr #1 {
entry:
  call fastcc void @onebyonecpy_hls.p0i1(i1* align 512 %1, i1* %0)
  call fastcc void @onebyonecpy_hls.p0i1(i1* align 512 %3, i1* %2)
  call fastcc void @"onebyonecpy_hls.p0struct.ap_uint<8>"(i8* align 512 %5, %"struct.ap_uint<8>"* %4)
  ret void
}

; Function Attrs: argmemonly noinline norecurse
define internal fastcc void @onebyonecpy_hls.p0i1(i1* noalias align 512, i1* noalias readonly) unnamed_addr #2 {
entry:
  %2 = icmp eq i1* %0, null
  %3 = icmp eq i1* %1, null
  %4 = or i1 %2, %3
  br i1 %4, label %ret, label %copy

copy:                                             ; preds = %entry
  %5 = bitcast i1* %1 to i8*
  %6 = load i8, i8* %5
  %7 = trunc i8 %6 to i1
  store i1 %7, i1* %0, align 512
  br label %ret

ret:                                              ; preds = %copy, %entry
  ret void
}

; Function Attrs: argmemonly noinline norecurse
define internal fastcc void @copy_out(i1* noalias "unpacked"="0", i1* noalias readonly align 512 "unpacked"="1", i1* noalias "unpacked"="2", i1* noalias readonly align 512 "unpacked"="3", %"struct.ap_uint<8>"* noalias "unpacked"="4", i8* noalias nocapture readonly align 512 "unpacked"="5.0.0.0") unnamed_addr #3 {
entry:
  call fastcc void @onebyonecpy_hls.p0i1(i1* %0, i1* align 512 %1)
  call fastcc void @onebyonecpy_hls.p0i1(i1* %2, i1* align 512 %3)
  call fastcc void @"onebyonecpy_hls.p0struct.ap_uint<8>.10"(%"struct.ap_uint<8>"* %4, i8* align 512 %5)
  ret void
}

; Function Attrs: argmemonly noinline norecurse
define internal fastcc void @"onebyonecpy_hls.p0struct.ap_uint<8>.10"(%"struct.ap_uint<8>"* noalias "unpacked"="0", i8* noalias nocapture readonly align 512 "unpacked"="1.0.0.0") unnamed_addr #2 {
entry:
  %2 = icmp eq %"struct.ap_uint<8>"* %0, null
  br i1 %2, label %ret, label %copy

copy:                                             ; preds = %entry
  %.01.0.05 = getelementptr %"struct.ap_uint<8>", %"struct.ap_uint<8>"* %0, i32 0, i32 0, i32 0, i32 0
  %3 = load i8, i8* %1, align 512
  store i8 %3, i8* %.01.0.05, align 1
  br label %ret

ret:                                              ; preds = %copy, %entry
  ret void
}

; Function Attrs: argmemonly noinline norecurse
define internal fastcc void @"onebyonecpy_hls.p0struct.ap_uint<8>"(i8* noalias nocapture align 512 "unpacked"="0.0.0.0", %"struct.ap_uint<8>"* noalias readonly "unpacked"="1") unnamed_addr #2 {
entry:
  %2 = icmp eq %"struct.ap_uint<8>"* %1, null
  br i1 %2, label %ret, label %copy

copy:                                             ; preds = %entry
  %.0.0.04 = getelementptr %"struct.ap_uint<8>", %"struct.ap_uint<8>"* %1, i32 0, i32 0, i32 0, i32 0
  %3 = load i8, i8* %.0.0.04, align 1
  store i8 %3, i8* %0, align 512
  br label %ret

ret:                                              ; preds = %copy, %entry
  ret void
}

declare void @apatb_top_hw(%"struct.ap_uint<8>"*, i1, i1, i1, %"struct.ap_uint<8>"*, i1, i1*, i1*, i8*)

; Function Attrs: argmemonly noinline norecurse
define internal fastcc void @copy_back(i1* noalias "unpacked"="0", i1* noalias readonly align 512 "unpacked"="1", i1* noalias "unpacked"="2", i1* noalias readonly align 512 "unpacked"="3", %"struct.ap_uint<8>"* noalias "unpacked"="4", i8* noalias nocapture readonly align 512 "unpacked"="5.0.0.0") unnamed_addr #3 {
entry:
  call fastcc void @onebyonecpy_hls.p0i1(i1* %0, i1* align 512 %1)
  call fastcc void @onebyonecpy_hls.p0i1(i1* %2, i1* align 512 %3)
  call fastcc void @"onebyonecpy_hls.p0struct.ap_uint<8>.10"(%"struct.ap_uint<8>"* %4, i8* align 512 %5)
  ret void
}

define void @top_hw_stub_wrapper(%"struct.ap_uint<8>"*, i1, i1, i1, %"struct.ap_uint<8>"*, i1, i1*, i1*, i8*) #4 {
entry:
  %9 = alloca %"struct.ap_uint<8>"
  call void @copy_out(i1* null, i1* %6, i1* null, i1* %7, %"struct.ap_uint<8>"* %9, i8* %8)
  call void @top_hw_stub(%"struct.ap_uint<8>"* %0, i1 %1, i1 %2, i1 %3, %"struct.ap_uint<8>"* %4, i1 %5, i1* %6, i1* %7, %"struct.ap_uint<8>"* %9)
  call void @copy_in(i1* null, i1* %6, i1* null, i1* %7, %"struct.ap_uint<8>"* %9, i8* %8)
  ret void
}

declare void @top_hw_stub(%"struct.ap_uint<8>"*, i1, i1, i1, %"struct.ap_uint<8>"*, i1, i1*, i1*, %"struct.ap_uint<8>"*)

attributes #0 = { noinline "fpga.wrapper.func"="wrapper" }
attributes #1 = { argmemonly noinline norecurse "fpga.wrapper.func"="copyin" }
attributes #2 = { argmemonly noinline norecurse "fpga.wrapper.func"="onebyonecpy_hls" }
attributes #3 = { argmemonly noinline norecurse "fpga.wrapper.func"="copyout" }
attributes #4 = { "fpga.wrapper.func"="stub" }

!llvm.dbg.cu = !{}
!llvm.ident = !{!0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0, !0}
!llvm.module.flags = !{!1, !2, !3}
!blackbox_cfg = !{!4}

!0 = !{!"clang version 7.0.0 "}
!1 = !{i32 2, !"Dwarf Version", i32 4}
!2 = !{i32 2, !"Debug Info Version", i32 3}
!3 = !{i32 1, !"wchar_size", i32 4}
!4 = !{}
