`include "mycpu_top.h"
`timescale 1ns / 1ps

/*
文件职责：IF1 是流水线的第一站，专门决定“下一拍从哪个 PC 取指”。
上游输入来自 EXE、ID、WB 和 BTB；输出送给 IF2、IROM 和下一拍 PC。
它同时查 BTB 的两个读口：slot0 读 pc，slot1 读 pc+4。
如果 slot0 预测跳转，就只取 slot0；否则两条都送给 IF2。
分支纠错、异常和 mret 都会改 PC。优先级是：异常最高，其次 mret，最后是普通分支纠错。
*/
module if_stage #(
    parameter [31:0] RESET_PC = 32'h8000_0000
)(
    input  wire        clk,
    input  wire        reset,

    input  wire        if2_allow_in,
    output wire [2:0]  if_ras_ptr,
    output wire        if_flush,
    output wire        if_to_if2_valid,
    output wire [`IF_TO_IF2_BUS_WIDTH-1:0] if_to_if2_bus,

    input  wire [`EXE_TO_IF_BUS_WIDTH-1:0] exe_to_if_bus,
    input  wire [`ID_TO_IF_BUS_WIDTH-1:0]  id_to_if_bus,

    /*WB 级发现异常或执行 mret 时，IF 必须丢掉旧路径。
    wb_ex 跳到异常入口 ex_entry；mret_flush 跳回 csr_mepc_out。*/
    input  wire        wb_ex,
    input  wire [31:0] ex_entry,
    input  wire        mret_flush,
    input  wire [31:0] csr_mepc_out,

    output wire        inst_sram_en,
    output wire [3:0]  inst_sram_we,
    output wire [31:0] inst_sram_addr,
    output wire [31:0] inst_sram_wdata,
    /*IROM 的 B 口只读 pc+4 的第二条指令，不会写存储器。*/
    output wire        inst_sram_en_b,
    output wire [31:0] inst_sram_addr_b
);
    localparam META_W = `FETCH_BUS_WIDTH - 32;

    //============================================================
    // redirect and prediction state
    //============================================================

    /*EXE 把真实分支结果送回来。
    flush_en 表示预测错了；exe_target 是正确目标；exe_we 等信号用于更新 BTB；
    exe_restore_ras_ptr 是取指时保存的 RAS 指针，预测错时用它恢复现场。*/
    wire        flush_en;
    wire [31:0] exe_target;
    wire        exe_we;
    wire [5:0]  exe_index;
    wire [14:0] exe_tag;
    wire        exe_taken;
    wire        exe_is_ret;
    wire [2:0]  exe_restore_ras_ptr;
    assign {flush_en, exe_target, exe_we, exe_index, exe_tag,
            exe_taken, exe_is_ret, exe_restore_ras_ptr} = exe_to_if_bus;

    /*ID 识别出函数调用或返回后，会在这里维护 RAS。
    mret 是 CSR 返回，不是函数返回，所以不会经过这组信号。*/
    wire        id_push_ras;
    wire        id_pop_ras;
    wire [31:0] id_ras_wdata;
    assign {id_push_ras, id_pop_ras, id_ras_wdata} = id_to_if_bus;

    /*flush 是“旧取指路径作废”的总开关。
    如果多个原因同拍出现，选择顺序固定为：异常入口 > mret 返回地址 > 分支目标。*/
    wire flush = wb_ex || mret_flush || flush_en;
    wire [31:0] flush_pc = wb_ex      ? ex_entry     :
                           mret_flush ? csr_mepc_out : exe_target;

    /*pc 是本拍 slot0 的取指地址；pc_b 是紧邻的 slot1 地址。*/
    reg [31:0] pc;
    wire [31:0] pc_b = pc + 32'd4;

    /*BTB 仍然只有一份表，但有两个读口。
    A 口给较老的 slot0，B 口给较年轻的 slot1；两条指令共享同一套预测状态。*/
    wire        pre_taken;
    wire [31:0] pre_target;
    wire        pre_is_ret;
    wire [5:0]  pre_index;
    wire        pre_taken_b;
    wire [31:0] pre_target_b;
    wire        pre_is_ret_b;
    wire [5:0]  pre_index_b;
    wire [2:0]  current_ras_ptr;

    /*u_btb 保存预测表、两位饱和计数器和 RAS。
    这里没有复制预测状态，只增加了第二个查询口。*/
    btb #(
        .INDEX_LEN(6),
        .TAG_LEN(15)
    ) u_btb (
        .clk(clk),
        .reset(reset),
        .if_pc(pc),
        .pre_taken(pre_taken),
        .pre_target(pre_target),
        .pre_is_ret(pre_is_ret),
        .pre_index(pre_index),
        .if_pc_b(pc_b),
        .pre_taken_b(pre_taken_b),
        .pre_target_b(pre_target_b),
        .pre_is_ret_b(pre_is_ret_b),
        .pre_index_b(pre_index_b),
        .id_push_ras(id_push_ras),
        .id_ras_wdata(id_ras_wdata),
        .id_pop_ras(id_pop_ras),
        .exe_we(exe_we),
        .exe_index(exe_index),
        .exe_tag(exe_tag),
        .exe_is_ret(exe_is_ret),
        .exe_taken(exe_taken),
        .exe_target(exe_target),
        .current_ras_ptr(current_ras_ptr),
        .flush_en(flush_en),
        .exe_restore_ras_ptr(exe_restore_ras_ptr)
    );

    //============================================================
    // IF1 to IF2 request
    //============================================================

    /*一次请求最多带两条指令。
    slot0 预测跳转时，pc+4 已经是错误路径，所以只请求一条。
    slot0 不跳转时，slot1 仍在顺序路径上，即使 slot1 自己预测跳转，也先把两条取回来。*/
    wire [1:0] fetch_num = pre_taken ? 2'd1 : 2'd2;
    wire [31:0] pred_pc = pre_taken   ? pre_target   :
                          pre_taken_b ? pre_target_b : pc + 32'd8;
    wire [META_W-1:0] req0 = {pc, pre_taken, pre_target, pre_index};
    wire [META_W-1:0] req1 = {pc_b, pre_taken_b, pre_target_b, pre_index_b};

    /*IF1 每拍都把当前 PC 和预测结果摆在输出线上。
    只有 IF2 说 FIFO 有空间时，PC 才在时钟沿推进到 pred_pc。*/
    assign if_to_if2_valid = !reset;
    assign if_to_if2_bus = {fetch_num, req0, req1};
    assign if_ras_ptr = current_ras_ptr;
    assign if_flush = flush;

    always @(posedge clk) begin
        if (reset) begin
            pc <= RESET_PC;
        end else if (flush) begin
            /*冲刷优先于普通顺序取指。时钟沿把 PC 改到选好的目标地址。*/
            pc <= flush_pc;
        end else if (if_to_if2_valid && if2_allow_in) begin
            pc <= pred_pc;
        end
    end

    //============================================================
    // dual-port instruction memory request
    //============================================================

    /*IROM 是只读存储器，所以写使能和写数据固定为 0。
    两个读口保持开启；真正决定返回值能不能进 FIFO 的，是 IF2 保存的 req_valid/req_num。
    地址仍是字节地址，外层连接 BRAM 时再按工程约定取地址位。*/
    assign inst_sram_en = 1'b1;
    assign inst_sram_we = 4'b0;
    assign inst_sram_addr = pc;
    assign inst_sram_wdata = 32'b0;
    assign inst_sram_en_b = 1'b1;
    assign inst_sram_addr_b = pc_b;

    /*当前路径暂时不使用两个“是否为返回指令”的预测位。
    把它们收拢到 keep_unused，只是告诉工具这些线是有意保留的。*/
    wire keep_unused = pre_is_ret || pre_is_ret_b;
endmodule
