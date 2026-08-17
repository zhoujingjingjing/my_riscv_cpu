// Copyright lowRISC contributors.
// Copyright 2018 ETH Zurich and University of Bologna, see also the Ibex CREDITS.md.
// Licensed under the Apache License, Version 2.0, see third_party/rv32m/ibex/LICENSE.
// SPDX-License-Identifier: Apache-2.0

// [RV32M移植] 操作数拆成高、低16位并扩展为17位，使乘法映射到
// Xilinx 7系列DSP48E1。
//
// [RV32M时序优化] 原Ibex单周期路径在一拍内串联两个DSP48E1和一条
// CARRY4加法链，170MHz实现后建立时间失败。现在使用四个并行17x17
// 乘法单元：EXE只计算并寄存四个部分积，MEM再对齐、求和。
//
// [阶段5A修改] 去掉一次请求独占整个EXE的busy状态机。en表示EXE/MEM
// 边界本拍可以前进；in_valid可以连续每拍为1，out_valid与部分积一起保持。
// 文件职责：两级流水MUL单元，产生四个有符号/无符号组合下的64位乘积。
// 上游：rv32m_mdu送入操作数和valid；异常或mret通过cancel清除年轻结果。
// 下游：EXE/MEM使用result和mul_valid完成MUL写回，不阻塞普通ALU。
module rv32m_mul (
    input  wire        clk,
    input  wire        resetn,
    input  wire        en,
    input  wire        cancel,
    input  wire        in_valid,
    input  wire [2:0]  op,
    input  wire [31:0] operand_a,
    input  wire [31:0] operand_b,
    output reg         out_valid,
    output wire [31:0] result
);
    reg [2:0] op_q;

    //============================================================
    // operation classification and operand split
    //============================================================
    // funct3决定高半部分是否按有符号数扩展。先把控制信号放在操作数拆分前，
    // 阅读时就能直接看出MUL/MULH/MULHSU/MULHU四种路径的区别。
    wire operation_a_is_signed = (op == 3'b001) || (op == 3'b010);
    wire operation_b_is_signed = (op == 3'b001);

    // 低16位恒为无符号数；高16位是否符号扩展由MULH/MULHSU决定。
    // [阶段5A修改] EXE输入已经来自流水寄存器，直接完成17位格式转换；
    // 不再额外增加一拍操作数寄存器。
    wire signed [16:0] operand_a_low = $signed({1'b0, operand_a[15:0]});
    wire signed [16:0] operand_a_high =
        $signed({operation_a_is_signed && operand_a[31], operand_a[31:16]});
    wire signed [16:0] operand_b_low = $signed({1'b0, operand_b[15:0]});
    wire signed [16:0] operand_b_high =
        $signed({operation_b_is_signed && operand_b[31], operand_b[31:16]});

    //============================================================
    // parallel partial products
    //============================================================
    // [RV32M时序优化] 四个部分积都有独立结果寄存器。这些寄存器切断
    // “DSP乘法 -> DSP级联加法 -> CARRY4”的旧关键路径。
    reg signed [33:0] product_ll_q;
    reg signed [33:0] product_lh_q;
    reg signed [33:0] product_hl_q;
    reg signed [33:0] product_hh_q;

    // 四个17x17乘法并行执行，Vivado可分别映射到四个DSP48E1。
    wire signed [33:0] product_ll = operand_a_low  * operand_b_low;
    wire signed [33:0] product_lh = operand_a_low  * operand_b_high;
    wire signed [33:0] product_hl = operand_a_high * operand_b_low;
    wire signed [33:0] product_hh = operand_a_high * operand_b_high;


    //============================================================
    // MEM-stage balanced adder tree
    //============================================================
    // A = ah*2^16 + al，B = bh*2^16 + bl，因此：
    // A*B = al*bl + (al*bh + ah*bl)*2^16 + ah*bh*2^32。
    // 统一扩展到66位可以保留有符号部分积的符号，也容纳无符号64位乘积。
    wire signed [65:0] product_ll_aligned =
        $signed({{32{product_ll_q[33]}}, product_ll_q});
    wire signed [65:0] product_lh_aligned =
        $signed({{16{product_lh_q[33]}}, product_lh_q, 16'b0});
    wire signed [65:0] product_hl_aligned =
        $signed({{16{product_hl_q[33]}}, product_hl_q, 16'b0});
    wire signed [65:0] product_hh_aligned =
        $signed({product_hh_q, 32'b0});

// ||||||||00000000
//     ||||||||0000
//         ||||||||
    // [阶段5A时序控制] 两两并行求和，再做最后一级相加。相比四项连续
    // 表达式，这个写法明确告诉综合器使用平衡加法树，缩短MEM组合深度。
    wire signed [65:0] product_sum0 =
        product_ll_aligned + product_lh_aligned;
    wire signed [65:0] product_sum1 =
        product_hl_aligned + product_hh_aligned;
    wire signed [65:0] full_product = product_sum0 + product_sum1;

    assign result = (op_q == 3'b000)
        ? full_product[31:0]
        : full_product[63:32];

    //============================================================
    // valid/cancel and partial-product registers
    //============================================================
    // en代表EXE/MEM边界能前进；cancel来自更老异常或mret，必须立即
    // 作废out_valid，避免年轻MUL结果继续被MEM承认。
    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            op_q             <= 3'b000;
            product_ll_q     <= 34'sd0;
            product_lh_q     <= 34'sd0;
            product_hl_q     <= 34'sd0;
            product_hh_q     <= 34'sd0;
            out_valid        <= 1'b0;
        end else if (cancel) begin
            // trap或mret冲刷优先，旧部分积可以保留，但valid必须立即作废。
            out_valid <= 1'b0;
        end else if (en) begin
            // [阶段5A新增] en=0时控制和数据同时保持；en=1时即使输入是气泡，
            // 也要把out_valid清成0，防止旧MUL结果被重复承认。
            out_valid <= in_valid;
            if (in_valid) begin
                op_q         <= op;
                product_ll_q <= product_ll;
                product_lh_q <= product_lh;
                product_hl_q <= product_hl;
                product_hh_q <= product_hh;
            end
        end
    end

endmodule
