// Copyright lowRISC contributors.
// Copyright 2018 ETH Zurich and University of Bologna, see also the Ibex CREDITS.md.
// Licensed under the Apache License, Version 2.0, see third_party/rv32m/ibex/LICENSE.
// SPDX-License-Identifier: Apache-2.0

// [RV32M移植] 操作数拆成高、低16位并扩展为17位，使乘法映射到
// Xilinx 7系列DSP48E1。接口保持start/busy/done/result/cancel不变。
//
// [RV32M时序优化] 原Ibex单周期路径在一拍内串联两个DSP48E1和一条
// CARRY4加法链，170MHz实现后建立时间失败。现在使用四个并行17x17
// 乘法单元：第一拍只计算并寄存四个部分积，第二拍对齐、求和并提交结果。
module rv32m_mul (
    input  wire        clk,
    input  wire        resetn,
    input  wire        start,
    input  wire        cancel,
    input  wire [2:0]  op,
    input  wire [31:0] operand_a,
    input  wire [31:0] operand_b,
    output reg         busy,
    output reg         done,
    output reg  [31:0] result
);

    localparam [1:0] STATE_IDLE    = 2'd0;
    localparam [1:0] STATE_PRODUCT = 2'd1;
    localparam [1:0] STATE_SUM     = 2'd2;

    reg [1:0] state;
    reg [2:0] op_q;

    // 低16位恒为无符号数；高16位是否符号扩展由MULH/MULHSU决定。
    // 在接收请求时完成这种17位格式转换，DSP计算拍不再经过op译码。
    reg signed [16:0] operand_a_low_q;
    reg signed [16:0] operand_a_high_q;
    reg signed [16:0] operand_b_low_q;
    reg signed [16:0] operand_b_high_q;

    // [RV32M时序优化] 四个部分积都有独立结果寄存器。这些寄存器切断
    // “DSP乘法 -> DSP级联加法 -> CARRY4”的旧关键路径。
    reg signed [33:0] product_ll_q;
    reg signed [33:0] product_lh_q;
    reg signed [33:0] product_hl_q;
    reg signed [33:0] product_hh_q;

    wire operation_a_is_signed = (op == 3'b001) || (op == 3'b010);
    wire operation_b_is_signed = (op == 3'b001);

    // 四个17x17乘法并行执行，Vivado可分别映射到四个DSP48E1。
    wire signed [33:0] product_ll = operand_a_low_q  * operand_b_low_q;
    wire signed [33:0] product_lh = operand_a_low_q  * operand_b_high_q;
    wire signed [33:0] product_hl = operand_a_high_q * operand_b_low_q;
    wire signed [33:0] product_hh = operand_a_high_q * operand_b_high_q;

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

    wire signed [65:0] full_product =
        product_ll_aligned + product_lh_aligned +
        product_hl_aligned + product_hh_aligned;

    wire [31:0] selected_result = (op_q == 3'b000)
        ? full_product[31:0]
        : full_product[63:32];

    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            state            <= STATE_IDLE;
            op_q             <= 3'b000;
            operand_a_low_q  <= 17'sd0;
            operand_a_high_q <= 17'sd0;
            operand_b_low_q  <= 17'sd0;
            operand_b_high_q <= 17'sd0;
            product_ll_q     <= 34'sd0;
            product_lh_q     <= 34'sd0;
            product_hl_q     <= 34'sd0;
            product_hh_q     <= 34'sd0;
            busy             <= 1'b0;
            done             <= 1'b0;
            result           <= 32'b0;
        end else begin
            // done只在结果提交的当前拍有效。
            done <= 1'b0;

            if (cancel) begin
                // trap或mret冲刷优先于计算完成，被取消请求永远不产生done。
                state <= STATE_IDLE;
                busy  <= 1'b0;
            end else begin
                case (state)
                    STATE_IDLE: begin
                        busy <= 1'b0;
                        if (start) begin
                            op_q <= op;
                            operand_a_low_q <=
                                $signed({1'b0, operand_a[15:0]});
                            operand_a_high_q <=
                                $signed({operation_a_is_signed && operand_a[31],
                                         operand_a[31:16]});
                            operand_b_low_q <=
                                $signed({1'b0, operand_b[15:0]});
                            operand_b_high_q <=
                                $signed({operation_b_is_signed && operand_b[31],
                                         operand_b[31:16]});
                            busy  <= 1'b1;
                            state <= STATE_PRODUCT;
                        end
                    end

                    STATE_PRODUCT: begin
                        // [RV32M时序优化] 第一计算拍只锁存DSP输出。
                        product_ll_q <= product_ll;
                        product_lh_q <= product_lh;
                        product_hl_q <= product_hl;
                        product_hh_q <= product_hh;
                        state        <= STATE_SUM;
                    end

                    STATE_SUM: begin
                        // 第二计算拍合并已寄存部分积，四条乘法统一在此完成。
                        result <= selected_result;
                        busy   <= 1'b0;
                        done   <= 1'b1;
                        state  <= STATE_IDLE;
                    end

                    default: begin
                        state <= STATE_IDLE;
                        busy  <= 1'b0;
                    end
                endcase
            end
        end
    end

endmodule
