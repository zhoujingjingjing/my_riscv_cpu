// [RV32M时序优化] 参考香山高基数SRT的多位商生成思路，针对固定RV32
// 重新实现为一个自包含的Radix-4非恢复除法器。
// 算法参考：https://github.com/OpenXiangShan/XS-Verilog-Library
// 参考提交：5463f180ae9d1ac0e1c0e62a38bcbabfa219d5ee
//
// [RV32M时序优化] 原实现每拍串联4次34位加减，在XC7K325T实现后形成
// 36个CARRY4的关键路径。现在每拍只串联2次微迭代、生成2位商，普通
// 32位除法最多需要16个迭代周期，再用1个独立FINALIZE周期完成结果。
// 文件职责：提供CPU可见的自包含DIV/DIVU/REM/REMU串行运算单元。
// 上游：rv32m_mdu在EXE发出start、操作码和两个操作数。
// 下游：rv32m_mdu等待busy/done并把result送回EXE；cancel来自异常冲刷。
module rv32m_div (
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

    //============================================================
    // operation classification and special cases
    //============================================================
    // 先判定有符号/取余，以及除零、0被除数、MIN_INT/-1等RISC-V规定的特殊返回值。
    // RV32M funct3：100 DIV，101 DIVU，110 REM，111 REMU。
    wire operation_is_signed = (op == 3'b100) || (op == 3'b110);
    wire operation_is_remainder = (op == 3'b110) || (op == 3'b111);

    wire divide_by_zero = operand_b == 32'b0;
    wire dividend_is_zero = operand_a == 32'b0;
    wire signed_overflow = operation_is_signed &&
                           (operand_a == 32'h8000_0000) &&
                           (operand_b == 32'hffff_ffff);

    // 有符号除法先计算两个绝对值。0x80000000取绝对值后仍是
    // 0x80000000，把它作为无符号数2147483648参与迭代即可。
    wire [31:0] dividend_magnitude =
        (operation_is_signed && operand_a[31])
        ? (~operand_a + 32'd1) : operand_a;
    wire [31:0] divisor_magnitude =
        (operation_is_signed && operand_b[31])
        ? (~operand_b + 32'd1) : operand_b;

    //============================================================
    // small helper functions
    //============================================================
    // [RV32M时序优化] 一次主迭代处理2位，启动时跳过最高端完整的零对。
    function [3:0] leading_zero_pairs;
        input [31:0] value;
        begin
            if      (|value[31:30]) leading_zero_pairs = 4'd0;
            else if (|value[29:28]) leading_zero_pairs = 4'd1;
            else if (|value[27:26]) leading_zero_pairs = 4'd2;
            else if (|value[25:24]) leading_zero_pairs = 4'd3;
            else if (|value[23:22]) leading_zero_pairs = 4'd4;
            else if (|value[21:20]) leading_zero_pairs = 4'd5;
            else if (|value[19:18]) leading_zero_pairs = 4'd6;
            else if (|value[17:16]) leading_zero_pairs = 4'd7;
            else if (|value[15:14]) leading_zero_pairs = 4'd8;
            else if (|value[13:12]) leading_zero_pairs = 4'd9;
            else if (|value[11:10]) leading_zero_pairs = 4'd10;
            else if (|value[9:8])   leading_zero_pairs = 4'd11;
            else if (|value[7:6])   leading_zero_pairs = 4'd12;
            else if (|value[5:4])   leading_zero_pairs = 4'd13;
            else if (|value[3:2])   leading_zero_pairs = 4'd14;
            else if (|value[1:0])   leading_zero_pairs = 4'd15;
            else                    leading_zero_pairs = 4'd0;
        end
    endfunction

    // 一个二进制SRT非恢复微迭代的规则是：
    //
    //   旧部分余数 >= 0：新部分余数 = 2×旧部分余数 + 下一位 - 除数
    //   旧部分余数 <  0：新部分余数 = 2×旧部分余数 + 下一位 + 除数
    //
    // 新部分余数非负时，本次商位为1；为负时，本次商位为0。
    // [RV32M时序优化] 函数在一个组合路径中只连续执行2次，因此一个
    // 时钟周期生成2位商，基数是2^2=4；组合加减链比原实现缩短一半。
    //
    // 返回值[65:32]是新的34位有符号部分余数，[31:0]是新的商寄存器。
    function [65:0] two_bit_srt_step;
        input [33:0] partial_in;
        input [31:0] quotient_in;
        input [31:0] divisor_in;
        reg signed [33:0] partial_work;
        reg signed [33:0] shifted_partial;
        reg signed [33:0] divisor_extended;
        reg        [31:0] quotient_work;
        integer bit_index;
        begin
            partial_work    = $signed(partial_in);
            quotient_work   = quotient_in;
            divisor_extended = $signed({2'b00, divisor_in});

            for (bit_index = 0; bit_index < 2; bit_index = bit_index + 1) begin
                // 左移部分余数，同时把尚未处理的被除数最高位送入最低位。
                shifted_partial = $signed({partial_work[32:0],
                                            quotient_work[31]});
                quotient_work = {quotient_work[30:0], 1'b0};

                if (partial_work[33] == 1'b0)
                    partial_work = shifted_partial - divisor_extended;
                else
                    partial_work = shifted_partial + divisor_extended;

                // 非恢复除法允许部分余数暂时为负，符号直接决定当前商位。
                quotient_work[0] = ~partial_work[33];
            end

            two_bit_srt_step = {partial_work, quotient_work};
        end
    endfunction

    //============================================================
    // divider state registers and next-step wires
    //============================================================
    // partial_remainder_q和quotient_q保存当前迭代状态；*_negative_q记录最后是否需要补码恢复符号。
    reg signed [33:0] partial_remainder_q;
    reg        [31:0] quotient_q;
    reg        [31:0] divisor_q;
    reg         [3:0] iteration_q;
    reg               return_remainder_q;
    reg               quotient_negative_q;
    reg               remainder_negative_q;
    reg               special_pending_q;
    reg               finalize_pending_q;
    reg        [31:0] special_result_q;

    wire [3:0] skipped_pairs = leading_zero_pairs(dividend_magnitude);
    wire [4:0] skipped_bits = {skipped_pairs, 1'b0};

    wire [65:0] srt_step_result = two_bit_srt_step(
        partial_remainder_q,
        quotient_q,
        divisor_q
    );
    wire signed [33:0] partial_remainder_next =
        $signed(srt_step_result[65:32]);
    wire [31:0] quotient_next = srt_step_result[31:0];

    // 32个商位全部生成后，非恢复算法可能留下一个负的部分余数。
    // 加回一次正除数即可得到RV32M需要的非负余数绝对值。
    // [RV32M时序优化] 修正和符号恢复只读取上一拍锁存的最终迭代结果。
    // 这样两次微迭代的组合路径终止于partial_remainder_q/quotient_q，
    // 不再继续串过余数修正、二进制补码和result寄存器。
    wire [31:0] corrected_remainder = partial_remainder_q[33]
        ? (partial_remainder_q[31:0] + divisor_q)
        : partial_remainder_q[31:0];

    wire [31:0] signed_quotient_result = quotient_negative_q
        ? (~quotient_q + 32'd1) : quotient_q;
    wire [31:0] signed_remainder_result = remainder_negative_q
        ? (~corrected_remainder + 32'd1)
        : corrected_remainder;

    //============================================================
    // start/busy/done sequential protocol
    //============================================================
    // start只在空闲时采样一次；done只保持一个周期。cancel优先级最高，用于
    // 更老异常或mret冲刷流水线时丢弃当前除法。
    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            busy                 <= 1'b0;
            done                 <= 1'b0;
            result               <= 32'b0;
            partial_remainder_q  <= 34'sd0;
            quotient_q           <= 32'b0;
            divisor_q            <= 32'b0;
            iteration_q          <= 4'b0;
            return_remainder_q   <= 1'b0;
            quotient_negative_q  <= 1'b0;
            remainder_negative_q <= 1'b0;
            special_pending_q    <= 1'b0;
            finalize_pending_q   <= 1'b0;
            special_result_q     <= 32'b0;
        end else begin
            // done只表示当前这一拍结果有效。
            done <= 1'b0;

            if (cancel) begin
                // cancel优先级最高；寄存器内容可以保留，但busy立即清零，
                // 后续时钟不会继续承认这次运算，也不会产生done。
                busy              <= 1'b0;
                special_pending_q <= 1'b0;
                finalize_pending_q <= 1'b0;
                iteration_q       <= 4'b0;
            end else if (!busy) begin
                if (start) begin
                    busy               <= 1'b1;
                    return_remainder_q <= operation_is_remainder;
                    finalize_pending_q <= 1'b0;

                    if (divide_by_zero || signed_overflow || dividend_is_zero) begin
                        special_pending_q <= 1'b1;
                        if (divide_by_zero)
                            special_result_q <= operation_is_remainder
                                ? operand_a : 32'hffff_ffff;
                        else if (signed_overflow)
                            special_result_q <= operation_is_remainder
                                ? 32'b0 : 32'h8000_0000;
                        else
                            special_result_q <= 32'b0;
                    end else begin
                        special_pending_q    <= 1'b0;
                        partial_remainder_q  <= 34'sd0;
                        // 跳过最高端完整零对后，把第一个有效2位组移到最高端。
                        quotient_q           <= dividend_magnitude << skipped_bits;
                        divisor_q            <= divisor_magnitude;
                        iteration_q          <= skipped_pairs;
                        quotient_negative_q  <= operation_is_signed &&
                                                (operand_a[31] ^ operand_b[31]);
                        remainder_negative_q <= operation_is_signed && operand_a[31];
                    end
                end
            end else if (special_pending_q) begin
                result            <= special_result_q;
                busy              <= 1'b0;
                done              <= 1'b1;
                special_pending_q <= 1'b0;
            end else if (finalize_pending_q) begin
                // [RV32M时序优化] FINALIZE独占一拍，输入全部来自寄存器，
                // 不再与最后一次two_bit_srt_step串在同一条组合路径上。
                result <= return_remainder_q
                    ? signed_remainder_result : signed_quotient_result;
                busy               <= 1'b0;
                done               <= 1'b1;
                finalize_pending_q <= 1'b0;
            end else if (iteration_q == 4'd15) begin
                // [RV32M时序优化] 最后一次迭代只锁存最终部分余数和商；
                // 下一拍再做余数修正、符号恢复和结果提交。
                partial_remainder_q <= partial_remainder_next;
                quotient_q          <= quotient_next;
                finalize_pending_q  <= 1'b1;
            end else begin
                partial_remainder_q <= partial_remainder_next;
                quotient_q          <= quotient_next;
                iteration_q         <= iteration_q + 4'd1;
            end
        end
    end

endmodule
