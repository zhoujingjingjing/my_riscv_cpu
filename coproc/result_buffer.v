// ============================================================================
// result_buffer.v  —— 干净的"去斜结果捕获 + 按行读出"模块
// ----------------------------------------------------------------------------
// 背景：脉动阵列底行 out_sum[j] 每拍吐出 A@W 的一个对角元素：
//   C[m][j] = out_sum[j]  出现在  (结果起始拍 + m + j)。
// 本模块在结果浮现窗口内，把 out_sum 按对角线散射写入 rmem[m][j]，
// 完成去斜重组；之后 CPU 可按行号 result_raddr 读出整行(16 个 16bit)。
//
// 替换原 output_buffer 的理由：原模块只取高字节、且其捕获时机假定结果在
// OUTPUT 态之后才浮现，与"延长流式窗口后结果在 CALCULATE 末尾就浮现"的实际
// 时序不符；这里改成由 cap_en 精确框定捕获窗口、保存完整 16bit、行可寻址。
// ============================================================================
module result_buffer #(
    parameter N = 16
)(
    input  wire                 CLK,
    input  wire                 RESET,        // 低有效
    input  wire                 cap_en,       // 捕获窗口使能：结果浮现的连续 ~(2N-1) 拍内拉高
    input  wire [N*16-1:0]      out_sum,      // 阵列底行 16 个 16bit 累加和
    input  wire [3:0]           result_raddr, // 读出行号 0..15
    output wire [N*16-1:0]      result_row    // = rmem[result_raddr]，16 个 16bit
);
    integer m, j;
    reg signed [15:0] rmem [0:N-1][0:N-1];
    reg [4:0] cc;   // 捕获计数器：cap_en 高时每拍 +1，0..2N-2

    always @(posedge CLK or negedge RESET) begin
        if (~RESET) begin
            cc <= 5'd0;
            for (m = 0; m < N; m = m + 1)
                for (j = 0; j < N; j = j + 1)
                    rmem[m][j] <= 16'd0;
        end else if (cap_en) begin
            // 对角散射：本拍 out_sum[j] 属于输出行 m = cc - j（0<=m<N 时有效）
            for (j = 0; j < N; j = j + 1) begin
                if (cc >= j[4:0] && (cc - j) < N)
                    rmem[cc - j][j] <= out_sum[16*j +: 16];
            end
            cc <= cc + 5'd1;
        end
    end

    // 按行读出：把 rmem[result_raddr] 的 16 个 16bit 拼成 256 位
    genvar g;
    generate
        for (g = 0; g < N; g = g + 1) begin: rd
            assign result_row[16*g +: 16] = rmem[result_raddr][g];
        end
    endgenerate
endmodule
