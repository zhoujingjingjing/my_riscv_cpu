// [RV32M移植] RV32M独立乘除法单元顶层。
// 本模块只负责按照funct3把请求分发给乘法器或除法器，并选择最终结果。
module rv32m_mdu (
    input  wire        clk,
    input  wire        resetn,
    input  wire        start,
    input  wire        cancel,
    input  wire [2:0]  op,
    input  wire [31:0] operand_a,
    input  wire [31:0] operand_b,
    output wire        busy,
    output wire        done,
    output wire [31:0] result
);

    // RV32M的funct3直接作为op：
    // 000 MUL，001 MULH，010 MULHSU，011 MULHU，
    // 100 DIV，101 DIVU，110 REM，111 REMU。
    // [RV32M移植] busy期间拒绝新请求，cancel同拍也不接收请求。
    wire accept_request = start && !busy && !cancel;
    wire mul_start = accept_request && !op[2];
    wire div_start = accept_request &&  op[2];

    wire        mul_busy;
    wire        mul_done;
    wire [31:0] mul_result;
    wire        div_busy;
    wire        div_done;
    wire [31:0] div_result;

    rv32m_mul u_rv32m_mul (
        .clk       (clk),
        .resetn    (resetn),
        .start     (mul_start),
        .cancel    (cancel),
        .op        (op),
        .operand_a (operand_a),
        .operand_b (operand_b),
        .busy      (mul_busy),
        .done      (mul_done),
        .result    (mul_result)
    );

    // [SRT16重写] 单文件除法器原生返回DIV/DIVU的商和REM/REMU的余数。
    rv32m_div u_rv32m_div (
        .clk       (clk),
        .resetn    (resetn),
        .start     (div_start),
        .cancel    (cancel),
        .op        (op),
        .operand_a (operand_a),
        .operand_b (operand_b),
        .busy      (div_busy),
        .done      (div_done),
        .result    (div_result)
    );

    // 一次只会启动一个后端，因此busy和done可以直接合并。
    assign busy   = mul_busy | div_busy;
    assign done   = mul_done | div_done;
    assign result = div_done ? div_result : mul_result;

endmodule

