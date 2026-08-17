// [RV32M移植] RV32M独立乘除法单元顶层。
//
// [阶段5A修改] 乘法和除法不再共用busy/done协议：MUL是固定延迟、
// 每拍可接收一条的流水单元；DIV/REM仍是一次只处理一条的串行单元。
// 两个后端共享操作数字段，是因为ID配对器已经保证一组最多一条M指令。
// 文件职责：把四条乘法和四条除法映射到各自的RV32M运算单元。
// 上游：EXE提供M指令类型、操作数和启动/取消控制。
// 下游：EXE接收连续MUL结果或串行DIV结果；本模块不改变流水级边界。
module rv32m_mdu (
    input  wire        clk,
    input  wire        resetn,
    input  wire        cancel,

    input  wire        mul_en,
    input  wire        mul_valid,
    output wire        mul_valid_o,
    output wire [31:0] mul_result,

    input  wire        div_start,
    output wire        div_busy,
    output wire        div_done,
    output wire [31:0] div_result,

    input  wire [2:0]  op,
    input  wire [31:0] operand_a,
    input  wire [31:0] operand_b
);

    //============================================================
    // shared EXE-side MDU protocol
    //============================================================
    // EXE一次最多送来一条M指令，op[2]把乘法和除法分开：
    // 乘法走固定两级流水，除法走可取消的串行状态机。
    // 两条路径共享输入端口，但各自拥有独立的valid/busy协议，
    // 因而连续MUL不会等待DIV结束，DIV也不会误采纳MUL结果。

    //============================================================
    // pipelined multiplier
    //============================================================
    // RV32M的funct3直接作为op：
    // 000 MUL，001 MULH，010 MULHSU，011 MULHU，
    // 100 DIV，101 DIVU，110 REM，111 REMU。
    rv32m_mul u_rv32m_mul (
        .clk       (clk),
        .resetn    (resetn),
        .en        (mul_en),
        .cancel    (cancel),
        .in_valid  (mul_valid),
        .op        (op),
        .operand_a (operand_a),
        .operand_b (operand_b),
        .out_valid (mul_valid_o),
        .result    (mul_result)
    );

    // 乘法输出在MEM级消费；mul_valid_o与result属于同一个流水拍。

    //============================================================
    // iterative divider
    //============================================================
    // [SRT16重写] 单文件除法器原生返回DIV/DIVU的商和REM/REMU的余数。
    // [阶段5A保留] div_start只在外层确认除法器空闲时产生；DIV协议和
    // 算术实现均不因MUL流水化而改变。
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

    // 除法只在div_start握手时锁存操作数，完成时用div_done给出一个拍的脉冲。

endmodule
