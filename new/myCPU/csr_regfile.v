// 文件职责：实现M模式CSR寄存器、异常入口/返回状态和性能计数器。
// 上游：WB提供CSR写入、ecall、mret和异常信息；顶层提供当前指令/周期事件。
// 下游：EXE读取CSR旧值，IF读取mepc/异常入口，调试接口读取计数器。
// ============================================================
// csr_regfile.v  —— M 模式 CSR 寄存器堆（任务12：ecall/mret/CSR指令）
// ============================================================
module csr_regfile (
    input  wire        clk,
    input  wire        reset,

    // ---- 指令访问接口（来自 WB 级）----
    input  wire [11:0] csr_addr,    // 12位 CSR 地址
    input  wire        csr_we,      // 写使能（CSR指令写时为1）
    input  wire [31:0] csr_wvalue,  // 写入值（已由WB级算好最终值）
    output wire [31:0] csr_rvalue,  // 读出值（组合逻辑异步读）

    // [第4步验证修复] EXE专用只读口。WB口仍用于CSR写地址和rd旧值读取。
    input  wire [11:0] exe_csr_addr,
    output wire [31:0] exe_csr_rvalue,

    // ---- 异常触发接口（来自 WB 级）----
    input  wire        wb_ex,       // WB级有异常触发
    input  wire [31:0] wb_pc,       // 触发异常的指令PC
    input  wire [31:0] wb_cause,    // 异常原因（mcause内容）
    input  wire [31:0] wb_tval,     // 异常附加信息（mtval内容）

    // ---- mret 接口（来自 WB 级）----
    input  wire        mret_flush,  // mret指令在WB级执行

    // [测试框架完善] 性能事件输入。retire_count为本拍WB退休数，
    // 当前单发射CPU只能是0或1，接口保留8位以便未来多发射扩展。
    input  wire [7:0]  retire_count,
    input  wire        load_use_stall_event,
    input  wire        branch_event,
    input  wire        branch_mispredict_event,
    // [阶段2新增] 影子双发射配对事件，仅用于硬件性能分析，不映射CSR。
    input  wire        pair_event,
    input  wire        pair_ok_event,
    input  wire        pair_raw_event,
    input  wire        pair_waw_event,
    input  wire        pair_lsu_event,
    input  wire        pair_mul_event,
    input  wire        pair_ctrl_event,
    input  wire        pair_serial_event,

    // ---- 输出到流水线 ----
    output wire [31:0] ex_entry,    // 异常入口地址 → pre-IF（nextpc选择）
    output wire [31:0] csr_mepc_out,// mepc → pre-IF（mret时作为nextpc）

    // [测试框架完善] Performance Interface V1导出的原始64位累计值。
    output wire [63:0] perf_mcycle,
    output wire [63:0] perf_minstret,
    output wire [63:0] perf_load_use_stall_count,
    output wire [63:0] perf_branch_count,
    output wire [63:0] perf_mispredict_count,
    // [阶段2新增] 配对统计直接导出给仿真测试框架，不改变软件可见CSR空间。
    output wire [63:0] perf_pair_count,
    output wire [63:0] perf_pair_ok_count,
    output wire [63:0] perf_pair_raw_count,
    output wire [63:0] perf_pair_waw_count,
    output wire [63:0] perf_pair_lsu_count,
    output wire [63:0] perf_pair_mul_count,
    output wire [63:0] perf_pair_ctrl_count,
    output wire [63:0] perf_pair_serial_count,

    // ---- Hart ID（只读，外部输入）----
    input  wire [31:0] coreid_in
);

// ================================================================
// CSR 地址常量
// ================================================================
localparam CSR_MSTATUS   = 12'h300;
localparam CSR_MISA      = 12'h301;
localparam CSR_MIE       = 12'h304;
localparam CSR_MTVEC     = 12'h305;
localparam CSR_MSCRATCH  = 12'h340;
localparam CSR_MEPC      = 12'h341;
localparam CSR_MCAUSE    = 12'h342;
localparam CSR_MTVAL     = 12'h343;
localparam CSR_MIP       = 12'h344;
localparam CSR_MCYCLE    = 12'hB00;
localparam CSR_MINSTRET  = 12'hB02;
localparam CSR_MCYCLEH   = 12'hB80;
localparam CSR_MINSTRETH = 12'hB82;
localparam CSR_MHARTID   = 12'hF14;

// ================================================================
// mstatus：MIE(bit3) / MPIE(bit7) / MPP(bit12:11)
// ================================================================
reg        csr_mstatus_mie;
reg        csr_mstatus_mpie;
reg [1:0]  csr_mstatus_mpp;

always @(posedge clk) begin
    if (reset) begin
        csr_mstatus_mie  <= 1'b0;
        csr_mstatus_mpie <= 1'b1;
        csr_mstatus_mpp  <= 2'b11;  // 复位后在 M 模式
    end
    else if (wb_ex) begin
        // 进入异常：保存 MIE→MPIE，关中断，MPP记当前模式
        csr_mstatus_mpie <= csr_mstatus_mie;
        csr_mstatus_mie  <= 1'b0;
        csr_mstatus_mpp  <= 2'b11;  // 当前只有 M 模式
    end
    else if (mret_flush) begin
        // 退出异常：MPIE→MIE，MPIE置1，MPP置U(00)
        csr_mstatus_mie  <= csr_mstatus_mpie;
        csr_mstatus_mpie <= 1'b1;
        csr_mstatus_mpp  <= 2'b00;
    end
    else if (csr_we && csr_addr == CSR_MSTATUS) begin
        csr_mstatus_mie  <= csr_wvalue[3];
        csr_mstatus_mpie <= csr_wvalue[7];
        csr_mstatus_mpp  <= csr_wvalue[12:11];
    end
end

// ================================================================
// mtvec：异常入口基地址（Direct模式，低2位为MODE）
// ================================================================
reg [31:0] csr_mtvec;

always @(posedge clk) begin
    if (reset)
        csr_mtvec <= 32'h80000000;  // 复位值，可按实际SoC地址调整
    else if (csr_we && csr_addr == CSR_MTVEC)
        csr_mtvec <= csr_wvalue;
end

// Direct模式：所有异常/中断均跳转到 BASE（低2位清零）
assign ex_entry = {csr_mtvec[31:2], 2'b00};

// ================================================================
// mepc：异常返回 PC（低2位硬件保证为0）
// ================================================================
reg [31:0] csr_mepc;

always @(posedge clk) begin
    if (wb_ex)
        csr_mepc <= {wb_pc[31:2], 2'b00};
    else if (csr_we && csr_addr == CSR_MEPC)
        csr_mepc <= {csr_wvalue[31:2], 2'b00};
end

assign csr_mepc_out = csr_mepc;

// ================================================================
// mcause：异常原因（bit31=中断标志，bit30:0=异常代码）
// ================================================================
reg [31:0] csr_mcause;

always @(posedge clk) begin
    if (reset)
        csr_mcause <= 32'b0;
    else if (wb_ex)
        csr_mcause <= wb_cause;
    else if (csr_we && csr_addr == CSR_MCAUSE)
        csr_mcause <= csr_wvalue;
end

// ================================================================
// mtval：异常附加信息
// ================================================================
reg [31:0] csr_mtval;

always @(posedge clk) begin
    if (reset)
        csr_mtval <= 32'b0;
    else if (wb_ex)
        csr_mtval <= wb_tval;
    else if (csr_we && csr_addr == CSR_MTVAL)
        csr_mtval <= csr_wvalue;
end

// ================================================================
// mscratch：软件临时寄存器，纯软件读写
// ================================================================
reg [31:0] csr_mscratch;

always @(posedge clk) begin
    if (reset)
        csr_mscratch <= 32'b0;
    else if (csr_we && csr_addr == CSR_MSCRATCH)
        csr_mscratch <= csr_wvalue;
end

// ================================================================
// mhartid：Hart ID，只读，由外部输入
// ================================================================
wire [31:0] csr_mhartid = coreid_in;

// ================================================================
// mcycle：64位时钟周期计数器（RV32拆为高低两个32位CSR）
// ================================================================
reg [63:0] csr_mcycle;

always @(posedge clk) begin
    if (reset)
        csr_mcycle <= 64'b0;
    else if (csr_we && csr_addr == CSR_MCYCLE)
        csr_mcycle[31:0]  <= csr_wvalue;
    else if (csr_we && csr_addr == CSR_MCYCLEH)
        csr_mcycle[63:32] <= csr_wvalue;
    else
        csr_mcycle <= csr_mcycle + 64'b1;  // 每周期自增
end

// ================================================================
// minstret：64位已退休指令计数器
// ================================================================
reg [63:0] csr_minstret;

always @(posedge clk) begin
    if (reset)
        csr_minstret <= 64'b0;
    else if (csr_we && csr_addr == CSR_MINSTRET)
        csr_minstret[31:0]  <= csr_wvalue;
    else if (csr_we && csr_addr == CSR_MINSTRETH)
        csr_minstret[63:32] <= csr_wvalue;
    // [测试框架完善] 按本拍实际退休数累加；不能按取指数或写回寄存器数计数。
    else
        csr_minstret <= csr_minstret + {56'b0, retire_count};
end

// [测试框架完善] 三类微架构事件使用独立64位计数器。它们只用于性能分析，
// 不参与CPU功能路径；复位清零后一直累计到本次程序场景结束。
reg [63:0] load_use_stall_count;
reg [63:0] branch_count;
reg [63:0] mispredict_count;

// [阶段2新增] 影子配对累计值。每个阻塞事件由配对器固定优先级产生，
// 因而一次尝试只会落入成功或某一种阻塞原因，不会重复分类。
reg [63:0] pair_count;
reg [63:0] pair_ok_count;
reg [63:0] pair_raw_count;
reg [63:0] pair_waw_count;
reg [63:0] pair_lsu_count;
reg [63:0] pair_mul_count;
reg [63:0] pair_ctrl_count;
reg [63:0] pair_serial_count;
// [阶段2新增] 先把8个分类事件打一拍，再驱动64位累加器。
// 这样“第二槽译码->相关比较->原因优先级”不会继续串到64位加法器，
// 配对观察逻辑不会加深当前ID真实发射路径，也尽量降低旁路时序压力。
reg [7:0] pair_event_q;

always @(posedge clk) begin
    if (reset) begin
        load_use_stall_count <= 64'b0;
        branch_count         <= 64'b0;
        mispredict_count     <= 64'b0;
    end else begin
        if (load_use_stall_event)
            load_use_stall_count <= load_use_stall_count + 64'd1;
        if (branch_event)
            branch_count <= branch_count + 64'd1;
        if (branch_mispredict_event)
            mispredict_count <= mispredict_count + 64'd1;
    end
end

always @(posedge clk) begin
    if (reset) begin
        pair_event_q <= 8'b0;
        pair_count        <= 64'b0;
        pair_ok_count     <= 64'b0;
        pair_raw_count    <= 64'b0;
        pair_waw_count    <= 64'b0;
        pair_lsu_count    <= 64'b0;
        pair_mul_count    <= 64'b0;
        pair_ctrl_count   <= 64'b0;
        pair_serial_count <= 64'b0;
    end else begin
        pair_event_q <= {
            pair_event, pair_ok_event, pair_raw_event, pair_waw_event,
            pair_lsu_event, pair_mul_event, pair_ctrl_event, pair_serial_event
        };
        if (pair_event_q[7])
            pair_count <= pair_count + 64'd1;
        if (pair_event_q[6])
            pair_ok_count <= pair_ok_count + 64'd1;
        if (pair_event_q[5])
            pair_raw_count <= pair_raw_count + 64'd1;
        if (pair_event_q[4])
            pair_waw_count <= pair_waw_count + 64'd1;
        if (pair_event_q[3])
            pair_lsu_count <= pair_lsu_count + 64'd1;
        if (pair_event_q[2])
            pair_mul_count <= pair_mul_count + 64'd1;
        if (pair_event_q[1])
            pair_ctrl_count <= pair_ctrl_count + 64'd1;
        if (pair_event_q[0])
            pair_serial_count <= pair_serial_count + 64'd1;
    end
end

// [测试框架完善] 正式端口导出，避免C++依赖Verilator生成的内部层次名称。
assign perf_mcycle               = csr_mcycle;
assign perf_minstret             = csr_minstret;
assign perf_load_use_stall_count = load_use_stall_count;
assign perf_branch_count         = branch_count;
assign perf_mispredict_count     = mispredict_count;
assign perf_pair_count           = pair_count;
assign perf_pair_ok_count        = pair_ok_count;
assign perf_pair_raw_count       = pair_raw_count;
assign perf_pair_waw_count       = pair_waw_count;
assign perf_pair_lsu_count       = pair_lsu_count;
assign perf_pair_mul_count       = pair_mul_count;
assign perf_pair_ctrl_count      = pair_ctrl_count;
assign perf_pair_serial_count    = pair_serial_count;

// ================================================================
// CSR 读出多路选择器（组合逻辑，同步于写回级译码）
// ================================================================
wire [31:0] mstatus_rval = {
    19'b0,
    csr_mstatus_mpp,   // [12:11]
    2'b0,              // [10:9] 保留
    1'b0,              // [8]  SPP
    csr_mstatus_mpie,  // [7]
    3'b0,              // [6:4] 保留/SPIE/UPIE
    csr_mstatus_mie,   // [3]
    3'b0               // [2:0] 保留/SIE/UIE
};

// [第4步验证修复] 两个读口共用同一份地址译码，保证EXE和WB观察到同一份CSR状态。
function automatic [31:0] csr_read_mux;
    input [11:0] read_addr;
    begin
        csr_read_mux =
            ({32{read_addr == CSR_MSTATUS  }} & mstatus_rval        ) |
            ({32{read_addr == CSR_MTVEC    }} & csr_mtvec           ) |
            ({32{read_addr == CSR_MEPC     }} & csr_mepc            ) |
            ({32{read_addr == CSR_MCAUSE   }} & csr_mcause          ) |
            ({32{read_addr == CSR_MTVAL    }} & csr_mtval            ) |
            ({32{read_addr == CSR_MSCRATCH }} & csr_mscratch        ) |
            ({32{read_addr == CSR_MHARTID  }} & csr_mhartid         ) |
            ({32{read_addr == CSR_MCYCLE   }} & csr_mcycle[31:0]    ) |
            ({32{read_addr == CSR_MCYCLEH  }} & csr_mcycle[63:32]   ) |
            ({32{read_addr == CSR_MINSTRET }} & csr_minstret[31:0]  ) |
            ({32{read_addr == CSR_MINSTRETH}} & csr_minstret[63:32] );
    end
endfunction

assign csr_rvalue     = csr_read_mux(csr_addr);
assign exe_csr_rvalue = csr_read_mux(exe_csr_addr);

endmodule
