`include "mycpu_top.h"

// 文件职责：七级流水的WB级，按年龄提交CSR/异常、通用寄存器写回和退休信息。
// 上游：mem_stage提供两个MEM/WB总线；CSR寄存器堆接收slot0的CSR提交。
// 下游：regfile、CSR、IF和顶层调试/退休接口接收本级结果。
// [阶段4修改] 双槽WB级。slot0始终更老，slot1只在slot0有效时有效。
// 两条普通指令可同拍产生两个寄存器堆写口和两份退休元数据。
module wb_stage(
    input  wire clk,
    input  wire reset,
    input  wire [`MEM_TO_WB_BUS_WIDTH-1:0] mem_to_wb_bus,
    input  wire [`MEM_TO_WB_BUS_WIDTH-1:0] mem_to_wb_bus1,
    output wire [`WB_TO_ID_BUS_WIDTH-1:0] wb_to_id_bus,
    output wire [`WB_TO_ID_BUS_WIDTH-1:0] wb_to_id_bus1,
    input  wire mem_to_wb_valid,
    input  wire mem_to_wb_valid1,
    output wire wb_allow_in,
    output wire wb_ex,
    output wire [31:0] wb_pc_for_csr,
    output wire [31:0] wb_cause,
    output wire [31:0] wb_tval,
    output wire mret_flush,
    output wire wb_csr_we,
    output wire [11:0] wb_csr_addr,
    output wire [31:0] wb_csr_wdata,
    input  wire [31:0] csr_rvalue,
    output wire [31:0] debug_wb_pc,
    output wire [3:0]  debug_wb_rf_we,
    output wire [4:0]  debug_wb_rf_wnum,
    output wire [31:0] debug_wb_rf_wdata,
    output wire        debug_wb_valid,
    output wire [31:0] debug_wb_inst,
    output wire [31:0] debug_wb_dnpc,
    output wire        debug_wb_mem_valid,
    output wire        debug_wb_mem_store,
    output wire [31:0] debug_wb_mem_addr,
    output wire [3:0]  debug_wb_mem_mask,
    output wire [31:0] debug_wb_mem_wdata,
    output wire [31:0] debug_wb_mem_rdata,
    output wire        debug_wb_trap_valid,
    output wire [31:0] debug_wb_trap_cause,
    output wire [31:0] debug_wb_pc1,
    output wire [3:0]  debug_wb_rf_we1,
    output wire [4:0]  debug_wb_rf_wnum1,
    output wire [31:0] debug_wb_rf_wdata1,
    output wire        debug_wb_valid1,
    output wire [31:0] debug_wb_inst1,
    output wire [31:0] debug_wb_dnpc1,
    output wire        debug_wb_mem_valid1,
    output wire        debug_wb_mem_store1,
    output wire [31:0] debug_wb_mem_addr1,
    output wire [3:0]  debug_wb_mem_mask1,
    output wire [31:0] debug_wb_mem_wdata1,
    output wire [31:0] debug_wb_mem_rdata1,
    output wire        debug_wb_trap_valid1,
    output wire [31:0] debug_wb_trap_cause1
);
    //============================================================
    // pipeline registers
    //============================================================

    // 白话信号导读：
    // wb_valid/wb_valid1是WB两个退休座位的“有人没人”标记；slot1只有在slot0有效时才可能有效。
    // wb_reg/wb_reg1是MEM交来的整包退休资料，包含PC、结果、访存记录、CSR写数据和异常标记。
    // wb_allow_in恒为1，因为WB已经是流水线最后一站，没有下一级会反压它。
    // mem_to_wb_valid1不会单独进入WB，必须被mem_to_wb_valid罩住，保证退休顺序永远连续不空洞。
    reg wb_valid;
    reg wb_valid1;
    reg [`MEM_TO_WB_BUS_WIDTH-1:0] wb_reg;
    reg [`MEM_TO_WB_BUS_WIDTH-1:0] wb_reg1;
    // WB是本CPU最后一级，没有下游反压，所以allow_in恒为1。
    // valid1必须依赖valid0，保证退休接口永远是“slot0有效后slot1才可能有效”。
    assign wb_allow_in = 1'b1;

    always @(posedge clk) begin
        if (reset) begin
            wb_valid <= 1'b0;
            wb_valid1 <= 1'b0;
        end else begin
            wb_valid <= mem_to_wb_valid;
            wb_valid1 <= mem_to_wb_valid && mem_to_wb_valid1;
        end
    end

    always @(posedge clk) begin
        if (mem_to_wb_valid) begin
            wb_reg <= mem_to_wb_bus;
            wb_reg1 <= mem_to_wb_bus1;
        end
    end

    //============================================================
    // slot0: oldest instruction, CSR and trap owner
    //============================================================

    wb_lane u_lane0(
        .bus(wb_reg), .valid(wb_valid), .csr_rvalue(csr_rvalue),
        .wb_bus(wb_to_id_bus), .wb_ex(wb_ex),
        .wb_pc_for_csr(wb_pc_for_csr), .wb_cause(wb_cause), .wb_tval(wb_tval),
        .mret_flush(mret_flush), .wb_csr_we(wb_csr_we),
        .wb_csr_addr(wb_csr_addr), .wb_csr_wdata(wb_csr_wdata),
        .debug_pc(debug_wb_pc), .debug_rf_we(debug_wb_rf_we),
        .debug_rf_rd(debug_wb_rf_wnum), .debug_rf_data(debug_wb_rf_wdata),
        .debug_valid(debug_wb_valid), .debug_inst(debug_wb_inst),
        .debug_dnpc(debug_wb_dnpc), .debug_mem_valid(debug_wb_mem_valid),
        .debug_mem_store(debug_wb_mem_store), .debug_mem_addr(debug_wb_mem_addr),
        .debug_mem_mask(debug_wb_mem_mask), .debug_mem_wdata(debug_wb_mem_wdata),
        .debug_mem_rdata(debug_wb_mem_rdata),
        .debug_trap_valid(debug_wb_trap_valid),
        .debug_trap_cause(debug_wb_trap_cause)
    );

    // slot0是精确异常边界：ecall、mret和CSR写只从这里向CSR寄存器堆提交。
    // 这样即使双槽同拍退休，软件可见的控制状态也仍按程序顺序更新。

    //============================================================
    // slot1: younger ordinary writeback and retirement
    //============================================================

    // slot1的这些信号平时像“封存的报警线”：
    // wb_ex1/mret1/csr_we1如果被拉高，说明年轻槽混进了异常、mret或CSR写，这是基础双发不允许的。
    // csr_pc1/cause1/tval1/csr_addr1/csr_wdata1是slot1对应的CSR/异常字段，只拿来让lint知道没有悬空。
    // 真正会改CSR、改PC、触发精确异常的，只允许slot0做；slot1只承担普通写回和退休展示。
    wire wb_ex1, mret1, csr_we1;
    wire [31:0] csr_pc1, cause1, tval1, csr_wdata1;
    wire [11:0] csr_addr1;
    // slot1经过ID白名单过滤，只允许普通指令进入。这里仍完整实例化wb_lane，
    // 方便调试接口保持slot0/slot1字段对称，同时用unused_serial1兜住不应出现的控制字段。
    wb_lane u_lane1(
        .bus(wb_reg1), .valid(wb_valid1), .csr_rvalue(32'b0),
        .wb_bus(wb_to_id_bus1), .wb_ex(wb_ex1),
        .wb_pc_for_csr(csr_pc1), .wb_cause(cause1), .wb_tval(tval1),
        .mret_flush(mret1), .wb_csr_we(csr_we1),
        .wb_csr_addr(csr_addr1), .wb_csr_wdata(csr_wdata1),
        .debug_pc(debug_wb_pc1), .debug_rf_we(debug_wb_rf_we1),
        .debug_rf_rd(debug_wb_rf_wnum1), .debug_rf_data(debug_wb_rf_wdata1),
        .debug_valid(debug_wb_valid1), .debug_inst(debug_wb_inst1),
        .debug_dnpc(debug_wb_dnpc1), .debug_mem_valid(debug_wb_mem_valid1),
        .debug_mem_store(debug_wb_mem_store1), .debug_mem_addr(debug_wb_mem_addr1),
        .debug_mem_mask(debug_wb_mem_mask1), .debug_mem_wdata(debug_wb_mem_wdata1),
        .debug_mem_rdata(debug_wb_mem_rdata1),
        .debug_trap_valid(debug_wb_trap_valid1),
        .debug_trap_cause(debug_wb_trap_cause1)
    );

    // 基础白名单禁止CSR/Trap/mret进入slot1；保留这些归约用于lint和后续精确状态阶段。
    wire unused_serial1 = &{1'b0, wb_ex1, mret1, csr_we1, csr_pc1,
                            cause1, tval1, csr_addr1, csr_wdata1};
endmodule

//============================================================
// local writeback slot
//============================================================

// 本文件私有辅助模块：wb_lane只由wb_stage实例化，负责单槽WB字段解包。
`include "mycpu_top.h"

// [阶段4新增] 单槽WB组合通路。架构状态仍由外层按slot0、slot1年龄排列；
// CSR、ecall和mret在基础双发白名单中独占，因此只会出现在slot0。
module wb_lane(
    input  wire [`MEM_TO_WB_BUS_WIDTH-1:0] bus,
    input  wire        valid,
    input  wire [31:0] csr_rvalue,
    output wire [`WB_TO_ID_BUS_WIDTH-1:0] wb_bus,
    output wire        wb_ex,
    output wire [31:0] wb_pc_for_csr,
    output wire [31:0] wb_cause,
    output wire [31:0] wb_tval,
    output wire        mret_flush,
    output wire        wb_csr_we,
    output wire [11:0] wb_csr_addr,
    output wire [31:0] wb_csr_wdata,
    output wire [31:0] debug_pc,
    output wire [3:0]  debug_rf_we,
    output wire [4:0]  debug_rf_rd,
    output wire [31:0] debug_rf_data,
    output wire        debug_valid,
    output wire [31:0] debug_inst,
    output wire [31:0] debug_dnpc,
    output wire        debug_mem_valid,
    output wire        debug_mem_store,
    output wire [31:0] debug_mem_addr,
    output wire [3:0]  debug_mem_mask,
    output wire [31:0] debug_mem_wdata,
    output wire [31:0] debug_mem_rdata,
    output wire        debug_trap_valid,
    output wire [31:0] debug_trap_cause
);
    //============================================================
    // MEM/WB bus unpack
    //============================================================
    // 输入总线已经在mem_lane按统一顺序打包。这里先集中解包，后面每组输出
    // 都只引用这些本地名字，避免在调试信号里反复写固定切片。
    // pc/inst/dnpc/result分别是退休PC、原始指令、顺序/跳转后的下一PC、普通计算结果。
    // reg_we/reg_rd说明这条指令是否写通用寄存器以及写哪个rd。
    // ecall/mret/csr_we_bus/csr_addr/csr_wdata是WB真正提交CSR和精确异常时要看的控制资料。
    // is_csr表示写回rd的数据不是result，而是CSR旧值csr_rvalue。
    // mem_valid/mem_store/mem_addr/mem_mask/mem_wdata/mem_rdata是交给NEMU对拍的访存记录。
    wire [31:0] pc, inst, dnpc, result;
    wire reg_we;
    wire [4:0] reg_rd;
    wire ecall, mret, csr_we_bus;
    wire [11:0] csr_addr;
    wire [31:0] csr_wdata;
    wire is_csr, mem_valid, mem_store;
    wire [31:0] mem_addr;
    wire [3:0] mem_mask;
    wire [31:0] mem_wdata, mem_rdata;

    assign {
        pc, inst, dnpc, result, reg_we, reg_rd,
        ecall, mret, csr_we_bus, csr_addr, csr_wdata, is_csr,
        mem_valid, mem_store, mem_addr, mem_mask, mem_wdata, mem_rdata
    } = bus;

    //============================================================
    // register-file writeback and CSR/trap control
    //============================================================
    // CSR指令写回rd的是“CSR旧值”，普通指令写回MEM传来的result。
    // ecall只触发异常，不再同时提交CSR写；mret只产生流水线冲刷返回mepc。
    // rf_data是最终送到regfile写口的数据；wb_bus再把valid、写使能、rd和值打包回ISSUE用于前递。
    // wb_ex/wb_cause/wb_tval描述精确异常；mret_flush告诉前端从mepc重新取指。
    // wb_csr_we/wb_csr_addr/wb_csr_wdata是CSR寄存器堆的提交口，只在valid且没有ecall异常时生效。
    wire [31:0] rf_data = is_csr ? csr_rvalue : result;
    assign wb_bus = {valid, reg_we && valid, reg_rd, rf_data};

    assign wb_ex = valid && ecall;
    assign wb_pc_for_csr = pc;
    assign wb_cause = ecall ? 32'd11 : 32'b0;
    assign wb_tval = 32'b0;
    assign mret_flush = valid && mret;
    assign wb_csr_we = valid && csr_we_bus && !wb_ex;
    assign wb_csr_addr = csr_addr;
    assign wb_csr_wdata = csr_wdata;

    //============================================================
    // retirement/debug output
    //============================================================
    // 顶层退休接口直接使用这些字段与NEMU逐条比对。valid为0时payload可为旧值，
    // 外部只在debug_valid有效时采样，因此不额外清零数据字段。
    assign debug_pc = pc;
    assign debug_rf_we = {4{valid && reg_we}};
    assign debug_rf_rd = reg_rd;
    assign debug_rf_data = rf_data;
    assign debug_valid = valid;
    assign debug_inst = inst;
    assign debug_dnpc = dnpc;
    assign debug_mem_valid = valid && mem_valid;
    assign debug_mem_store = mem_store;
    assign debug_mem_addr = mem_addr;
    assign debug_mem_mask = mem_mask;
    assign debug_mem_wdata = mem_wdata;
    assign debug_mem_rdata = mem_rdata;
    assign debug_trap_valid = wb_ex;
    assign debug_trap_cause = wb_cause;
endmodule
