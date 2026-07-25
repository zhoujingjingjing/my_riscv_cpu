`include "mycpu_top.h"

module mycpu_top #(
    parameter [31:0] RESET_PC = 32'h8000_0000
)(
    input  wire        clk,
    input  wire        resetn,
    // inst sram interface
    output wire        inst_sram_en,
    output wire [3:0]  inst_sram_we,
    output wire [31:0] inst_sram_addr,
    output wire [31:0] inst_sram_wdata,
    input  wire [31:0] inst_sram_rdata,
    // data sram interface
    output wire        data_sram_en,
    output wire [3:0]  data_sram_we,
    output wire [31:0] data_sram_addr,
    output wire [31:0] data_sram_wdata,
    input  wire [31:0] data_sram_rdata,
    // trace debug interface
    output wire [31:0] debug_wb_pc,
    output wire [ 3:0] debug_wb_rf_we,//写使能为什么4位？
    output wire [ 4:0] debug_wb_rf_wnum,//写了哪个寄存器
    output wire [31:0] debug_wb_rf_wdata,//写了什么数据
    output wire        debug_wb_valid,
    // Retirement Interface V1。单发射时retire_count只有0或1。
    output wire [31:0] retire_if_version,
    output wire [7:0]  retire_count,
    output reg  [31:0] retire_pc,
    output reg  [31:0] retire_inst,
    output reg  [31:0] retire_dnpc,
    output reg         retire_rd_we,
    output reg  [4:0]  retire_rd,
    output reg  [31:0] retire_rd_data,
    output wire [1023:0] retire_gpr_flat,
    output reg         retire_mem_valid,
    output reg         retire_mem_store,
    output reg  [31:0] retire_mem_addr,
    output reg  [3:0]  retire_mem_mask,
    output reg  [31:0] retire_mem_wdata,
    output reg  [31:0] retire_mem_rdata,
    output reg         retire_trap_valid,
    output reg         retire_trap_interrupt,
    output reg  [31:0] retire_trap_cause,
    output reg         retire_skip_ref,

    // [测试框架完善] Performance Interface V1。全部导出原始累计值，
    // 百分比和CPI由测试框架计算，避免RTL中加入除法器。
    output wire [31:0] perf_if_version,
    output wire [63:0] perf_mcycle,
    output wire [63:0] perf_minstret,
    output wire [63:0] perf_load_use_stall_count,
    output wire [63:0] perf_branch_count,
    output wire [63:0] perf_mispredict_count
);

    reg reset;
    always @(posedge clk) reset <= ~resetn;

/*.............信号定义..........................*/

    wire if_to_id_valid;
    wire [`IF_TO_ID_BUS_WIDTH-1:0] if_to_id_bus;

    wire [`ID_TO_IF_BUS_WIDTH-1:0] id_to_if_bus;

    wire id_to_exe_valid;
    wire id_allow_in;
    wire [`ID_TO_EXE_BUS_WIDTH-1:0] id_to_exe_bus;
    wire [`EXE_TO_IF_BUS_WIDTH-1:0] exe_to_if_bus;

    wire exe_to_mem_valid;
    wire exe_allow_in;
    wire [`EXE_TO_MEM_BUS_WIDTH-1:0] exe_to_mem_bus;
    wire [`EXE_TO_ID_BYPASS_BUS_WIDTH-1:0] exe_to_id_bypass_bus;

    wire mem_to_wb_valid;
    wire mem_to_wb_valid_raw;
    wire mem_allow_in;
    wire [`MEM_TO_WB_BUS_WIDTH-1:0] mem_to_wb_bus;
    wire [`MEM_TO_ID_BYPASS_BUS_WIDTH-1:0] mem_to_id_bypass_bus;

    wire wb_allow_in;
    wire [`WB_TO_ID_BUS_WIDTH-1:0] wb_to_id_bus;


    // ---- [新增] WB→CSR 接口信号 ----
    wire        wb_ex;          // WB级有异常触发
    wire [31:0] wb_pc_for_csr; // WB级PC（供CSR保存mepc）
    wire [31:0] wb_cause;       // 异常原因
    wire [31:0] wb_tval;        // 异常附加信息
    wire        mret_flush;     // mret在WB级执行
    wire        wb_csr_we;      // WB级CSR写使能
    wire [11:0] wb_csr_addr;   // WB级CSR地址
    wire [31:0] wb_csr_wdata;  // WB级CSR写入值
 
    // ---- [新增] CSR→流水线 接口信号 ----
    wire [31:0] ex_entry;       // 异常入口（→IF级nextpc）
    wire [31:0] csr_mepc_out;   // mepc（→IF级nextpc，供mret用）
    // [第4步验证修复] CSR提供EXE/WB两个独立异步读口。
    wire [11:0] exe_csr_raddr;
    wire [31:0] exe_csr_rvalue;
    wire [31:0] wb_csr_rvalue;
    wire [1023:0] debug_gpr_flat;
    wire [31:0] debug_wb_inst;
    wire [31:0] debug_wb_dnpc;
    wire debug_wb_mem_valid;
    wire debug_wb_mem_store;
    wire [31:0] debug_wb_mem_addr;
    wire [3:0] debug_wb_mem_mask;
    wire [31:0] debug_wb_mem_wdata;
    wire [31:0] debug_wb_mem_rdata;
    wire debug_wb_trap_valid;
    wire [31:0] debug_wb_trap_cause;
    // [测试框架完善] 从ID/EXE级导出的单周期性能事件。
    wire perf_load_use_stall_event;
    wire perf_branch_event;
    wire perf_branch_mispredict_event;



/*.............模块实例化..........................*/
  if_stage #(
      .RESET_PC(RESET_PC)
  ) if_stage (
      .clk(clk),
      .reset(reset),
      .if_to_id_bus(if_to_id_bus),
      // [新增] 接收来自 ID 的 RAS 维护信号 (push/pop/wdata)
      .id_to_if_bus(id_to_if_bus),
      .exe_to_if_bus(exe_to_if_bus),
      .if_to_id_valid(if_to_id_valid),
      .id_allow_in(id_allow_in),
      // [新增] 异常/mret时的流水线冲刷和跳转目标
      .wb_ex(wb_ex),
      .ex_entry(ex_entry),
      .mret_flush(mret_flush),
      .csr_mepc_out(csr_mepc_out),

      .inst_sram_en(inst_sram_en),
      .inst_sram_we(inst_sram_we),
      .inst_sram_addr(inst_sram_addr),
      .inst_sram_wdata(inst_sram_wdata),
      .inst_sram_rdata(inst_sram_rdata)
    );

  id_stage  id_stage (
      .clk(clk),
      .reset(reset),
      .id_to_exe_bus(id_to_exe_bus),
      // [新增] 输出给 IF 的 RAS 维护信号
      .id_to_if_bus(id_to_if_bus),
      .if_to_id_bus(if_to_id_bus),
      .wb_to_id_bus(wb_to_id_bus),
      .exe_to_id_bypass_bus(exe_to_id_bypass_bus),
      .mem_to_id_bypass_bus(mem_to_id_bypass_bus),
      .id_allow_in(id_allow_in),
      .id_to_exe_valid(id_to_exe_valid),
      .exe_allow_in(exe_allow_in),
      .if_to_id_valid(if_to_id_valid),
      // [新增] CSR冲突阻塞：WB级告知ID级有CSR写指令或mret在执行
      .wb_ex(wb_ex),
      .mret_flush(mret_flush)
      ,.debug_gpr_flat(debug_gpr_flat),
      .perf_load_use_stall(perf_load_use_stall_event)
    );

  exe_stage  exe_stage (
      .clk(clk),
      .reset(reset),
      .id_to_exe_bus(id_to_exe_bus),
      .exe_to_mem_bus(exe_to_mem_bus),
      .exe_to_if_bus(exe_to_if_bus),
      .exe_to_id_bypass_bus(exe_to_id_bypass_bus),
      .id_to_exe_valid(id_to_exe_valid),
      .mem_allow_in(mem_allow_in),
      .exe_allow_in(exe_allow_in),
      .exe_to_mem_valid(exe_to_mem_valid),
      // [RV32M移植] 取消被更老异常/mret冲刷的年轻M指令。
      .wb_ex(wb_ex),
      .mret_flush(mret_flush),
      // [第4步验证修复] EXE专用CSR读口。
      .csr_raddr(exe_csr_raddr),
      .csr_rvalue(exe_csr_rvalue),
      .data_sram_en(data_sram_en),
      .data_sram_we(data_sram_we),
      .data_sram_addr(data_sram_addr),
      .data_sram_wdata(data_sram_wdata),
      .perf_branch(perf_branch_event),
      .perf_branch_mispredict(perf_branch_mispredict_event)
    );
 
  mem_stage  mem_stage (
    .clk(clk),
    .reset(reset),
    .exe_to_mem_bus(exe_to_mem_bus),
    .mem_to_wb_bus(mem_to_wb_bus),
    .mem_to_id_bypass_bus(mem_to_id_bypass_bus),
    .exe_to_mem_valid(exe_to_mem_valid),
    .wb_allow_in(wb_allow_in),
    .mem_allow_in(mem_allow_in),
    .mem_to_wb_valid(mem_to_wb_valid_raw),
    .data_sram_rdata(data_sram_rdata)
  );

  // [RV32M移植] ecall/mret在WB确定冲刷时，MEM里仍可能有一条更年轻指令。
  // 必须阻止它进入WB，否则EXE里的M指令即使已cancel，前一条年轻指令仍会错误退休。
  // 这里只过滤valid，不改原MEM→WB数据总线，也不创建新的退休通路。
  assign mem_to_wb_valid = mem_to_wb_valid_raw && !wb_ex && !mret_flush;

  wb_stage  wb_stage (
    .clk(clk),
    .reset(reset),
    .mem_to_wb_bus(mem_to_wb_bus),
    .wb_to_id_bus(wb_to_id_bus),
    .mem_to_wb_valid(mem_to_wb_valid),
    .wb_allow_in(wb_allow_in),
    // [新增] WB→CSR 接口
    .wb_ex(wb_ex),
    .wb_pc_for_csr(wb_pc_for_csr),
    .wb_cause(wb_cause),
    .wb_tval(wb_tval),
    .mret_flush(mret_flush),
    .wb_csr_we(wb_csr_we),
    .wb_csr_addr(wb_csr_addr),
    .wb_csr_wdata(wb_csr_wdata),
    // [新增] CSR读值（供WB级CSR指令返回值写回regfile）
    .csr_rvalue(wb_csr_rvalue),
    .debug_wb_pc(debug_wb_pc),
    .debug_wb_rf_we(debug_wb_rf_we),
    .debug_wb_rf_wnum(debug_wb_rf_wnum),
    .debug_wb_rf_wdata(debug_wb_rf_wdata),
    .debug_wb_valid(debug_wb_valid),
    .debug_wb_inst(debug_wb_inst),
    .debug_wb_dnpc(debug_wb_dnpc),
    .debug_wb_mem_valid(debug_wb_mem_valid),
    .debug_wb_mem_store(debug_wb_mem_store),
    .debug_wb_mem_addr(debug_wb_mem_addr),
    .debug_wb_mem_mask(debug_wb_mem_mask),
    .debug_wb_mem_wdata(debug_wb_mem_wdata),
    .debug_wb_mem_rdata(debug_wb_mem_rdata),
    .debug_wb_trap_valid(debug_wb_trap_valid),
    .debug_wb_trap_cause(debug_wb_trap_cause)
  );
 
  // ---- [新增] CSR寄存器堆 ----
  csr_regfile  u_csr_regfile (
    .clk          (clk           ),
    .reset        (reset         ),
    // 指令访问接口（WB级）
    .csr_addr     (wb_csr_addr   ),
    .csr_we       (wb_csr_we     ),
    .csr_wvalue   (wb_csr_wdata  ),
    .csr_rvalue   (wb_csr_rvalue ),
    // [第4步验证修复] EXE读口只读，不参与WB写地址选择。
    .exe_csr_addr (exe_csr_raddr ),
    .exe_csr_rvalue(exe_csr_rvalue),
    // 异常触发接口
    .wb_ex        (wb_ex         ),
    .wb_pc        (wb_pc_for_csr ),
    .wb_cause     (wb_cause      ),
    .wb_tval      (wb_tval       ),
    // mret接口
    .mret_flush   (mret_flush    ),
    // [测试框架完善] WB有效表示本拍真正退休；EXE事件还要排除同拍
    // 被更老异常/mret冲刷的年轻指令。
    .retire_count ({7'b0, debug_wb_valid}),
    .load_use_stall_event(perf_load_use_stall_event),
    .branch_event(perf_branch_event && !wb_ex && !mret_flush),
    .branch_mispredict_event(perf_branch_mispredict_event && !wb_ex && !mret_flush),
    // 输出到流水线
    .ex_entry     (ex_entry      ),
    .csr_mepc_out (csr_mepc_out  ),
    .perf_mcycle(perf_mcycle),
    .perf_minstret(perf_minstret),
    .perf_load_use_stall_count(perf_load_use_stall_count),
    .perf_branch_count(perf_branch_count),
    .perf_mispredict_count(perf_mispredict_count),
    // Hart ID
    .coreid_in    (32'b0         )   // 单核，Hart ID = 0
  );

  assign retire_if_version = 32'd1;
  // [测试框架完善] 性能接口版本与退休接口分开演进。
  assign perf_if_version = 32'd1;
  assign retire_count = {7'b0, retire_valid};
  assign retire_gpr_flat = debug_gpr_flat;
  reg retire_valid;

  // 在这个CPU中，寄存器堆写入发生在WB信息出现后的时钟沿。
  // 所以把WB元数据寄存一拍；时钟沿后，元数据与已经更新的GPR快照属于同一条指令。
  always @(posedge clk) begin
    if (reset) begin
      retire_valid <= 1'b0;
      retire_pc <= 32'b0;
      retire_inst <= 32'b0;
      retire_dnpc <= 32'b0;
      retire_rd_we <= 1'b0;
      retire_rd <= 5'b0;
      retire_rd_data <= 32'b0;
      retire_mem_valid <= 1'b0;
      retire_mem_store <= 1'b0;
      retire_mem_addr <= 32'b0;
      retire_mem_mask <= 4'b0;
      retire_mem_wdata <= 32'b0;
      retire_mem_rdata <= 32'b0;
      retire_trap_valid <= 1'b0;
      retire_trap_interrupt <= 1'b0;
      retire_trap_cause <= 32'b0;
      retire_skip_ref <= 1'b0;
    end else begin
      retire_valid <= debug_wb_valid;
      retire_pc <= debug_wb_pc;
      retire_inst <= debug_wb_inst;
      retire_dnpc <= debug_wb_trap_valid ? ex_entry :
                     mret_flush ? csr_mepc_out : debug_wb_dnpc;
      retire_rd_we <= (|debug_wb_rf_we) && (debug_wb_rf_wnum != 5'b0);
      retire_rd <= debug_wb_rf_wnum;
      retire_rd_data <= debug_wb_rf_wdata;
      retire_mem_valid <= debug_wb_mem_valid;
      retire_mem_store <= debug_wb_mem_store;
      retire_mem_addr <= debug_wb_mem_addr;
      retire_mem_mask <= debug_wb_mem_mask;
      retire_mem_wdata <= debug_wb_mem_wdata;
      retire_mem_rdata <= debug_wb_mem_rdata;
      retire_trap_valid <= debug_wb_trap_valid;
      retire_trap_interrupt <= debug_wb_trap_cause[31];
      retire_trap_cause <= debug_wb_trap_cause;
      retire_skip_ref <= debug_wb_mem_valid &&
                         (debug_wb_mem_addr >= 32'h8020_0000);
    end
  end

endmodule
