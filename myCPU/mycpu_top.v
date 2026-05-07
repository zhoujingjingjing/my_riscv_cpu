`include "mycpu_top.h"

module mycpu_top(
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
    output wire        debug_wb_valid
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
    wire [31:0] csr_rvalue;     // CSR读出值（→WB级，供CSR指令读）



/*.............模块实例化..........................*/
  if_stage  if_stage (
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
      // [新增] CSR读值（EXE级计算csr_wdata时需要当前CSR值）
      .csr_rvalue(csr_rvalue),
      .data_sram_en(data_sram_en),
      .data_sram_we(data_sram_we),
      .data_sram_addr(data_sram_addr),
      .data_sram_wdata(data_sram_wdata)
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
    .mem_to_wb_valid(mem_to_wb_valid),
    .data_sram_rdata(data_sram_rdata)
  );
 
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
    .csr_rvalue(csr_rvalue),
    .debug_wb_pc(debug_wb_pc),
    .debug_wb_rf_we(debug_wb_rf_we),
    .debug_wb_rf_wnum(debug_wb_rf_wnum),
    .debug_wb_rf_wdata(debug_wb_rf_wdata),
    .debug_wb_valid(debug_wb_valid)
  );
 
  // ---- [新增] CSR寄存器堆 ----
  csr_regfile  u_csr_regfile (
    .clk          (clk           ),
    .reset        (reset         ),
    // 指令访问接口（WB级）
    .csr_addr     (wb_csr_addr   ),
    .csr_we       (wb_csr_we     ),
    .csr_wvalue   (wb_csr_wdata  ),
    .csr_rvalue   (csr_rvalue    ),
    // 异常触发接口
    .wb_ex        (wb_ex         ),
    .wb_pc        (wb_pc_for_csr ),
    .wb_cause     (wb_cause      ),
    .wb_tval      (wb_tval       ),
    // mret接口
    .mret_flush   (mret_flush    ),
    // 输出到流水线
    .ex_entry     (ex_entry      ),
    .csr_mepc_out (csr_mepc_out  ),
    // Hart ID
    .coreid_in    (32'b0         )   // 单核，Hart ID = 0
  );

endmodule
