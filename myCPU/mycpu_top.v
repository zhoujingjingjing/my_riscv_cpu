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
    output wire [31:0] debug_wb_rf_wdata//写了什么数据
);

    reg reset;
    always @(posedge clk) reset <= ~resetn;

/*.............信号定义..........................*/

    wire if_to_id_valid;
    wire [`IF_TO_ID_BUS_WIDTH-1:0] if_to_id_bus;

    wire id_to_exe_valid;
    wire id_allow_in;
    wire [`ID_TO_EXE_BUS_WIDTH-1:0] id_to_exe_bus;
    wire [`ID_TO_IF_BUS_WIDTH-1:0] id_to_if_bus;

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

/*.............模块实例化..........................*/
    if_stage  if_stage (
    .clk(clk),
    .reset(reset),
    .if_to_id_bus(if_to_id_bus),
    .id_to_if_bus(id_to_if_bus),
    .if_to_id_valid(if_to_id_valid),
    .id_allow_in(id_allow_in),
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
    .id_to_if_bus(id_to_if_bus),
    .if_to_id_bus(if_to_id_bus),
    .wb_to_id_bus(wb_to_id_bus),
    .exe_to_id_bypass_bus(exe_to_id_bypass_bus),
    .mem_to_id_bypass_bus(mem_to_id_bypass_bus),
    .id_allow_in(id_allow_in),
    .id_to_exe_valid(id_to_exe_valid),
    .exe_allow_in(exe_allow_in),
    .if_to_id_valid(if_to_id_valid)
  );

  exe_stage  exe_stage (
    .clk(clk),
    .reset(reset),
    .id_to_exe_bus(id_to_exe_bus),
    .exe_to_mem_bus(exe_to_mem_bus),
    .exe_to_id_bypass_bus(exe_to_id_bypass_bus),
    .id_to_exe_valid(id_to_exe_valid),
    .mem_allow_in(mem_allow_in),
    .exe_allow_in(exe_allow_in),
    .exe_to_mem_valid(exe_to_mem_valid),
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
    .debug_wb_pc(debug_wb_pc),
    .debug_wb_rf_we(debug_wb_rf_we),
    .debug_wb_rf_wnum(debug_wb_rf_wnum),
    .debug_wb_rf_wdata(debug_wb_rf_wdata)
  );


endmodule
