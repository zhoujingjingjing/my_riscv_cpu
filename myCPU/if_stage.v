`include "mycpu_top.h"
`timescale 1ns / 1ps

module if_stage (
    input  wire        clk,
    input  wire        reset,
   
    output wire [`IF_TO_ID_BUS_WIDTH-1:0] if_to_id_bus,
    input  wire [`EXE_TO_IF_BUS_WIDTH-1:0] exe_to_if_bus,
    input wire [`ID_TO_IF_BUS_WIDTH-1:0] id_to_if_bus,

    output wire if_to_id_valid,
    input  wire id_allow_in, 

    output wire        inst_sram_en,
    output wire  [3:0] inst_sram_we,//写使能改为4位
    output wire [31:0] inst_sram_addr,
    output wire [31:0] inst_sram_wdata,
    input  wire [31:0] inst_sram_rdata
);


// BTB 预测模块实例化 
btb # (
    .INDEX_LEN(6),
    .TAG_LEN(15)
  )
  u_btb (
    .clk(clk),
    .reset(reset),
    .if_pc(if_pc),

    .pre_taken(pre_taken),
    .pre_target(pre_target),
    .pre_is_ret(pre_is_ret),
    .pre_index(pre_index),

    .id_push_ras(id_push_ras),
    .id_ras_wdata(id_ras_wdata),
    .id_pop_ras(id_pop_ras),

    .exe_we(exe_we),
    .exe_index(exe_index),
    .exe_tag(exe_tag),
    .exe_is_ret(exe_is_ret),
    .exe_taken(exe_taken),
    .exe_target(exe_target)
  );


// output bus to ID stage
    reg  [31:0] if_pc;
    wire [31:0] inst;

    wire        pre_taken;
    wire [31:0] pre_target;
    wire [5:0]  pre_index;
    wire        pre_is_ret;
    assign if_to_id_bus = {if_pc, inst, pre_taken, pre_target, pre_index};
    //位宽: 32 + 32 + 1 + 32 + 6 = 103

    // input bus from EXE (for branch flush and BTB update)
    wire        flush_en;
    wire        exe_we;  
    wire [5:0]  exe_index; 
    wire [14:0] exe_tag;
    wire        exe_taken; 
    wire [31:0] exe_target;   
    wire        exe_is_ret;   
      
    assign {flush_en, exe_target, exe_we, exe_index, exe_tag, exe_taken, exe_is_ret} = exe_to_if_bus;

    // [新增] input bus from ID (for RAS maintenance)
    // 请确保 if_stage.v 的模块端口列表里有 : input wire [`ID_TO_IF_BUS_WIDTH-1:0] id_to_if_bus
    wire        id_push_ras;
    wire        id_pop_ras;
    wire [31:0] id_ras_wdata;
    assign {id_push_ras, id_pop_ras, id_ras_wdata} = id_to_if_bus;

    /*......pipeline control.......*/


    wire [31:0] seq_pc;
    wire [31:0] nextpc;
    assign seq_pc       = if_pc + 32'h4;
    assign nextpc = flush_en  ? exe_target : 
                    pre_taken ? pre_target : 
                                seq_pc;//这段代码什么意思？
   

    wire if_allow_in;
    wire if_ready_go;
    reg if_valid;

    assign if_to_id_valid = if_valid && if_ready_go && !flush_en;
    assign if_allow_in = !if_valid || (if_ready_go && id_allow_in)|| flush_en;//这里加flush_en干什么？
    assign if_ready_go   = 1'b1;

    always @(posedge clk) begin
        if (reset) begin
            if_valid <= 1'b0;
         end else if (flush_en) begin
            if_valid <= 1'b0;  
        end else if (if_allow_in) begin                
            if_valid <= ~reset;
        end
    end

    always@ (posedge clk) begin
        if (reset) begin
            if_pc <= 32'h7FFFFFFC;
        end else if ( if_allow_in) begin
            if_pc <= nextpc;
        end
    end
    
    assign inst_sram_we    = 4'b0;
    assign inst_sram_addr  = nextpc;//更新的是nextpc对应的指令，与pc寄存器对应
    assign inst_sram_wdata = 32'b0;
    assign inst_sram_en    = ~reset && if_allow_in;
    assign inst            = inst_sram_rdata;//i bram也相当于一个reg,所以使能条件与pc这个reg是一样的

//    // output bus to ID stage
//     reg  [31:0] if_pc;
//     wire [31:0] inst;
//     assign if_to_id_bus = {if_pc, inst};//位宽: 32 + 32 = 64

//     // input bus from ID (for branch)
//     wire        br_taken;
//     wire [31:0] br_target;
//     assign {br_taken, br_target} = id_to_if_bus;

//     /*......pipeline control.......*/

//     //pre_if stage
//     wire [31:0] seq_pc;
//     wire [31:0] nextpc;
//     wire pre_if_valid;
//     assign seq_pc       = if_pc + 32'h4;
//     assign nextpc       = br_taken ? br_target : seq_pc;
//     assign pre_if_valid = ~reset; 
//     wire pre_if_to_if_valid = pre_if_valid; //pre_if_to_if_valid = pre_if_valid


//     //if stage
//     wire if_allow_in;
//     wire if_ready_go;
//     reg if_valid;

//     assign if_allow_in = !if_valid || (if_ready_go && id_allow_in);
//     assign if_ready_go   = 1'b1;

//     always @(posedge clk) begin
//         if (reset) begin
//             if_valid <= 1'b0;
//         end else if (if_allow_in) begin                
//             if_valid <= pre_if_to_if_valid;
//         end

//     end

   
//     assign if_to_id_valid = if_valid && if_ready_go;
//     always@ (posedge clk) begin
//         if (reset) begin
//             if_pc <= 32'h1bfffffc;
//         end else if (pre_if_to_if_valid && if_allow_in) begin
//             if_pc <= nextpc;//相当于其他阶段的reg
//         end
//     end
    
//     assign inst_sram_we    = 4'b0;
//     assign inst_sram_addr  = nextpc;//更新的是nextpc对应的指令，与pc寄存器对应
//     assign inst_sram_wdata = 32'b0;
//     assign inst_sram_en    = pre_if_to_if_valid && if_allow_in;
//     assign inst            = inst_sram_rdata;//i bram也相当于一个reg,所以使能条件与pc这个reg是一样的

       
   
endmodule