`include "mycpu_top.h"
module if_stage (
    input  wire        clk,
    input  wire        reset,
   
    output wire [`IF_TO_ID_BUS_WIDTH-1:0] if_to_id_bus,
    input  wire [`ID_TO_IF_BUS_WIDTH-1:0] id_to_if_bus,

    output wire if_to_id_valid,
    input  wire id_allow_in, 

    output wire        inst_sram_en,
    output wire  [3:0] inst_sram_we,//写使能改为4位
    output wire [31:0] inst_sram_addr,
    output wire [31:0] inst_sram_wdata,
    input  wire [31:0] inst_sram_rdata
);

// output bus to ID stage
    reg  [31:0] if_pc;
    wire [31:0] inst;
    assign if_to_id_bus = {if_pc, inst};//位宽: 32 + 32 = 64

    // input bus from ID (for branch)
    wire        br_taken;
    wire [31:0] br_target;
    assign {br_taken, br_target} = id_to_if_bus;

    /*......pipeline control.......*/


    wire [31:0] seq_pc;
    wire [31:0] nextpc;

    assign seq_pc       = if_pc + 32'h4;
    assign nextpc       = br_taken ? br_target : seq_pc;

    wire if_allow_in;
    wire if_ready_go;
    reg if_valid;

    assign if_to_id_valid = if_valid && if_ready_go;
    assign if_allow_in = !if_valid || (if_ready_go && id_allow_in);
    assign if_ready_go   = 1'b1;

    always @(posedge clk) begin
        if (reset) begin
            if_valid <= 1'b0;
        end else if (if_allow_in) begin                
            if_valid <= ~reset;
        end
    end

    always@ (posedge clk) begin
        if (reset) begin
            if_pc <= 32'h7FFFFFFC;
        end else if ( if_allow_in) begin
            if_pc <= nextpc;//相当于其他阶段的reg
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