`include "mycpu_top.h"
module wb_stage(
    input  wire                 clk,
    input  wire                 reset,
    
    input  wire [`MEM_TO_WB_BUS_WIDTH-1:0] mem_to_wb_bus,              
    output wire [`WB_TO_ID_BUS_WIDTH-1:0]  wb_to_id_bus,//不要误以为有wb_to_id_bus就需要加wb_t0_id_vaild和id_allow_in了，wb_to_id_bus本来就是写回阶段自己用的，只不过实现模块有一部分在前面的ID阶段，其实还是相当于在wb阶段内部执行
    input wire  mem_to_wb_valid, 
    output wire wb_allow_in, 

    output wire [31:0] debug_wb_pc,
    output wire [3:0]  debug_wb_rf_we,
    output wire [4:0]  debug_wb_rf_wnum,
    output wire [31:0] debug_wb_rf_wdata,
    output wire        debug_wb_valid

);
    
    //pipeline control 
    reg         wb_valid;
    wire        wb_ready_go;

    assign wb_ready_go    = 1'b1;
    assign wb_allow_in    = !wb_valid || wb_ready_go  ; 
    
    always@(posedge clk) begin
        if(reset) begin
            wb_valid <= 1'b0;
        end
        else if(wb_allow_in) begin
            wb_valid <= mem_to_wb_valid;
        end
    end

    //input bus from mem stage
    wire [31:0] wb_pc;
    wire        wb_reg_we;
    wire [4:0]  wb_reg_waddr;
    wire [31:0] wb_final_result;

    reg [`MEM_TO_WB_BUS_WIDTH-1:0] wb_reg;

    always @(posedge clk) begin
        if(mem_to_wb_valid && wb_allow_in) begin
            wb_reg <= mem_to_wb_bus;
        end
    end

    assign {
        wb_pc,
        wb_final_result,
        wb_reg_we,
        wb_reg_waddr
     } = wb_reg;


    //output bus to id stage
    assign wb_to_id_bus = {
        wb_valid,
        wb_reg_we,
        wb_reg_waddr,
        wb_final_result
    };
    //位宽1+1+5+32=39



     /*..................internal signals................*/

    // debug info generate

    assign debug_wb_pc       = wb_pc;
    assign debug_wb_rf_we   = {4{wb_reg_we & wb_valid}};//★为什么加上wb_valid？为什么debug_wb_rf_wen是4位的？32位的数据包含了 4个字节。为了方便调试，Trace测试平台要求你的CPU交代得非常详细：你到底写了这32位数据里面的哪几个字节
    assign debug_wb_rf_wnum = wb_reg_waddr;  //写了哪个寄存器(写地址)
    assign debug_wb_rf_wdata = wb_final_result;
    assign debug_wb_valid = wb_valid;


/*具体写回的操作已经在ID阶段的寄存器堆那里实现了，这里就不需要再写了，直接把要写回的数据通过总线传回ID阶段就行了

    assign {wb_rf_we, wb_rf_waddr, wb_rf_wdata} = wb_to_id_bus;//输入
        regfile u_regfile(
        .clk    (clk      ),
        .raddr1 (rf_raddr1),
        .rdata1 (rf_rdata1),
        .raddr2 (rf_raddr2),
        .rdata2 (rf_rdata2),
        .we     (rf_we    ),
        .waddr  (rf_waddr ),
        .wdata  (rf_wdata )
        );
    //写回
    assign rf_we    = wb_rf_we;//位于WB阶段的前3条指令传回来的写使能信号，用于在ID阶段写回寄存器堆。当前位于ID阶段的指令产生的写使能信号reg_we要传到下一个阶段EXE，等它自己到了WB阶段再用
    assign rf_waddr = wb_rf_waddr;//原理同rf_we
    assign rf_wdata = wb_rf_wdata;//原理同rf_we
*/


endmodule 