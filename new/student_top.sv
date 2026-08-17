`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/16/2025 06:21:13 PM
// Design Name: 
// Module Name: student_top
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module student_top#(
    parameter                           P_SW_CNT            = 64,
    parameter                           P_LED_CNT           = 32,
    parameter                           P_SEG_CNT           = 40,
    parameter                           P_KEY_CNT           = 8
) (
    input                                       w_cpu_clk     ,
    input                                       w_clk_50Mhz   ,
    input                                       w_clk_rst     ,
    input  [P_KEY_CNT - 1:0]                    virtual_key   ,
    input  [P_SW_CNT  - 1:0]                    virtual_sw    ,

    output [P_LED_CNT - 1:0]                    virtual_led   ,
    output [P_SEG_CNT - 1:0]                    virtual_seg   
);

    wire resetn = ~w_clk_rst; // myCPU通常低电平复位

    // IROM 
    logic        inst_en;
    logic [3:0]  inst_we;
    logic [31:0] inst_addr;
    logic [31:0] inst_wdata;
    logic [31:0] inst_rdata;
    // [阶段1新增] IROM B口读取PC+4；命名沿用现有inst_*风格。
    logic        inst_en_b;
    logic [31:0] inst_addr_b;
    logic [31:0] inst_rdata_b;

    // perip 
    logic        perip_en;
    logic [3:0]  perip_we;
    logic [31:0] perip_addr;
    logic [31:0] perip_wdata;
    logic [31:0] perip_rdata;

    //debug
    wire [31:0] debug_wb_pc;
    wire [3:0]  debug_wb_rf_we;
    wire [4:0]  debug_wb_rf_wnum;
    wire [31:0] debug_wb_rf_wdata;

    mycpu_top Core_cpu (
        .clk                (w_cpu_clk),
        .resetn             (resetn),

        // Interface to IROM (替换掉原来的异步ROM接口)
        .inst_sram_en       (inst_en),
        .inst_sram_we       (inst_we),
        .inst_sram_addr     (inst_addr),
        .inst_sram_wdata    (inst_wdata),
        .inst_sram_rdata    (inst_rdata),
        // [阶段1新增] CPU第二取指请求连接到BRAM B口。
        .inst_sram_en_b     (inst_en_b),
        .inst_sram_addr_b   (inst_addr_b),
        .inst_sram_rdata_b  (inst_rdata_b),

        // Interface to DRAM & periphera
        .data_sram_en       (perip_en),     
        .data_sram_we       (perip_we),     
        .data_sram_addr     (perip_addr),     
        .data_sram_wdata    (perip_wdata),    
        .data_sram_rdata    (perip_rdata),    
        
        // 新增：连接 Debug 端口
        .debug_wb_pc        (debug_wb_pc),
        .debug_wb_rf_we     (debug_wb_rf_we),
        .debug_wb_rf_wnum   (debug_wb_rf_wnum),
        .debug_wb_rf_wdata  (debug_wb_rf_wdata)
    );

    // 16KB = 2^12 * 32bit
    // 实例化 BRAM IP 核替换原来的 IROM
    // [阶段1新增] IP需改为True Dual Port RAM。A口读PC，B口读PC+4；
    // 两口共用w_cpu_clk，且IROM永远只读，所以B口写使能和写数据固定为0。
    inst_ram Mem_IRAM (
        .clka  (w_cpu_clk         ),
        .ena   (inst_en           ),
        .wea   (inst_we           ),
        .addra (inst_addr[13:2]   ), // 按照BRAM的Word地址截取，因为一条指令是4字节
        .dina  (inst_wdata        ),
        .douta (inst_rdata        ),
        .clkb  (w_cpu_clk         ),
        .enb   (inst_en_b         ),
        .web   (4'b0000           ),
        .addrb (inst_addr_b[13:2] ), // B口同样把字节地址转换成12位Word地址
        .dinb  (32'b0             ),
        .doutb (inst_rdata_b      )
    );
    
    perip_bridge bridge_inst (
        .clk				(w_cpu_clk),
        .cnt_clk            (w_clk_50Mhz),
        .rst                (w_clk_rst),

        .perip_en           (perip_en),    // 总线使能
        .perip_addr			(perip_addr),
        .perip_wdata		(perip_wdata),
        .perip_we			(perip_we),    // 4位宽字节掩码
        .perip_rdata		(perip_rdata),

        .virtual_sw_input	(virtual_sw),
        .virtual_key_input	(virtual_key),	
        .virtual_seg_output	(virtual_seg),
        .virtual_led_output (virtual_led)
    );

endmodule
