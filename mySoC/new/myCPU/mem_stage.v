`include "mycpu_top.h"
module mem_stage(
    input  wire                 clk,
    input  wire                 reset,
    
    input  wire [`EXE_TO_MEM_BUS_WIDTH-1:0] exe_to_mem_bus,              
    output wire [`MEM_TO_WB_BUS_WIDTH-1:0] mem_to_wb_bus,
    output wire [`MEM_TO_ID_BYPASS_BUS_WIDTH-1:0] mem_to_id_bypass_bus, //bus to ID stage for bypass

    input wire  exe_to_mem_valid,
    input wire  wb_allow_in,
    output wire mem_allow_in,
    output wire mem_to_wb_valid,

    // [新增] 异常/mret 冲刷信号：ecall/mret 在 WB 级触发时，MEM 级里更年轻的
    //        指令必须一起作废，否则会错误提交 ecall/mret 后面的指令。
    input wire  wb_ex,
    input wire  mret_flush,

    input wire [31:0] data_sram_rdata //读内存数据

);
    
    //pipeline control 
    reg         mem_valid;
    wire        mem_ready_go;

    // [新增] WB 级异常/mret 冲刷：作废 MEM 级当前指令
    wire wb_flush = wb_ex || mret_flush;

    assign mem_ready_go    = 1'b1;
    assign mem_allow_in    = !mem_valid||( mem_ready_go && wb_allow_in) ;
    assign mem_to_wb_valid   = mem_valid && mem_ready_go && !wb_flush; // [修改] 冲刷时本拍输出也作废

    always@(posedge clk) begin
        if(reset) begin
            mem_valid <= 1'b0;
        end
        else if(wb_flush) begin     // [新增] 异常/mret 时冲刷 MEM 级（优先于正常推进）
            mem_valid <= 1'b0;
        end
        else if(mem_allow_in) begin
            mem_valid <= exe_to_mem_valid;
        end
    end

    //input bus from exe stage
    wire [31:0] mem_pc;
    wire [31:0] mem_alu_result;
    wire        mem_res_from_mem;
    wire        mem_reg_we;
    wire [4:0]  mem_reg_waddr;

    // 新增：接收从 EXE 传来的 load 类型指令
    wire        inst_lb;
    wire        inst_lh;
    wire        inst_lw;
    wire        inst_lbu;
    wire        inst_lhu;

    // [新增] CSR 透传字段
    wire        mem_inst_ecall;
    wire        mem_inst_mret;
    wire        mem_csr_we;
    wire [11:0] mem_csr_addr;
    wire [31:0] mem_csr_wdata;
    wire        mem_is_csr_inst;

    reg [`EXE_TO_MEM_BUS_WIDTH-1:0] mem_reg;

    always @(posedge clk) begin
        if(exe_to_mem_valid && mem_allow_in) begin
            mem_reg <= exe_to_mem_bus;
        end
    end

    assign {
        mem_pc,
        mem_alu_result,//并不因为提前一拍绕过ID/EX寄存器就不需要传递了，因为后续的写回阶段还需要这个结果，并不是读内存用的
        mem_res_from_mem,    
        mem_reg_we,     
        mem_reg_waddr,  
        inst_lb, inst_lh, inst_lw, inst_lbu, inst_lhu,
        // [新增]
        mem_inst_ecall,
        mem_inst_mret,
        mem_csr_we,
        mem_csr_addr,
        mem_csr_wdata,
        mem_is_csr_inst
    } = mem_reg;

    //output bus to wb stage
    wire [31:0] final_result;
    assign mem_to_wb_bus = {
        mem_pc,
        final_result,
        mem_reg_we,
        mem_reg_waddr,
        // [新增] 透传给 WB
        mem_inst_ecall,
        mem_inst_mret,
        mem_csr_we,
        mem_csr_addr,
        mem_csr_wdata,
        mem_is_csr_inst

    };
   //位宽是32+32+1+5=70
    // 70 + 48 = 118

    //output bus to id stage for bypass
    assign mem_to_id_bypass_bus = {
        mem_valid, 
        mem_reg_we, 
        mem_reg_waddr, 
        final_result
    };
    //位宽是1+1+5+32=39

 /*..................internal signals................*/
    wire [31:0] mem_result;

    // 修改：根据 Load 类型对齐取出所需字节，并做对应符号/零扩展
    wire [31:0] rdata_byte_shifted = data_sram_rdata >> (mem_alu_result[1:0] * 8);
    wire [31:0] rdata_half_shifted = data_sram_rdata >> (mem_alu_result[1]   * 16);
    wire [7:0]  rdata_byte = (inst_lb | inst_lbu) ? rdata_byte_shifted[7:0]   : 8'b0;
    wire [15:0] rdata_half = (inst_lh | inst_lhu) ? rdata_half_shifted[15:0]  : 16'b0;
    /*提取阶段：利用地址的低两位 mem_alu_result[1:0]。
    如果地址低两位是 2'b01，对于按字节读（lb/lbu），1 * 8 = 8，则将整个 32 位数据右移 8 位。此时原本在 [15:8] 的有用字节被移到了 [7:0] 的位置。然后截取最低 8 位赋给 rdata_byte。
    对于半字读（lh/lhu），只看地址的第 1 位（0 或 1）。如果是 1，1 * 16 = 16，原数据右移 16 位，高半字落入低半字位置。
    扩展阶段：数据此时已经全部对齐在最低位（即 rdata_byte 和 rdata_half 中）。此时只看具体指令：lb 和 lh 进行符号扩展（复制最高位），lbu 和 lhu 进行零扩展（补 0）*/

    assign mem_result = inst_lb  ? {{24{rdata_byte[7]}}, rdata_byte}  :
                        inst_lbu ? {24'b0, rdata_byte}                :
                        inst_lh  ? {{16{rdata_half[15]}}, rdata_half} :
                        inst_lhu ? {16'b0, rdata_half}                :
                                   data_sram_rdata; // 默认或者 inst_lw
    /*为什么非搞一个mem_result来接data_sram_rdata，而是直接把data_sram_rdata连到写回寄存器的多路选择器上不行吗？
                                    assign mem_result = inst_ld_b ? { {24{data_sram_rdata[7]}}, data_sram_rdata[7:0] } :
                                                        inst_ld_h ? { {16{data_sram_rdata[15]}}, data_sram_rdata[15:0] } :
                                                        data_sram_rdata;*/
    assign final_result = mem_res_from_mem ? mem_result : mem_alu_result;



endmodule

