`include "mycpu_top.h"
module exe_stage(
    input  wire                 clk,
    input  wire                 reset,
    
    input  wire [`ID_TO_EXE_BUS_WIDTH-1:0] id_to_exe_bus,              
    output wire [`EXE_TO_MEM_BUS_WIDTH-1:0] exe_to_mem_bus,
    output wire [`EXE_TO_ID_BYPASS_BUS_WIDTH-1:0] exe_to_id_bypass_bus, //bus to ID stage for bypass

    input wire id_to_exe_valid, 
    input wire mem_allow_in, 
    output wire exe_allow_in, 
    output wire exe_to_mem_valid,

         
    output wire       data_sram_en,
    output wire [3:0] data_sram_we,
    output wire [31:0] data_sram_addr,
    output wire [31:0] data_sram_wdata


);
    
   //pipeline control signal
    reg         exe_valid; //ex valid flag
    wire        exe_ready_go; //ex stage ready to accept new input

    assign exe_ready_go    = 1'b1;
    assign exe_allow_in    = !exe_valid||( exe_ready_go && mem_allow_in) ;
    assign exe_to_mem_valid   = exe_valid && exe_ready_go;   
    
    always@(posedge clk) begin
        if(reset) begin
            exe_valid <= 1'b0;
        end
        else if(exe_allow_in) begin
            exe_valid <= id_to_exe_valid;
        end
    end

    //input bus from id stage
    wire [31:0] exe_pc;
    wire [11:0] exe_alu_op;
    wire        exe_src1_is_pc;
    wire        exe_src2_is_imm;
    wire        exe_res_from_mem;
    wire        exe_mem_en;
    wire [3:0]  exe_mem_we;
    wire        exe_reg_we;
    wire [4:0]  exe_reg_waddr;
    wire [31:0] exe_rs1_value;
    wire [31:0] exe_rs2_value;
    wire [31:0] exe_imm;

    // 新增：取出总线上的具体访存类型信号
    wire        inst_lb;
    wire        inst_lh;
    wire        inst_lw;
    wire        inst_lbu;
    wire        inst_lhu;
    wire        inst_sb;
    wire        inst_sh;
    wire        inst_sw;

    reg [`ID_TO_EXE_BUS_WIDTH-1:0] exe_reg;

    always @(posedge clk) begin
        if(id_to_exe_valid && exe_allow_in) begin
            exe_reg <= id_to_exe_bus;
        end
    end 

        assign {
            exe_pc,         //32
            exe_alu_op,     //12
            exe_src1_is_pc, //1
            exe_src2_is_imm,//1
            exe_res_from_mem,//1
            exe_mem_en,     //1
            exe_mem_we,     //4
            exe_reg_we,     //1
            exe_reg_waddr,  //5
            exe_rs1_value,   //32
            exe_rs2_value,  //32
            exe_imm,         //32
            inst_lb,         //1
            inst_lh,         //1
            inst_lw,         //1
            inst_lbu,        //1
            inst_lhu,        //1
            inst_sb,         //1
            inst_sh,         //1
            inst_sw          //1
        } = exe_reg;
    
    //output bus to mem stage    
    wire [31:0] alu_result;
    assign exe_to_mem_bus = {
            exe_pc,
            alu_result,//并不因为提前一拍绕过ID/EX寄存器就不需要传递了，因为后续的写回阶段还需要这个结果，并不是读内存用的
            exe_res_from_mem,    
            exe_reg_we,     
            exe_reg_waddr,  
            // 修改：只需要将 load 信号向后传，store不需要
            inst_lb, inst_lh, inst_lw, inst_lbu, inst_lhu
    };
    //位宽是32+32+1+1+5+5=76

    //output bus to id stage for bypass
    wire exe_is_load = exe_mem_en && (exe_mem_we == 4'b0000);//判断exe阶段的这条指令是不是ld.w指令，特征是使能内存但不写内存（exe_mem_en 区分访存指令和其他指令，exe_mem_we==4'b0000是为了区分ld.w和st.w）
    assign exe_to_id_bypass_bus = {
        exe_valid, 
        exe_reg_we, 
        exe_reg_waddr, 
        alu_result,
        exe_is_load 
    };
    //位宽是1+1+5+32+1=40



  /*..................internal signals................*/
    wire [31:0] alu_src1;
    wire [31:0] alu_src2;
    wire [11:0] alu_op;
    assign alu_src1 = exe_src1_is_pc  ? exe_pc : exe_rs1_value;
    assign alu_src2 = exe_src2_is_imm ? exe_imm : exe_rs2_value;
    assign alu_op = exe_alu_op;

    alu u_alu(
        .alu_op     (alu_op    ),
        .alu_src1   (alu_src1  ),
        .alu_src2   (alu_src2  ),
        .alu_result (alu_result)
        );

    // 修改：数据存储器 根据指令要求完成写掩码和移位
    wire [3:0] st_data_byte_en;
    wire [31:0] st_data;

    // 根据你提供的写法，利用移位简捷生成写入数据和写使能掩码
    assign st_data = inst_sb ? {4{exe_rs2_value[7:0]}}:
                     inst_sh ? {2{exe_rs2_value[15:0]}}:
                               exe_rs2_value;

    assign st_data_byte_en = inst_sw ? 4'b1111:
                             inst_sh ? 4'b0011 << alu_result[1:0]:
                             inst_sb ? 4'b0001 << alu_result[1:0]:
                             4'b0000;


    // 数据存储器 因为data_sram是同步读，提前一拍绕过ID/EX寄存器，直接从ID阶段拿到控制信号和数据，减少一个周期的访问延迟
    assign data_sram_en    = exe_mem_en && exe_valid;
    // 取 exe_mem_we[0] 是因为在ID阶段我们置了 {4{op_STORE}}，这里再按实际需要移位,1111 & 0010 = 0010 就是只写第二个字节
    assign data_sram_we    = {4{exe_mem_we[0]}} & st_data_byte_en & {4{exe_valid}};/*为什么别的控制信号不需要vaild，它需要?
                                    只有带永久记忆特性的写操作和改写流水线方向的操作，才配带上 valid 安全锁。
                                （都跟时序逻辑器件有关，pc寄存器、数据存储器、寄存器堆，像alu这样的组合逻辑器件就不需要）
                                虽然data_sram_en 带了 exe_valid 来保证指令无效时不启用设备。
                                但这并不严谨。当处理异常或流水线冲刷时，如果没有强管控 EN 优先于 WE，这里可能会误触发写操作，所以这里也加上exe_valid保险*/
    assign data_sram_addr  = alu_result;
    assign data_sram_wdata = st_data;


    
    endmodule
