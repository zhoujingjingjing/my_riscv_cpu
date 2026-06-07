`include "mycpu_top.h"
module exe_stage(
    input  wire                 clk,
    input  wire                 reset,
    
    input  wire [`ID_TO_EXE_BUS_WIDTH-1:0] id_to_exe_bus,              
    output wire [`EXE_TO_MEM_BUS_WIDTH-1:0] exe_to_mem_bus,
    output wire [`EXE_TO_IF_BUS_WIDTH-1:0] exe_to_if_bus, //bus to IF stage (for branch)
    output wire [`EXE_TO_ID_BYPASS_BUS_WIDTH-1:0] exe_to_id_bypass_bus, //bus to ID stage for bypass

    input wire id_to_exe_valid, 
    input wire mem_allow_in, 
    output wire exe_allow_in, 
    output wire exe_to_mem_valid,

    // [新增] CSR 当前读出值（组合输入，用于计算 csr_wdata）
    // [修改] 这里的 csr_rvalue 必须是"按 EXE 级自己这条 CSR 指令的地址 exe_csr_addr 读出的值"。
    //        原先 top 层把 CSR 读口接到 wb_csr_addr（WB 级指令地址），EXE 级会读到错误的
    //        CSR（WB 那条指令的地址），导致 csrrs/csrrc 在 CSR 原值非 0 时算错写入值。
    //        现新增 exe_csr_addr 输出口，由 top 用它驱动 CSR 的第二个读口，再把结果接回这里。
    output wire [11:0] exe_csr_addr_o,  // [新增] EXE 级 CSR 读地址，送往 csr_regfile 第二读口
    input  wire [31:0] csr_rvalue,      // 由 top 驱动为 CSR[exe_csr_addr_o] 的值

    // [新增] 异常/mret 冲刷信号：ecall/mret 在 WB 级触发时，EXE 级里更年轻的
    //        指令必须一起作废（否则会错误提交 ecall/mret 后面那条指令）。
    input  wire        wb_ex,
    input  wire        mret_flush,

    output wire       data_sram_en,
    output wire [3:0] data_sram_we,
    output wire [31:0] data_sram_addr,
    output wire [31:0] data_sram_wdata


);
    
   //pipeline control signal
    reg         exe_valid; //ex valid flag
    wire        exe_ready_go; //ex stage ready to accept new input

    // [新增] WB 级异常/mret 冲刷：作废 EXE 级当前指令
    wire wb_flush = wb_ex || mret_flush;

    assign exe_ready_go    = 1'b1;
    assign exe_allow_in    = !exe_valid||( exe_ready_go && mem_allow_in) ;
    assign exe_to_mem_valid   = exe_valid && exe_ready_go && !wb_flush; // [修改] 冲刷时本拍输出也作废

    always@(posedge clk) begin
        if(reset) begin
            exe_valid <= 1'b0;
        end
        else if(wb_flush) begin     // [新增] 异常/mret 时冲刷 EXE 级（优先于正常推进）
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

    wire        inst_beq;
    wire        inst_bne;
    wire        inst_blt;
    wire        inst_bge;
    wire        inst_bltu;
    wire        inst_bgeu;
    wire        inst_jal;
    wire        inst_jalr;
    wire        pre_taken;
    wire [31:0] pre_target;
    wire [5:0]  pre_index;
    wire [31:0] imm_B;
    wire [31:0] imm_J;
    wire [31:0] imm_I;
    wire        exe_is_ret_from_id;
    wire [2:0]  exe_safe_ras_ptr;

    // [新增] CSR 相关字段
    wire exe_inst_ecall, exe_inst_mret;
    wire exe_inst_csrrw, exe_inst_csrrs, exe_inst_csrrc;
    wire exe_inst_csrrwi, exe_inst_csrrsi, exe_inst_csrrci;
    wire [11:0] exe_csr_addr;
    wire [4:0]  exe_csr_uimm;
    wire        exe_is_csr_inst;


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
            inst_sw,          //1

            inst_beq,         //1
            inst_bne,         //1
            inst_blt,         //1
            inst_bge,         //1
            inst_bltu,        //1
            inst_bgeu,        //1
            inst_jal,         //1
            inst_jalr,        //1
            pre_taken,        //1
            pre_target,       //32
            pre_index,         //6
            imm_B,            //32
            imm_J,            //32
            imm_I,            //32
            exe_is_ret_from_id,   //1
            exe_safe_ras_ptr,       //3

            // [新增] CSR 字段
            exe_inst_ecall, exe_inst_mret,
            exe_inst_csrrw, exe_inst_csrrs, exe_inst_csrrc,
            exe_inst_csrrwi, exe_inst_csrrsi, exe_inst_csrrci,
            exe_csr_addr,
            exe_csr_uimm,
            exe_is_csr_inst
        } = exe_reg;
    

    //output bus to if stage 
    wire flush_en;                  // 1
    wire [31:0] exe_target;       // 32
    wire exe_we;                    // 1
    wire [14:0] exe_tag;            // 15
    wire exe_taken;                 // 1
    wire exe_is_ret;                // 1   

    assign exe_to_if_bus={
        flush_en,
        exe_target,
        exe_we,
        pre_index,
        exe_tag,
        exe_taken,
        exe_is_ret,
        exe_safe_ras_ptr
    };
    //位宽是1+32+1+6+15+1+1 +3 =60




    // ================================================================
    // [新增] CSR 写入值计算（在 EXE 级算好，通过总线传至 WB 级执行写操作）
    //
    // 之所以在 EXE 级算而不在 WB 级算：
    //   - EXE 级有 rs1_value（已过旁路），WB 级不再有 rs1_value。
    //   - csr_rvalue 在 EXE 级就能读到（CSR 是异步读），不需要等到 WB。
    //   - 这样 EXE→MEM→WB 只需透传算好的 csr_wdata，路径短，时序好。
    //
    // 三种操作：
    //   csrrw/csrrwi : csr_wdata = rs1_value（或 uimm），全量写入
    //   csrrs/csrrsi : csr_wdata = csr_rvalue | rs1_value（置位）
    //   csrrc/csrrci : csr_wdata = csr_rvalue & ~rs1_value（清位）
    // ================================================================
    // [新增] 把 EXE 级这条 CSR 指令的地址送出去，供 top 驱动 csr_regfile 的 EXE 读口
    assign exe_csr_addr_o = exe_csr_addr;

    wire [31:0] csr_op_src; // csrrw用rs1，csrrwi用uimm（零扩展到32位）
    wire is_csr_imm = exe_inst_csrrwi | exe_inst_csrrsi | exe_inst_csrrci;
    assign csr_op_src = is_csr_imm ? {27'b0, exe_csr_uimm} : exe_rs1_value;
 
    wire [31:0] exe_csr_wdata;
    assign exe_csr_wdata =
        (exe_inst_csrrw  | exe_inst_csrrwi) ? csr_op_src                        :   // 全量写
        (exe_inst_csrrs  | exe_inst_csrrsi) ? (csr_rvalue | csr_op_src)         :   // 置位
        (exe_inst_csrrc  | exe_inst_csrrci) ? (csr_rvalue & ~csr_op_src)        :   // 清位
        32'b0;
 
    // CSR 写使能：csrrs/csrrc 当 src=0 时不写（只读）
    wire exe_csr_we = exe_valid && (
        exe_inst_csrrw  | exe_inst_csrrwi |                                 // csrrw 始终写
        (exe_inst_csrrs  | exe_inst_csrrsi) && (csr_op_src != 32'b0) |     // csrrs: src!=0 才写
        (exe_inst_csrrc  | exe_inst_csrrci) && (csr_op_src != 32'b0)       // csrrc: src!=0 才写
    );


    //output bus to mem stage    
    wire [31:0] alu_result;
    assign exe_to_mem_bus = {
            exe_pc,
            alu_result,//并不因为提前一拍绕过ID/EX寄存器就不需要传递了，因为后续的写回阶段还需要这个结果，并不是读内存用的
            exe_res_from_mem,    
            exe_reg_we,     
            exe_reg_waddr,  
            // 修改：只需要将 load 信号向后传，store不需要
            inst_lb, inst_lh, inst_lw, inst_lbu, inst_lhu,

            // [新增] CSR 相关透传字段（供 WB 级使用）
            exe_inst_ecall,   // 1
            exe_inst_mret,    // 1
            exe_csr_we,       // 1  (已算好的CSR写使能)
            exe_csr_addr,     // 12
            exe_csr_wdata,    // 32
            exe_is_csr_inst   // 1
            // 合计新增 48 位
    };
    //位宽是32+32+1+1+5+5=76
    //  76 + 48 = 124

    //output bus to id stage for bypass
    wire exe_is_load = exe_mem_en && (exe_mem_we == 4'b0000);//判断exe阶段的这条指令是不是ld.w指令，特征是使能内存但不写内存（exe_mem_en 区分访存指令和其他指令，exe_mem_we==4'b0000是为了区分ld.w和st.w）
    assign exe_to_id_bypass_bus = {
        exe_valid, 
        exe_reg_we, 
        exe_reg_waddr, 
        alu_result,
        exe_is_load,
        flush_en 
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

    
    /*分支跳转br unit*/ 
    wire rs1_eq_rs2 = (exe_rs1_value == exe_rs2_value);
    // 新增：提取判断条件给新型分支指令用
    wire rs1_l_rs2  = ($signed(exe_rs1_value) < $signed(exe_rs2_value));     // blt
    wire rs1_lu_rs2 = (exe_rs1_value < exe_rs2_value);                       // bltu
    
    // 判断是否分支发生 (修改：增加大小相关的分支判定)
    assign exe_taken = (  (inst_beq  && rs1_eq_rs2)
                      || (inst_bne  && !rs1_eq_rs2)
                      || (inst_blt  && rs1_l_rs2)
                      || (inst_bge  && !rs1_l_rs2)
                      || (inst_bltu && rs1_lu_rs2)
                      || (inst_bgeu && !rs1_lu_rs2)
                      || inst_jal
                      || inst_jalr
                      ) && exe_valid;
                      // [注意] ecall/mret 不触发 EXE 级的 flush_en
                      // 它们的 PC 重定向由 WB 级的 wb_ex/mret_flush 处理
   
      
    // 分支目标地址计算
    wire [31:0] br_target = (inst_beq | inst_bne | inst_blt | inst_bge | inst_bltu | inst_bgeu) ? (exe_pc + imm_B) :
                     (inst_jal)    ? (exe_pc + imm_J) :
                     (inst_jalr)   ? ((exe_rs1_value + imm_I) & ~32'b1) : 
                      32'b0;    
    assign exe_target = exe_taken ? br_target : (exe_pc + 32'h4);

    assign flush_en = ((exe_taken != pre_taken) || (exe_taken && (br_target != pre_target)))
                      && exe_valid;   

    // [注意] BTB 更新：ecall/mret/CSR 指令不更新 BTB，也不进行动态分支预测
    // exe_we 只在真正的分支/跳转指令上置1
    assign exe_we = (inst_beq | inst_bne | inst_blt | inst_bge | inst_bltu | inst_bgeu | inst_jal | inst_jalr)
                                  && exe_valid && exe_ready_go;

    assign exe_tag      = exe_pc[22:8]; 
    assign exe_is_ret = exe_is_ret_from_id;



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