`include "mycpu_top.h"

// 文件职责：七级流水的EXE级，执行整数ALU、分支、LSU地址和共享RV32M请求。
// 上游：issue_stage提供两条带前递操作数的控制包；CSR和除法器提供只读/结果输入。
// 下游：mem_stage接收两个EXE结果总线，IF接收分支纠错，顶层接收访存请求。
// [阶段4修改] 双槽EXE级。两个整数ALU位于exe_lane中独立并行，
// LSU、分支更新入口和RV32M单元保持共享；配对器保证每组最多申请一次共享资源。
module exe_stage(
    input  wire clk,
    input  wire reset,
    input  wire [`ISSUE_TO_EXE_BUS_WIDTH-1:0] id_to_exe_bus,
    input  wire [`ISSUE_TO_EXE_BUS_WIDTH-1:0] id_to_exe_bus1,
    output wire [`EXE_TO_MEM_BUS_WIDTH-1:0] exe_to_mem_bus,
    output wire [`EXE_TO_MEM_BUS_WIDTH-1:0] exe_to_mem_bus1,
    output wire [`EXE_TO_IF_BUS_WIDTH-1:0] exe_to_if_bus,
    output wire [`EXE_TO_ID_BYPASS_BUS_WIDTH-1:0] exe_to_id_bypass_bus,
    output wire [`EXE_TO_ID_BYPASS_BUS_WIDTH-1:0] exe_to_id_bypass_bus1,
    input  wire id_to_exe_valid,
    input  wire id_to_exe_valid1,
    input  wire mem_allow_in,
    output wire exe_allow_in,
    output wire exe_to_mem_valid,
    output wire exe_to_mem_valid1,
    // [阶段5A新增] 共享流水MUL在EXE/MEM边界寄存部分积，结果和valid
    // 直接送到MEM，由携带is_mul的槽选择。
    output wire mul_valid_o,
    output wire [31:0] mul_result,
    input  wire wb_ex,
    input  wire mret_flush,
    output wire [11:0] csr_raddr,
    input  wire [31:0] csr_rvalue,
    // [阶段5C2新增] 直接把顶层DRAM返回值送入两个EXE槽的局部修复MUX，
    // 不经过MEM最终结果选择器，避免增加额外组合层级。
    input  wire [31:0] data_sram_rdata,
    output wire data_sram_en,
    output wire [3:0] data_sram_we,
    output wire [31:0] data_sram_addr,
    output wire [31:0] data_sram_wdata,
    output wire perf_branch,
    output wire perf_branch_mispredict
);
    //============================================================
    // pipeline control and shared MDU
    //============================================================

    // 白话信号导读：
    // wb_flush像“紧急清场铃”，WB发现ecall异常或mret返回时，EXE里正在排队的东西都不能继续算。
    // exe_valid/exe_valid1分别表示slot0/slot1这两个执行座位上有没有真指令；slot1永远比slot0年轻。
    // exe_reg/exe_reg1是ISSUE送来的整包资料，里面有PC、指令、操作数、访存类型、CSR信息等。
    // is_m0/is_m1表示本槽是不是乘除法；mdu_op0/mdu_op1用funct3压缩出具体是mul、div还是rem。
    // mdu_a*/mdu_b*是送给乘除法单元的两个原始操作数，保持原味，不走普通ALU的立即数选择。
    // mul_req*是“要坐快速乘法流水线”的申请，div_req*是“要占用慢速除法器”的申请。
    // mdu_sel1说明共享MDU这拍听slot1的话；正常配对规则保证同一组里最多只有一个人真正申请。
    // div_started记住串行除法已经按下启动键，防止EXE被除法拖住时反复向除法器发同一条命令。
    // div_busy/div_done/div_result来自除法器：忙、完成、结果；exe_ready_go只有在除法等到结果后才放行。
    // mul_en跟EXE跨到MEM的时机对齐，避免乘法部分积先跑走、对应指令却被MEM反压留在原地。
    // mul_valid只是告诉MDU“这组里确实有乘法”，异常/返回冲刷时不会再发新乘法。
    wire wb_flush = wb_ex || mret_flush;
    reg exe_valid;
    reg exe_valid1;
    reg [`ISSUE_TO_EXE_BUS_WIDTH-1:0] exe_reg;
    reg [`ISSUE_TO_EXE_BUS_WIDTH-1:0] exe_reg1;

    wire is_m0, is_m1;
    wire [2:0] mdu_op0, mdu_op1;
    wire [31:0] mdu_a0, mdu_b0, mdu_a1, mdu_b1;
    // [阶段5A修改] funct3[2]把固定延迟MUL和串行DIV分开。配对器保证
    // 一组最多一条M指令，因此两个槽只需要一层owner选择器。
    wire mul_req0 = exe_valid && is_m0 && !mdu_op0[2];
    wire mul_req1 = exe_valid1 && is_m1 && !mdu_op1[2];
    wire mul_req = mul_req0 || mul_req1;
    wire div_req0 = exe_valid && is_m0 && mdu_op0[2];
    wire div_req1 = exe_valid1 && is_m1 && mdu_op1[2];
    wire div_req = div_req0 || div_req1;
    wire mdu_sel1 = mul_req1 || div_req1;

    reg div_started;
    wire div_busy;
    wire div_done;
    wire [31:0] div_result;
    wire div_start = div_req && !div_started && !div_busy && !wb_flush;
    // MUL不再进入全局ready条件；只有真正的DIV/REM等待串行完成。
    wire exe_ready_go = !div_req || (div_started && div_done);

    // en与EXE组真正跨越EXE/MEM边界的条件一致。MEM被阻塞时，乘法
    // valid和四个部分积一起保持，避免结果和对应指令错拍。
    wire mul_en = exe_ready_go && mem_allow_in;
    wire mul_valid = mul_req && !wb_flush;

    assign exe_allow_in = !exe_valid || (exe_ready_go && mem_allow_in);
    assign exe_to_mem_valid = exe_valid && exe_ready_go && !wb_flush;
    assign exe_to_mem_valid1 = exe_valid1 && exe_ready_go && !wb_flush;

    always @(posedge clk) begin
        if (reset || wb_flush) begin
            exe_valid <= 1'b0;
            exe_valid1 <= 1'b0;
        end else if (exe_allow_in) begin
            exe_valid <= id_to_exe_valid;
            exe_valid1 <= id_to_exe_valid && id_to_exe_valid1;
        end
    end

    always @(posedge clk) begin
        if (id_to_exe_valid && exe_allow_in) begin
            exe_reg <= id_to_exe_bus;
            exe_reg1 <= id_to_exe_bus1;
        end
    end

    always @(posedge clk) begin
        if (reset || wb_flush)
            div_started <= 1'b0;
        else if (exe_allow_in)
            div_started <= 1'b0;
        else if (div_start)
            div_started <= 1'b1;
    end

    rv32m_mdu u_rv32m_mdu(
        .clk(clk), .resetn(~reset), .cancel(wb_flush),
        .mul_en(mul_en), .mul_valid(mul_valid),
        .mul_valid_o(mul_valid_o), .mul_result(mul_result),
        .div_start(div_start), .div_busy(div_busy),
        .div_done(div_done), .div_result(div_result),
        .op(mdu_sel1 ? mdu_op1 : mdu_op0),
        .operand_a(mdu_sel1 ? mdu_a1 : mdu_a0),
        .operand_b(mdu_sel1 ? mdu_b1 : mdu_b0)
    );

    //============================================================
    // two execution slots
    //============================================================

    // 两个lane各自像一张独立的算术桌：
    // br_bus0/br_bus1是分支纠错信封，里面有是否flush、新PC、BTB索引、RAS指针等信息。
    // data_en*说明本槽要访问数据存储器；data_we*为字节写使能，0表示load，非0表示store。
    // data_addr*/data_wdata*分别是LSU算出的地址和准备写入DRAM的数据。
    // ctrl0/ctrl1表示本槽是分支或跳转，外层用它挑出唯一一封要送回IF的分支纠错信封。
    // csr_addr0/csr_addr1是CSR读地址；基础双发规则让CSR只走较老的slot0，slot1地址只保留给lint兜底。
    // perf_br*/perf_mis*是每个槽贡献的分支统计脉冲，外层简单或起来给性能计数器。
    wire [`EXE_TO_IF_BUS_WIDTH-1:0] br_bus0, br_bus1;
    wire data_en0, data_en1;
    wire [3:0] data_we0, data_we1;
    wire [31:0] data_addr0, data_addr1, data_wdata0, data_wdata1;
    wire ctrl0, ctrl1;
    wire [11:0] csr_addr0, csr_addr1;
    wire perf_br0, perf_br1, perf_mis0, perf_mis1;

    exe_lane u_lane0(
        .bus(exe_reg), .valid(exe_valid), .ready(exe_ready_go),
        .wb_flush(wb_flush), .csr_rvalue(csr_rvalue), .div_result(div_result),
        .data_rdata(data_sram_rdata),
        .mem_bus(exe_to_mem_bus), .bypass_bus(exe_to_id_bypass_bus),
        .br_bus(br_bus0), .data_en(data_en0), .data_we(data_we0),
        .data_addr(data_addr0), .data_wdata(data_wdata0),
        .is_m(is_m0), .mdu_op(mdu_op0), .mdu_a(mdu_a0), .mdu_b(mdu_b0),
        .is_ctrl(ctrl0), .csr_addr(csr_addr0),
        .perf_branch(perf_br0), .perf_mispredict(perf_mis0)
    );
    exe_lane u_lane1(
        .bus(exe_reg1), .valid(exe_valid1), .ready(exe_ready_go),
        .wb_flush(wb_flush), .csr_rvalue(32'b0), .div_result(div_result),
        .data_rdata(data_sram_rdata),
        .mem_bus(exe_to_mem_bus1), .bypass_bus(exe_to_id_bypass_bus1),
        .br_bus(br_bus1), .data_en(data_en1), .data_we(data_we1),
        .data_addr(data_addr1), .data_wdata(data_wdata1),
        .is_m(is_m1), .mdu_op(mdu_op1), .mdu_a(mdu_a1), .mdu_b(mdu_b1),
        .is_ctrl(ctrl1), .csr_addr(csr_addr1),
        .perf_branch(perf_br1), .perf_mispredict(perf_mis1)
    );

    //============================================================
    // shared branch, CSR and data-memory outputs
    //============================================================

    // [阶段4新增] 每组最多一条控制流和一条访存，选择器只有一层2选1。
    // use_lane1_br像“选哪封改PC信”：slot1有效且它是控制流时，IF采用slot1的纠错结果；否则采用slot0。
    // use_lane1_mem像“选哪个人用数据口”：slot1有效且它访存时，DRAM地址/写数据来自slot1；否则来自slot0。
    // 这些选择没有重新仲裁复杂资源，只是落实ID阶段已经做好的配对承诺。
    wire use_lane1_br = exe_valid1 && ctrl1;
    assign exe_to_if_bus = use_lane1_br ? br_bus1 : br_bus0;
    assign csr_raddr = csr_addr0;

    wire use_lane1_mem = exe_valid1 && data_en1;
    assign data_sram_en = data_en0 || data_en1;
    assign data_sram_we = use_lane1_mem ? data_we1 : data_we0;
    assign data_sram_addr = use_lane1_mem ? data_addr1 : data_addr0;
    assign data_sram_wdata = use_lane1_mem ? data_wdata1 : data_wdata0;

    assign perf_branch = perf_br0 || perf_br1;
    assign perf_branch_mispredict = perf_mis0 || perf_mis1;

    // 配对器已经禁止双M、双LSU和slot0控制流配对；这些归约只用于lint。
    wire unused_guard = &{1'b0, ctrl0 && exe_valid1, csr_addr1,
                          (mul_req0 || div_req0) && (mul_req1 || div_req1),
                          data_en0 && data_en1};
endmodule

//============================================================
// local execution slot
//============================================================

// 本文件私有辅助模块：exe_lane只由上面的exe_stage实例化，负责单槽组合执行。
`include "mycpu_top.h"

// [阶段4新增] 单条执行通路的纯组合部分。
// 两个实例各自拥有一个整数ALU；LSU、分支更新口和MDU仍由外层exe_stage
// 统一仲裁。这样两条ALU指令并行计算，又不会建立跨槽结果接力。
module exe_lane(
    input  wire [`ISSUE_TO_EXE_BUS_WIDTH-1:0] bus,
    input  wire        valid,
    input  wire        ready,
    input  wire        wb_flush,
    input  wire [31:0] csr_rvalue,
    input  wire [31:0] div_result,
    // [阶段5C2新增] DRAM同步读口当前拍返回上一拍LW的数据。只有已经
    // 寄存的ld1/ld2标签会选择它，其他执行类型完全不使用这条数据线。
    input  wire [31:0] data_rdata,
    output wire [`EXE_TO_MEM_BUS_WIDTH-1:0] mem_bus,
    output wire [`EXE_TO_ID_BYPASS_BUS_WIDTH-1:0] bypass_bus,
    output wire [`EXE_TO_IF_BUS_WIDTH-1:0] br_bus,
    output wire        data_en,
    output wire [3:0]  data_we,
    output wire [31:0] data_addr,
    output wire [31:0] data_wdata,
    output wire        is_m,
    output wire [2:0]  mdu_op,
    output wire [31:0] mdu_a,
    output wire [31:0] mdu_b,
    output wire        is_ctrl,
    output wire [11:0] csr_addr,
    output wire        perf_branch,
    output wire        perf_mispredict
);
    //============================================================
    // ISSUE/EXE bus unpack
    //============================================================
    // ISSUE已经完成寄存器读取和前递选择，本槽只按固定总线顺序解包。
    // ld1/ld2是上一拍ISSUE生成的LW修复标签，late/dep1/dep2继续传给MEM后置ALU。
    // pc/inst是这条指令自己的地址和原始编码，退休、异常和分支修正都靠它们对账。
    // alu_op告诉ALU做加减、比较、移位还是逻辑运算；src1_pc/src2_imm控制两个ALU输入的来源。
    // res_mem表示最终写回值来自MEM读数而不是EXE结果；mem_en/mem_we描述是否访存以及store字节写法。
    // reg_we/reg_rd是写回寄存器的“收件人信息”，一路带到WB给regfile使用。
    // rs1/rs2是ISSUE已经读好、也已经尽量前递修补好的操作数；imm是译码阶段整理好的立即数。
    // lb/lh/lw/lbu/lhu/sb/sh/sw把访存宽度说清楚，后面生成mask和load扩展时直接看这些小旗子。
    // beq/bne/blt/bge/bltu/bgeu/jal/jalr描述控制流类型，EXE在这里真正比较条件、算跳转目标。
    // pre_taken/pre_target/pre_index是前端当初的预测记录，EXE拿真实结果和它对比，错了就发flush。
    // imm_b/imm_j/imm_i分别是分支、JAL、JALR会用到的目标偏移，提前带过来避免这里重新拼指令。
    // is_ret/safe_ras服务返回地址栈：is_ret表示这是函数返回，safe_ras是预测时保存下来的RAS安全指针。
    // ecall/mret和csrr*是CSR/异常类控制信号；csr_uimm是CSR立即数形式里的5位无符号立即数。
    // is_csr提醒WB写回rd时要写“CSR旧值”；is_m/mdu_op说明这条指令要走共享乘除法单元。
    wire [31:0] pc;
    wire [31:0] inst;
    wire [11:0] alu_op;
    wire src1_pc, src2_imm, res_mem, mem_en;
    wire [3:0] mem_we;
    wire reg_we;
    wire [4:0] reg_rd;
    wire [31:0] rs1, rs2, imm;
    wire lb, lh, lw, lbu, lhu, sb, sh, sw;
    wire beq, bne, blt, bge, bltu, bgeu, jal, jalr;
    wire pre_taken;
    wire [31:0] pre_target;
    wire [5:0] pre_index;
    wire [31:0] imm_b, imm_j, imm_i;
    wire is_ret;
    wire [2:0] safe_ras;
    wire ecall, mret, csrrw, csrrs, csrrc, csrrwi, csrrsi, csrrci;
    wire [4:0] csr_uimm;
    wire is_csr;
    wire ld1, ld2, late, dep1, dep2;

    assign {
        ld1, ld2, late, dep1, dep2,
        pc, inst, alu_op, src1_pc, src2_imm, res_mem, mem_en, mem_we,
        reg_we, reg_rd, rs1, rs2, imm,
        lb, lh, lw, lbu, lhu, sb, sh, sw,
        beq, bne, blt, bge, bltu, bgeu, jal, jalr,
        pre_taken, pre_target, pre_index,
        imm_b, imm_j, imm_i, is_ret, safe_ras,
        ecall, mret, csrrw, csrrs, csrrc, csrrwi, csrrsi, csrrci,
        csr_addr, csr_uimm, is_csr, is_m, mdu_op
    } = bus;

    //============================================================
    // ALU operands and RV32M result selection
    //============================================================
    // [阶段5C2新增] DRAM返回值只经过一个局部2选1MUX后进入本槽ALU。
    // rd比较和消费者译码已在上一拍ISSUE完成，不会进入当前数据关键路径。
    // rs1_eff/rs2_eff是“修补后的操作数”：普通情况用ISSUE给的rs1/rs2，紧跟LW时直接借DRAM回来的数据。
    // alu_src1/alu_src2是ALU真正看到的两个输入，一个可能换成PC，另一个可能换成立即数。
    // alu_raw是普通整数ALU的原始结果；exe_result再把除法结果接进来，乘法结果要等MEM阶段收尾。
    wire [31:0] rs1_eff = ld1 ? data_rdata : rs1;
    wire [31:0] rs2_eff = ld2 ? data_rdata : rs2;
    wire [31:0] alu_src1 = src1_pc ? pc : rs1_eff;
    wire [31:0] alu_src2 = src2_imm ? imm : rs2_eff;
    wire [31:0] alu_raw;
    alu u_alu(
        .alu_op(alu_op), .alu_src1(alu_src1),
        .alu_src2(alu_src2), .alu_result(alu_raw)
    );
    // [阶段5A修改] MUL的最终结果要到MEM才由已寄存部分积求出，EXE不能
    // 把尚未完成的值写进数据总线；DIV完成时仍在EXE直接选择串行结果。
    wire is_mul = is_m && !mdu_op[2];
    wire is_div = is_m &&  mdu_op[2];
    wire [31:0] exe_result = is_div ? div_result : alu_raw;

    // [阶段7A新增] LSU使用独立AGU，只读取原始base和立即数。阶段5C2的
    // data_rdata只进入普通ALU，不能再沿ALU结果延伸到Store/MMIO控制。
    // biRISC-V和VeeR同样把LSU地址计算与普通整数ALU数据通路分开。
    // agu_result是访存专用地址加法器结果，只做base+offset；result是本槽先交给后级的主结果。
    // mem_en为1时result暂放地址，MEM再用load数据覆盖；非访存时result就是ALU/DIV结果。
    wire [31:0] agu_result = rs1 + imm;
    wire [31:0] result = mem_en ? agu_result : exe_result;

    assign mdu_a = rs1;
    assign mdu_b = rs2;

    //============================================================
    // branch decision and BTB repair bus
    //============================================================
    // 分支比较只使用原始rs1/rs2，和普通ALU数据通路分离；真正改变PC的
    // flush必须等本槽valid且EXE准备离开，避免停顿期间提前清空流水线。
    // eq/lt/ltu分别是相等、有符号小于、无符号小于，六种条件分支都由这三个比较拼出来。
    // taken_raw是不考虑valid/flush的真实跳转结论；taken再加上“这条指令确实活着”的保护。
    // target_raw是真实目标地址，分支用pc+imm_b，jal用pc+imm_j，jalr用rs1+imm_i并清掉最低位。
    // target是告诉前端下一步去哪：跳转成立走target_raw，否则顺序走pc+4。
    // flush表示预测和真实结果不一致；btb_we表示这条控制流已经有真实结果，可以更新BTB。
    wire eq = rs1 == rs2;
    wire lt = $signed(rs1) < $signed(rs2);
    wire ltu = rs1 < rs2;
    wire taken_raw = (beq && eq) || (bne && !eq) ||
                     (blt && lt) || (bge && !lt) ||
                     (bltu && ltu) || (bgeu && !ltu) || jal || jalr;
    wire taken = taken_raw && valid && !wb_flush;
    wire [31:0] target_raw = (beq | bne | blt | bge | bltu | bgeu) ?
                             pc + imm_b :
                             jal ? pc + imm_j :
                             jalr ? ((rs1 + imm_i) & ~32'b1) : 32'b0;
    wire [31:0] target = taken ? target_raw : pc + 32'd4;
    wire flush = valid && ready && !wb_flush &&
                 ((taken != pre_taken) ||
                  (taken && target_raw != pre_target));
    assign is_ctrl = beq | bne | blt | bge | bltu | bgeu | jal | jalr;
    wire btb_we = valid && ready && !wb_flush && is_ctrl;
    assign br_bus = {
        flush, target, btb_we, pre_index, pc[22:8], taken,
        is_ret, safe_ras
    };

    assign perf_branch = valid && ready && !wb_flush &&
                         (beq | bne | blt | bge | bltu | bgeu);
    assign perf_mispredict = perf_branch && flush;

    //============================================================
    // CSR write-data calculation
    //============================================================
    // CSR旧值由csr_rvalue输入，写数据在EXE先算好，真正提交仍由WB按年龄完成。
    // csr_imm区分CSR操作数来自指令里的uimm还是来自rs1寄存器。
    // csr_src是已经选好的CSR源操作数；csr_wdata按csrrw/csrrs/csrrc语义生成新CSR值。
    // csr_we只表示“这条CSR指令确实需要写CSR”，真正写寄存器堆要等WB确认没有异常。
    wire csr_imm = csrrwi | csrrsi | csrrci;
    wire [31:0] csr_src = csr_imm ? {27'b0, csr_uimm} : rs1;
    wire [31:0] csr_wdata = (csrrw | csrrwi) ? csr_src :
                            (csrrs | csrrsi) ? (csr_rvalue | csr_src) :
                            (csrrc | csrrci) ? (csr_rvalue & ~csr_src) :
                            32'b0;
    wire csr_we = valid && ((csrrw | csrrwi) |
                  ((csrrs | csrrsi) && csr_src != 32'b0) |
                  ((csrrc | csrrci) && csr_src != 32'b0));

    //============================================================
    // LSU byte mask/data and EXE outputs
    //============================================================
    // store数据在EXE按访问宽度复制到32位，mask再根据低地址位移动。
    // load只生成mask和地址，真正字节抽取在MEM完成。
    // st_data把sb/sh/sw要写的数据摆成32位形状，方便按字节写使能落到DRAM。
    // st_mask告诉DRAM哪几个字节要写；ld_mask告诉MEM回来后该从32位读数里取哪几个字节。
    // data_en/data_we/data_addr/data_wdata是最终交给顶层数据口的访存请求线。
    // mem_bus是交给MEM的大包裹：包含写回信息、异常/CSR信息、访存信息和后置ALU需要的材料。
    wire [31:0] st_data = sb ? {4{rs2[7:0]}} :
                              sh ? {2{rs2[15:0]}} : rs2;
    wire [3:0] st_mask = sw ? 4'b1111 :
                         sh ? (4'b0011 << agu_result[1:0]) :
                         sb ? (4'b0001 << agu_result[1:0]) : 4'b0;
    wire [3:0] ld_mask = lw ? 4'b1111 :
                         (lh | lhu) ? (4'b0011 << agu_result[1:0]) :
                         (lb | lbu) ? (4'b0001 << agu_result[1:0]) : 4'b0;

    assign data_en = valid && ready && mem_en && !wb_flush;
    assign data_we = {4{mem_we[0]}} & st_mask & {4{data_en}};
    assign data_addr = agu_result;
    assign data_wdata = st_data;

    assign mem_bus = {
        pc, inst, target, result, res_mem, reg_we, reg_rd,
        lb, lh, lw, lbu, lhu,
        ecall, mret, csr_we, csr_addr, csr_wdata, is_csr,
        mem_en, |mem_we, agu_result,
        (|mem_we) ? st_mask : ld_mask, st_data,
        // [阶段5C新增] 后置ALU只读取这些已经越过EXE/MEM边界的字段。
        late, dep1, dep2, alu_op, alu_src1, alu_src2,
        is_mul
    };

    // is_load是真正的load；is_lw专门标出完整32位LW，因为它可以直接把DRAM返回值前递给下一条ALU。
    // is_late表示“EXE这里还拿不到最终写回值”：load等MEM数据，mul等MEM收尾，late等MEM后置ALU。
    // bypass_bus送回ISSUE做冒险判断和前递，最后一位flush让前面的级知道本槽纠正了取指方向。
    wire is_load = mem_en && mem_we == 4'b0;
    // [阶段5C2新增] is_lw严格区分完整32位LW与窄load。窄load需要在MEM
    // 做字节选择和符号/零扩展，不能直接使用原始BRAM返回值。
    wire is_lw = is_load && lw;
    // [阶段5A修改] load标志只代表真实load，继续供原性能计数器使用；
    // late标志表示EXE没有最终结果，ISSUE对load、MUL和后置ALU都必须
    // 等到MEM前递。真实load仍由独立is_load位统计，不改变性能计数口径。
    wire is_late = is_load || is_mul || late;
    assign bypass_bus = {
        valid && ready && !wb_flush,
        reg_we, reg_rd, result, is_load, is_lw, is_late, flush
    };
endmodule
