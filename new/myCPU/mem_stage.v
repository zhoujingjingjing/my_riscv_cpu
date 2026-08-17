`include "mycpu_top.h"

// 文件职责：七级流水的MEM级，完成load数据提取、MUL/late结果选择和写回打包。
// 上游：exe_stage提供两个EXE/MEM控制包，数据RAM和乘法器提供返回结果。
// 下游：wb_stage接收退休/写回总线，issue_stage接收本级旁路结果。
// [阶段4修改] 双槽MEM级。两个槽的PC、指令、rd、结果和访存元数据
// 始终同拍锁存、同拍前进，slot0保持更老，slot1保持更年轻。
module mem_stage(
    input  wire clk,
    input  wire reset,
    input  wire [`EXE_TO_MEM_BUS_WIDTH-1:0] exe_to_mem_bus,
    input  wire [`EXE_TO_MEM_BUS_WIDTH-1:0] exe_to_mem_bus1,
    output wire [`MEM_TO_WB_BUS_WIDTH-1:0] mem_to_wb_bus,
    output wire [`MEM_TO_WB_BUS_WIDTH-1:0] mem_to_wb_bus1,
    output wire [`MEM_TO_ID_BYPASS_BUS_WIDTH-1:0] mem_to_id_bypass_bus,
    output wire [`MEM_TO_ID_BYPASS_BUS_WIDTH-1:0] mem_to_id_bypass_bus1,
    input  wire exe_to_mem_valid,
    input  wire exe_to_mem_valid1,
    input  wire wb_allow_in,
    output wire mem_allow_in,
    output wire mem_to_wb_valid,
    output wire mem_to_wb_valid1,
    input  wire [31:0] data_sram_rdata,
    // [阶段5A新增] 与进入MEM的MUL槽同拍到达的共享流水结果。
    input  wire        mul_valid,
    input  wire [31:0] mul_result
);
    //============================================================
    // pipeline registers and EXE bus metadata
    //============================================================

    reg mem_valid;
    reg mem_valid1;
    reg [`EXE_TO_MEM_BUS_WIDTH-1:0] mem_reg;
    reg [`EXE_TO_MEM_BUS_WIDTH-1:0] mem_reg1;

    //============================================================
    // late ALU dependency selection
    //============================================================

    // [阶段5C新增] 新元数据位于EXE/MEM总线最低80位，is_mul继续保持
    // bit0，便于5A控制不变。所有选择信号都已经过EXE/MEM寄存器。
    wire late = mem_reg1[79];
    wire dep1 = mem_reg1[78];
    wire dep2 = mem_reg1[77];
    wire [11:0] late_op = mem_reg1[76:65];
    wire [31:0] late_src1_q = mem_reg1[64:33];
    wire [31:0] late_src2_q = mem_reg1[32:1];
    wire [31:0] alu_result0;
    wire [31:0] alu_result1;
    wire [31:0] late_src1 = dep1 ? alu_result0 : late_src1_q;
    wire [31:0] late_src2 = dep2 ? alu_result0 : late_src2_q;
    wire [31:0] late_result;

    late_alu u_late(
        .alu_op(late_op), .src1(late_src1),
        .src2(late_src2), .result(late_result)
    );

    //============================================================
    // MUL wait and downstream handshake
    //============================================================

    // [阶段5A新增] is_mul位位于单槽EXE/MEM总线最低位。正常情况下
    // mul_valid必定与MUL槽同拍；这里保留轻量valid检查，防止无效结果退休。
    wire mul_need = (mem_valid && mem_reg[0]) ||
                    (mem_valid1 && mem_reg1[0]);
    wire mem_ready = !mul_need || mul_valid;
    assign mem_allow_in = !mem_valid || (mem_ready && wb_allow_in);
    assign mem_to_wb_valid = mem_valid && mem_ready;
    assign mem_to_wb_valid1 = mem_valid1 && mem_ready;

    always @(posedge clk) begin
        if (reset) begin
            mem_valid <= 1'b0;
            mem_valid1 <= 1'b0;
        end else if (mem_allow_in) begin
            mem_valid <= exe_to_mem_valid;
            mem_valid1 <= exe_to_mem_valid && exe_to_mem_valid1;
        end
    end

    always @(posedge clk) begin
        if (exe_to_mem_valid && mem_allow_in) begin
            mem_reg <= exe_to_mem_bus;
            mem_reg1 <= exe_to_mem_bus1;
        end
    end

    //============================================================
    // two memory slots and MEM to WB buses
    //============================================================

    mem_lane u_lane0(
        .bus(mem_reg), .valid(mem_valid), .data_rdata(data_sram_rdata),
        .mul_result(mul_result), .late_result(32'b0),
        .wb_bus(mem_to_wb_bus), .bypass_bus(mem_to_id_bypass_bus),
        .alu_result_o(alu_result0)
    );
    mem_lane u_lane1(
        .bus(mem_reg1), .valid(mem_valid1), .data_rdata(data_sram_rdata),
        .mul_result(mul_result), .late_result(late_result),
        .wb_bus(mem_to_wb_bus1), .bypass_bus(mem_to_id_bypass_bus1),
        .alu_result_o(alu_result1)
    );

    wire unused_late = &{1'b0, late, alu_result1};
endmodule

//============================================================
// local memory slot
//============================================================

// 本文件私有辅助模块：mem_lane只由mem_stage实例化，完成单槽load/MUL/late结果选择。
`include "mycpu_top.h"

// [阶段4新增] 单槽MEM组合通路。两个槽共享同一个data RAM返回口，
// 配对器保证一组最多一条load/store，所以只有真正的load槽会选择rdata。
module mem_lane(
    input  wire [`EXE_TO_MEM_BUS_WIDTH-1:0] bus,
    input  wire        valid,
    input  wire [31:0] data_rdata,
    input  wire [31:0] mul_result,
    input  wire [31:0] late_result,
    output wire [`MEM_TO_WB_BUS_WIDTH-1:0] wb_bus,
    output wire [`MEM_TO_ID_BYPASS_BUS_WIDTH-1:0] bypass_bus,
    output wire [31:0] alu_result_o
);
    //============================================================
    // EXE/MEM bus unpack
    //============================================================
    // 单槽EXE已经把访存类型、CSR控制和退休追踪字段全部打进总线。
    // MEM只做结果选择和load字节抽取，不重新译码指令。
    wire [31:0] pc, inst, dnpc, alu_result;
    wire res_mem, reg_we;
    wire [4:0] reg_rd;
    wire lb, lh, lw, lbu, lhu;
    wire ecall, mret, csr_we;
    wire [11:0] csr_addr;
    wire [31:0] csr_wdata;
    wire is_csr, mem_valid, mem_store, is_mul;
    wire late, dep1, dep2;
    wire [11:0] late_op;
    wire [31:0] late_src1, late_src2;
    wire [31:0] mem_addr;
    wire [3:0] mem_mask;
    wire [31:0] mem_wdata;

    assign {
        pc, inst, dnpc, alu_result, res_mem, reg_we, reg_rd,
        lb, lh, lw, lbu, lhu,
        ecall, mret, csr_we, csr_addr, csr_wdata, is_csr,
        mem_valid, mem_store, mem_addr, mem_mask, mem_wdata,
        late, dep1, dep2, late_op, late_src1, late_src2,
        is_mul
    } = bus;

    //============================================================
    // load data extraction
    //============================================================
    // data_rdata是32位对齐返回值；字节/半字load根据地址低位右移到最低位，
    // 再按照LB/LH有符号、LBU/LHU无符号规则扩展成最终写回值。
    wire [31:0] byte_shift = data_rdata >> (alu_result[1:0] * 8);
    wire [31:0] half_shift = data_rdata >> (alu_result[1] * 16);
    wire [7:0] byte_data = byte_shift[7:0];
    wire [15:0] half_data = half_shift[15:0];
    wire [31:0] load_data = lb  ? {{24{byte_data[7]}}, byte_data} :
                            lbu ? {24'b0, byte_data} :
                            lh  ? {{16{half_data[15]}}, half_data} :
                            lhu ? {16'b0, half_data} : data_rdata;
    // [阶段5A新增] MUL结果在EXE/MEM边界之后才完成平衡加法树，因此
    // MUL槽优先选择共享乘法结果；load和普通ALU继续走原有选择路径。
    wire [31:0] result = late ? late_result :
                         is_mul ? mul_result :
                         res_mem ? load_data : alu_result;

    //============================================================
    // WB bus and MEM bypass output
    //============================================================
    // WB总线携带软件可见退休信息；旁路总线只携带ISSUE下一拍需要的rd和值。
    // valid为0时payload可保持旧值，消费者必须先看valid位。
    // [阶段5C新增] late_alu直接读取slot0在EXE算出的原始ALU结果，
    // 不经过load/MUL/最终结果MUX，缩短slot0到后置ALU的组合路径。
    assign alu_result_o = alu_result;

    assign wb_bus = {
        pc, inst, dnpc, result, reg_we, reg_rd,
        ecall, mret, csr_we, csr_addr, csr_wdata, is_csr,
        mem_valid, mem_store, mem_addr, mem_mask, mem_wdata, load_data
    };
    assign bypass_bus = {valid, reg_we, reg_rd, result};

    wire unused_late_meta = &{1'b0, dep1, dep2, late_op,
                              late_src1, late_src2};
endmodule

//============================================================
// local late ALU
//============================================================

// 本文件私有辅助模块：late_alu只由mem_stage使用，计算slot1的后置整数结果。
// [阶段5C新增] MEM后置整数ALU。
// 输入全部来自EXE/MEM寄存器：slot0结果已经锁存，slot1的操作类型和
// 非依赖操作数也已经锁存，因此不会形成同拍ALU0到ALU1的串联路径。
module late_alu(
    input  wire [11:0] alu_op,
    input  wire [31:0] src1,
    input  wire [31:0] src2,
    output wire [31:0] result
);
    wire sub = alu_op[1] || alu_op[2] || alu_op[3];
    wire [32:0] add = {1'b0, src1} + {1'b0, sub ? ~src2 : src2} + sub;
    wire [31:0] sum = add[31:0];
    wire slt = (src1[31] != src2[31]) ? src1[31] : sum[31];
    wire sltu = ~add[32];
    wire [31:0] sll = src1 << src2[4:0];
    wire [63:0] sr = {{32{alu_op[10] && src1[31]}}, src1} >> src2[4:0];
    // [ORC.B新增] 后置ALU也支持同一运算，使slot0普通ALU到slot1 orc.b
    // 的受限RAW配对仍可按阶段5C原设计执行，不必额外退化成单发射。
    wire [31:0] orcb = {{8{|src1[31:24]}},
                        {8{|src1[23:16]}},
                        {8{|src1[15: 8]}},
                        {8{|src1[ 7: 0]}}};

    // 各运算并行计算，末端仅保留一级结果选择；MEM不再进行指令译码。
    assign result = ({32{alu_op[0] || alu_op[1]}} & sum) |
                    ({32{alu_op[2]}} & {31'b0, slt}) |
                    ({32{alu_op[3]}} & {31'b0, sltu}) |
                    ({32{alu_op[4]}} & (src1 & src2)) |
                    ({32{alu_op[5]}} & orcb) |
                    ({32{alu_op[6]}} & (src1 | src2)) |
                    ({32{alu_op[7]}} & (src1 ^ src2)) |
                    ({32{alu_op[8]}} & sll) |
                    ({32{alu_op[9] || alu_op[10]}} & sr[31:0]) |
                    ({32{alu_op[11]}} & src2);
endmodule
