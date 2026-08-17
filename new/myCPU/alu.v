// 文件职责：32位整数ALU。它只做组合运算，不保存任何流水状态。
// 上游：exe_stage把译码得到的alu_op和两个操作数送入本模块。
// 下游：exe_stage把alu_result放入普通EXE结果或MEM级后置计算输入。
module alu(
  input  wire [11:0] alu_op,
  input  wire [31:0] alu_src1,
  input  wire [31:0] alu_src2,
  output wire [31:0] alu_result
);

// alu_op采用独热编码。译码器只会打开一个操作位，因此各个候选结果
// 可以并行计算，末尾的结果选择器再把唯一有效的结果送给EXE级。

//============================================================
// control code decomposition
//============================================================
wire op_add;   //add operation
wire op_sub;   //sub operation
wire op_slt;   //signed compared and set less than
wire op_sltu;  //unsigned compared and set less than
wire op_and;   //bitwise and
wire op_orcb;  //OR-combine bytes (Zbb orc.b)
wire op_or;    //bitwise or
wire op_xor;   //bitwise xor
wire op_sll;   //logic left shift
wire op_srl;   //logic right shift
wire op_sra;   //arithmetic right shift
wire op_lui;   //Load Upper Immediate

assign op_add  = alu_op[ 0];
assign op_sub  = alu_op[ 1];
assign op_slt  = alu_op[ 2];
assign op_sltu = alu_op[ 3];
assign op_and  = alu_op[ 4];
// [ORC.B新增] RISC-V没有原生nor，本CPU原alu_op[5]一直空闲，现复用为orc.b。
assign op_orcb = alu_op[ 5];
assign op_or   = alu_op[ 6];
assign op_xor  = alu_op[ 7];
assign op_sll  = alu_op[ 8];
assign op_srl  = alu_op[ 9];
assign op_sra  = alu_op[10];
assign op_lui  = alu_op[11];

wire [31:0] add_sub_result;
wire [31:0] slt_result;
wire [31:0] sltu_result;
wire [31:0] and_result;
wire [31:0] orcb_result;
wire [31:0] or_result;
wire [31:0] xor_result;
wire [31:0] lui_result;
wire [31:0] sll_result;
wire [63:0] sr64_result;
wire [31:0] sr_result;


//============================================================
// shared adder for ADD/SUB/SLT/SLTU
//============================================================
// 32-bit adder
wire [31:0] adder_a;
wire [31:0] adder_b;
wire        adder_cin;
wire [31:0] adder_result;
wire        adder_cout;

assign adder_a   = alu_src1;
assign adder_b   = (op_sub | op_slt | op_sltu) ? ~alu_src2 : alu_src2;  //src1 - src2 rs1-rs2
assign adder_cin = (op_sub | op_slt | op_sltu) ? 1'b1      : 1'b0;
assign {adder_cout, adder_result} = adder_a + adder_b + adder_cin;

// SUB、SLT、SLTU共用这一个33位加法器。减法通过“取反加一”完成；
// SLT读取有符号比较所需的符号位，SLTU读取最高位的借位结果。

// ADD, SUB result
assign add_sub_result = adder_result;

// SLT result
assign slt_result[31:1] = 31'b0;   //rs1 < rs2 1
assign slt_result[0]    = (alu_src1[31] & ~alu_src2[31])
                        | ((alu_src1[31] ~^ alu_src2[31]) & adder_result[31]);

// SLTU result
assign sltu_result[31:1] = 31'b0;
assign sltu_result[0]    = ~adder_cout;

//============================================================
// bit operations and shifts
//============================================================
// bitwise operation
assign and_result = alu_src1 & alu_src2;
assign or_result  = alu_src1 | alu_src2;
// [ORC.B新增] 每个字节独立判断“是否至少有一个1”。归约结果再复制8位：
// 输入字节为0时输出00，非0时输出ff。四个字节并行，不形成32位串行链。
assign orcb_result = {{8{|alu_src1[31:24]}},
                      {8{|alu_src1[23:16]}},
                      {8{|alu_src1[15: 8]}},
                      {8{|alu_src1[ 7: 0]}}};
assign xor_result = alu_src1 ^ alu_src2;
assign lui_result = alu_src2;

// SLL result
assign sll_result = alu_src1 << alu_src2[4:0];   //rs1 << i5

// SRL, SRA result
assign sr64_result = {{32{op_sra & alu_src1[31]}}, alu_src1[31:0]} >> alu_src2[4:0]; //rs1 >> i5

assign sr_result   = sr64_result[31:0];
//============================================================
// final result mux
//============================================================
// 所有操作都保持原有控制位和数据路径，最后用按位屏蔽合并为一个结果。
// final result mux
assign alu_result = ({32{op_add|op_sub}} & add_sub_result)
                  | ({32{op_slt       }} & slt_result)
                  | ({32{op_sltu      }} & sltu_result)
                  | ({32{op_and       }} & and_result)
                  | ({32{op_orcb      }} & orcb_result)
                  | ({32{op_or        }} & or_result)
                  | ({32{op_xor       }} & xor_result)
                  | ({32{op_lui       }} & lui_result)
                  | ({32{op_sll       }} & sll_result)
                  | ({32{op_srl|op_sra}} & sr_result);

endmodule
