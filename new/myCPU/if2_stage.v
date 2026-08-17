`include "mycpu_top.h"
`timescale 1ns / 1ps

/*
文件职责：这个文件是取指的“中转站”。它不负责决定下一条 PC，那是 if_stage 的工作。
上游输入来自 if_stage、IROM 和 ID；输出送给 ID。
它只做三件事：记住上一拍发给 IROM 的请求、把本拍返回的指令放进 FIFO、
同时给返回指令贴上配对用的轻量分类标签，
按程序顺序把 FIFO 队头的一条或两条指令交给 ID。
FIFO 里 q0 比 q1 老。ID 只能先拿 q0，只有两条指令可以安全配对时才同时拿 q0/q1。
遇到分支纠错时，旧请求和旧指令都作废；真正的 PC 重定向仍由 if_stage 完成。
*/
module if2_stage(
    input  wire        clk,
    input  wire        reset,

    input  wire        if_to_if2_valid,
    input  wire [`IF_TO_IF2_BUS_WIDTH-1:0] if_to_if2_bus,
    output wire        if2_allow_in,
    input  wire [2:0]  if_ras_ptr,
    input  wire        if_flush,

    input  wire [31:0] inst_sram_rdata,
    input  wire [31:0] inst_sram_rdata_b,

    output wire [`IF_TO_ID_BUS_WIDTH-1:0] if_to_id_bus,
    /*q0 是准备交给 ID 的老指令，q1 是紧跟在它后面的年轻指令。
    ID 会先看看 q0/q1 能不能配对；能配对才会让 id_take2 变成 1。*/
    output wire [`FETCH_BUS_WIDTH-1:0] if_to_id_bus1,
    /*两条预译码标签和各自的取指记录绑定在一起。
    ID 只用它们判断能不能双发，不必重新从机器码里猜一次指令类别。*/
    output wire [24:0] if_to_id_dec0,
    output wire [24:0] if_to_id_dec1,
    output wire        if_to_id_valid,
    output wire        if_to_id_valid1,
    input  wire        id_allow_in,
    /*id_take2 只回答一个问题：“这拍 ID 是否真的同时拿走 q0 和 q1？”
    它只控制 FIFO 弹出几项，不参与 PC 计算，也不参与 ALU 运算。*/
    input  wire        id_take2
);
    /*FIFO 每一项就是一条取指记录：PC、指令字、预测是否跳转、预测目标和 BTB 索引。
    RAS 指针不提前塞进 FIFO，而是在这条指令真正出队时再补上当前值。
    这样年轻指令不会拿到已经过期的 RAS 快照。*/
    localparam FETCH_W = `FETCH_BUS_WIDTH;
    localparam PREDEC_W = 25;
    localparam QUEUE_W = FETCH_W + PREDEC_W;
    localparam META_W  = FETCH_W - 32;

    //============================================================
    // IF1 request register and synchronous IROM response
    //============================================================

    wire [1:0] fetch_num;
    wire [META_W-1:0] if_req0;
    wire [META_W-1:0] if_req1;
    assign {fetch_num, if_req0, if_req1} = if_to_if2_bus;

    /*IROM 是同步读：这一拍给地址，下一拍才拿到指令字。
    req_valid 表示上一拍确实发过请求；req_num 记录那次请求取回一条还是两条；
    req0/req1 保存当时的 PC 和预测信息。它们必须和下一拍返回的指令字配对，不能串错。*/
    reg req_valid;
    reg [1:0] req_num;
    reg [META_W-1:0] req0;
    reg [META_W-1:0] req1;

    /*IROM 返回的机器码在进 FIFO 前先贴上一张 25 位“分类标签”。
    标签只记寄存器读写、ALU/LSU/MUL/控制流等配对所需资料；它不替代
    ID 的完整译码控制包。req_num 说明这次响应有几个有效指令，避免把
    另一读口的无意义数据当成一条指令预译码。*/
    wire [PREDEC_W-1:0] push_dec0;
    wire [PREDEC_W-1:0] push_dec1;
    dual_decode u_push_decode (
        .valid0(req_valid && (req_num >= 2'd1)), .inst0(inst_sram_rdata),
        .valid1(req_valid && (req_num == 2'd2)), .inst1(inst_sram_rdata_b),
        .dec0(push_dec0), .dec1(push_dec1)
    );

    /*push0/push1 是准备写入 FIFO 的完整内部记录：原有取指资料在低位，
    新的预译码标签在高位。这样 FIFO 移位时两者永远不会走散。
    如果上一拍没有有效请求，push_num 就是 0，这两个数据自然不会进入 FIFO。*/
    wire [1:0] push_num = req_valid ? req_num : 2'd0;
    wire [FETCH_W-1:0] push_fetch0 = {
        req0[META_W-1:META_W-32], inst_sram_rdata, req0[META_W-33:0]
    };
    wire [FETCH_W-1:0] push_fetch1 = {
        req1[META_W-1:META_W-32], inst_sram_rdata_b, req1[META_W-33:0]
    };
    wire [QUEUE_W-1:0] push0 = {push_dec0, push_fetch0};
    wire [QUEUE_W-1:0] push1 = {push_dec1, push_fetch1};

    //============================================================
    // fetch queue pop, push and capacity reservation
    //============================================================

    /*pop 表示本拍至少弹出一条。pop_num 只能是 0、1、2：
    0 表示 ID 没接收，1 表示只拿 q0，2 表示 q0/q1 一起拿。
    q1 不能越过 q0；配对失败时 q1 留在队列里，下一拍再试。*/
    wire [QUEUE_W-1:0] q_out0;
    wire [QUEUE_W-1:0] q_out1;
    wire [1:0] q_valid;
    wire [2:0] q_count;
    /*flush 不参与数据搬移。即使冲刷这一拍组合逻辑算出了旧数据的移动，
    时钟沿也会把 count 清零；旧数据没有有效位，所以不会再被 ID 看到。*/
    wire pop = q_valid[0] && id_allow_in;
    wire [1:0] pop_num = pop ? (id_take2 ? 2'd2 : 2'd1) : 2'd0;

    /*发新请求前要把两笔“可能进来的指令”算进去：
    一笔是上一拍 IROM 现在要返回的 push_num，另一笔是本拍准备发出的 fetch_num。
    count_next 算完 pop/push 后还剩多少，count_req 再加上新请求。
    只有总数不超过 FIFO 的 4 项容量，才允许继续发请求。这样不会覆盖旧指令。*/
    wire [3:0] count_next = {1'b0, q_count} - {2'b0, pop_num}
                                             + {2'b0, push_num};
    wire [3:0] count_req = count_next + {2'b0, fetch_num};
    /*IROM 只读，所以冲刷这一拍即使地址口还保持开启，也不会改坏存储器。
    下一拍 req_valid 会把这次旧响应丢掉。if2_allow_in 只看容量，不把 flush
    绕进取指数据路径。*/
    assign if2_allow_in = !reset && (count_req <= 4);

    /*四项 FIFO 只按队头顺序进出，保证 ID 永远先看到较老的指令。*/
    fetch_queue #(.WIDTH(QUEUE_W)) u_queue (
        .clk(clk),
        .reset(reset),
        .flush(if_flush),
        .push_num(push_num),
        .push0(push0),
        .push1(push1),
        .pop_num(pop_num),
        .out0(q_out0),
        .out1(q_out1),
        .valid(q_valid),
        .count(q_count)
    );

    //============================================================
    // IF2 to ID output
    //============================================================

    /*先把 FIFO 的内部记录拆开：原来的取指资料仍按原总线格式输出，
    预译码资料另走两条窄线，因此不会改动既有 PC/预测字段的位置。*/
    wire [FETCH_W-1:0] q_fetch0 = q_out0[FETCH_W-1:0];
    wire [FETCH_W-1:0] q_fetch1 = q_out1[FETCH_W-1:0];

    /*if_ras_ptr 是当前 RAS 指针。它像一个书签：分支预测错时，IF 可以回到
    这条指令取指当时的 RAS 位置。这个指针在指令真正从 FIFO 出队时才拼进去，
    不会因为 FIFO 里提前放了年轻指令而过期。*/
    assign if_to_id_bus = {q_fetch0, if_ras_ptr};
    /*valid 只表示 FIFO 当前有没有有效指令，不在组合逻辑里额外接 flush。
    分支纠错时，ID、ISSUE 和 FIFO 会在时钟沿一起清空；旧指令没有机会进入 EXE。*/
    assign if_to_id_valid = q_valid[0] && !reset;
    /*slot1 单独走自己的总线，避免把原来的 slot0 总线重新扩宽。
    是否真的消费 slot1，由 ID 的配对结果 id_take2 决定。*/
    assign if_to_id_bus1 = q_fetch1;
    assign if_to_id_dec0 = q_out0[QUEUE_W-1:FETCH_W];
    assign if_to_id_dec1 = q_out1[QUEUE_W-1:FETCH_W];
    assign if_to_id_valid1 = q_valid[1] && !reset;

    always @(posedge clk) begin
        if (reset) begin
            req_valid <= 1'b0;
            req_num <= 2'd0;
            req0 <= {META_W{1'b0}};
            req1 <= {META_W{1'b0}};
        end else begin
            /*请求的内容每拍都更新；req_valid 才是“这次内容能不能用”的开关。
            req_valid 为 0 时，req0/req1 里的旧内容不会被写进 FIFO。*/
            req_num <= fetch_num;
            req0 <= if_req0;
            req1 <= if_req1;

            if (if_flush) begin
                /*冲刷时丢掉冲刷前发出的旧请求，防止旧路径指令进入 FIFO。*/
                req_valid <= 1'b0;
            end else begin
                /*只有 if_to_if2_valid 和 if2_allow_in 同时为 1，才说明这次请求
                有 FIFO 空间可放。*/
                req_valid <= if_to_if2_valid && if2_allow_in;
            end
        end
    end
endmodule

//============================================================
// fetch-time pair predecoder
//============================================================

/*本文件私有辅助模块：decoder_one 在 IROM 数据返回时给一条指令贴分类标签。
标签只回答“会读写哪些寄存器、占用哪类执行资源、能不能参加配对”，不读取
寄存器值，也不生成真正送往 EXE 的完整控制包。标签随后和机器码一起进 FIFO。*/
module decoder_one (
    input  wire        valid,
    input  wire [31:0] inst,
    output wire [24:0] dec
);
    wire [6:0] opcode = inst[6:0];
    wire [4:0] rd     = inst[11:7];
    wire [2:0] funct3 = inst[14:12];
    wire [4:0] rs1    = inst[19:15];
    wire [4:0] rs2    = inst[24:20];
    wire [6:0] funct7 = inst[31:25];

    wire op_r      = (opcode == 7'b0110011);
    wire op_i      = (opcode == 7'b0010011);
    wire op_load   = (opcode == 7'b0000011);
    wire op_store  = (opcode == 7'b0100011);
    wire op_branch = (opcode == 7'b1100011);
    wire op_jal    = (opcode == 7'b1101111);
    wire op_jalr   = (opcode == 7'b1100111);
    wire op_lui    = (opcode == 7'b0110111);
    wire op_auipc  = (opcode == 7'b0010111);
    wire op_system = (opcode == 7'b1110011);
    wire op_fence  = (opcode == 7'b0001111);

    /*这里把 funct3/funct7 限制得很具体。
    非法编码会被标成 serial，不能因为 opcode 相同就误放进双发射。*/
    wire r_base = op_r && (
        (funct7 == 7'b0000000) ||
        ((funct7 == 7'b0100000) && ((funct3 == 3'b000) ||
                                    (funct3 == 3'b101)))
    );
    wire r_m = op_r && (funct7 == 7'b0000001);

    wire i_base = op_i && (
        (funct3 == 3'b000) || (funct3 == 3'b010) ||
        (funct3 == 3'b011) || (funct3 == 3'b100) ||
        (funct3 == 3'b110) || (funct3 == 3'b111) ||
        ((funct3 == 3'b001) && (funct7 == 7'b0000000)) ||
        ((funct3 == 3'b101) && ((funct7 == 7'b0000000) ||
                                (funct7 == 7'b0100000)))
    );
    wire i_orcb = ((inst & 32'hfff0_707f) == 32'h2870_5013);
    wire load_ok  = op_load && ((funct3 == 3'b000) ||
                                (funct3 == 3'b001) ||
                                (funct3 == 3'b010) ||
                                (funct3 == 3'b100) ||
                                (funct3 == 3'b101));
    wire store_ok = op_store && ((funct3 == 3'b000) ||
                                 (funct3 == 3'b001) ||
                                 (funct3 == 3'b010));
    wire branch_ok = op_branch && ((funct3 == 3'b000) ||
                                   (funct3 == 3'b001) ||
                                   (funct3 == 3'b100) ||
                                   (funct3 == 3'b101) ||
                                   (funct3 == 3'b110) ||
                                   (funct3 == 3'b111));
    wire jalr_ok = op_jalr && (funct3 == 3'b000);

    wire csr_reg = op_system && ((funct3 == 3'b001) ||
                                 (funct3 == 3'b010) ||
                                 (funct3 == 3'b011));
    wire csr_imm = op_system && ((funct3 == 3'b101) ||
                                 (funct3 == 3'b110) ||
                                 (funct3 == 3'b111));
    wire sys_ctl = (inst == 32'h0000_0073) || (inst == 32'h3020_0073);
    wire fence_ok = op_fence && ((funct3 == 3'b000) || (funct3 == 3'b001));

    wire legal = r_base || r_m || i_base || i_orcb || load_ok || store_ok ||
                 branch_ok || op_jal || jalr_ok || op_lui || op_auipc ||
                 csr_reg || csr_imm || sys_ctl || fence_ok;
    wire is_mul = r_m && !funct3[2];
    wire is_div = r_m &&  funct3[2];
    wire is_lsu = load_ok || store_ok;
    wire is_branch = branch_ok || op_jal || jalr_ok;
    wire is_serial = is_div || csr_reg || csr_imm || sys_ctl ||
                     fence_ok || !legal;
    wire is_alu = r_base || i_base || i_orcb || op_lui || op_auipc;

    wire rs1_en = r_base || r_m || i_base || i_orcb || load_ok || store_ok ||
                  branch_ok || jalr_ok || csr_reg;
    wire rs2_en = r_base || r_m || store_ok || branch_ok;
    wire rd_we  = r_base || r_m || i_base || i_orcb || load_ok || op_jal ||
                  jalr_ok || op_lui || op_auipc || csr_reg || csr_imm;

    /*25 位分类包的顺序固定为：合法、读写寄存器、资源类别、
    最后是 rs1、rs2、rd。这一格式与原 ID 预译码完全一致。*/
    assign dec = valid ? {legal, rs1_en, rs2_en, rd_we, is_alu, is_lsu,
                          is_mul, is_div, is_branch, is_serial,
                          rs1, rs2, rd} : 25'b0;
endmodule

/*本文件私有辅助模块：dual_decode 同时给两条 IROM 返回指令贴标签，
保证两个槽使用同一套规则。*/
module dual_decode (
    input  wire        valid0,
    input  wire [31:0] inst0,
    input  wire        valid1,
    input  wire [31:0] inst1,
    output wire [24:0] dec0,
    output wire [24:0] dec1
);
    decoder_one u_dec0 (.valid(valid0), .inst(inst0), .dec(dec0));
    decoder_one u_dec1 (.valid(valid1), .inst(inst1), .dec(dec1));
endmodule

//============================================================
// local fetch queue
//============================================================

/*本文件私有辅助模块：fetch_queue 是本文件内部的小 FIFO，只被 if2_stage 使用。
q0 放最老的指令，q1 放第二老的指令，q2/q3 依次更年轻。
所以 out0/out1 直接就是队头两项，不需要再做读指针和大选择器。*/
module fetch_queue #(
    parameter WIDTH = 103
)(
    input  wire             clk,
    input  wire             reset,
    input  wire             flush,
    input  wire [1:0]       push_num,
    input  wire [WIDTH-1:0] push0,
    input  wire [WIDTH-1:0] push1,
    input  wire [1:0]       pop_num,
    output wire [WIDTH-1:0] out0,
    output wire [WIDTH-1:0] out1,
    output wire [1:0]       valid,
    output reg  [2:0]       count
);
    /*q0 到 q3 按“从老到新”排列。*/
    reg [WIDTH-1:0] q0;
    reg [WIDTH-1:0] q1;
    reg [WIDTH-1:0] q2;
    reg [WIDTH-1:0] q3;

    /*n0~n3/ncount 是下一拍的候选值：先在组合逻辑里算好，时钟沿统一写入。*/
    reg [WIDTH-1:0] n0;
    reg [WIDTH-1:0] n1;
    reg [WIDTH-1:0] n2;
    reg [WIDTH-1:0] n3;
    reg [2:0] ncount;

    /*valid[0] 表示 q0 有效，valid[1] 表示 q1 也有效。*/
    assign out0 = q0;
    assign out1 = q1;
    assign valid[0] = (count != 3'd0);
    assign valid[1] = (count >= 3'd2);

    /*一拍可以先弹 0/1/2 条，再在队尾压入 0/1/2 条。
    上层已经保证容量合法；这里的任务只有一个：始终保持年龄顺序。*/
    always @(*) begin
        n0 = q0;
        n1 = q1;
        n2 = q2;
        n3 = q3;
        ncount = count;

        /*先弹出：年轻项目向队头移动，补上被拿走的老项目。*/
        case (pop_num)
            2'd1: begin
                n0 = q1;
                n1 = q2;
                n2 = q3;
                n3 = {WIDTH{1'b0}};
                ncount = count - 3'd1;
            end
            2'd2: begin
                n0 = q2;
                n1 = q3;
                n2 = {WIDTH{1'b0}};
                n3 = {WIDTH{1'b0}};
                ncount = count - 3'd2;
            end
            default: begin end
        endcase

        /*再压入：从新的队尾开始，先放 push0，再放 push1。*/
        case (ncount)
            3'd0: begin
                if (push_num >= 2'd1) n0 = push0;
                if (push_num == 2'd2) n1 = push1;
            end
            3'd1: begin
                if (push_num >= 2'd1) n1 = push0;
                if (push_num == 2'd2) n2 = push1;
            end
            3'd2: begin
                if (push_num >= 2'd1) n2 = push0;
                if (push_num == 2'd2) n3 = push1;
            end
            3'd3: begin
                if (push_num == 2'd1) n3 = push0;
            end
            default: begin end
        endcase
        ncount = ncount + {1'b0, push_num};
    end

    /*[阶段7A修改] reset仍初始化payload和有效状态；flush只把count清零。
    q0~q3中的旧数据在count=0后全部无效，不需要为了“看起来干净”而清零。
    这样EXE分支flush不再驱动4*WIDTH位payload寄存器的D/CE，只扇出到
    3位count，切断Table3报告中的高扇出EXE->FIFO数据关键路径。*/
    always @(posedge clk) begin
        if (reset) begin
            q0 <= {WIDTH{1'b0}};
            q1 <= {WIDTH{1'b0}};
            q2 <= {WIDTH{1'b0}};
            q3 <= {WIDTH{1'b0}};
            count <= 3'd0;
        end else begin
            q0 <= n0;
            q1 <= n1;
            q2 <= n2;
            q3 <= n3;
            /*flush 时 payload 可能还会跟着组合逻辑移动，但 count 会清零。
            清零后 FIFO 逻辑上就是空的，下一次 push 从 q0 开始。*/
            count <= flush ? 3'd0 : ncount;
        end
    end
endmodule
