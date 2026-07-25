`timescale 1ns / 1ps

module miniRV_SoC (
    input  wire        fpga_clk,           // C++ 层传进来的驱动时钟
    input  wire        fpga_rst,           // C++ 层传进来的复位信号 (通常测试框架是高有效)

    /* [新增] sim_error：difftest出错时由C++置1
     *        这根线不连接到CPU内部逻辑，仅作为VCD波形里的标记信号。
     *        在GTKWave里找到sim_error变高的那个时刻，即为difftest出错时刻，
     *        向前看几个周期即可定位bug。 */
    input  wire        sim_error,          // C++ 探针：difftest出错信号（仅波形可见）

    output wire        debug_wb_have_inst, // C++ 探针：当前周期是否有指令写回
    output wire [31:0] debug_wb_pc,        // C++ 探针：写回指令的PC
    output wire        debug_wb_ena,       // C++ 探针：是否写寄存器(RegWrite)
    output wire [4:0]  debug_wb_reg,       // C++ 探针：写入目的寄存器编号
    output wire [31:0] debug_wb_value,     // C++ 探针：写入的数据
    output wire        debug_v1_store_fire,
    output wire [31:0] debug_v1_store_addr,
    output wire [31:0] debug_v1_store_data
);

    // 1. 对于你原工程的 myCPU，一般是低电平复位，做一次反相
    wire resetn = ~fpga_rst;

    // 2. 指令 ROM 交互总线
    wire        inst_en;
    wire [3:0]  inst_we;      // 虽然指令ROM不写，但为了端口对齐接着它
    wire [31:0] inst_addr;
    wire [31:0] inst_wdata;
    wire [31:0] inst_rdata;

    // 3. 数据 RAM/总线交互
    wire        perip_en;
    wire [3:0]  perip_we;
    wire [31:0] perip_addr;
    wire [31:0] perip_wdata;
    wire [31:0] perip_rdata;

    // 4. 接出你原本 mycpu_top 的多位宽 debug 信号
    wire [3:0]  debug_wb_rf_we;
    wire        debug_wb_valid;

    // ====================================================================
    // 例化 CPU：直接剥离所有的纯外设和IP，这叫 "裸测"
    // ====================================================================
    // 仿真目录里的既有 bin/*.bin 链接在地址 0；上板 student_top 未覆盖
    // RESET_PC，仍使用 mycpu_top 的官方 IROM 基址 0x8000_0000。
    mycpu_top #(
        .RESET_PC(32'h0000_0000)
    ) u_cpu (
        .clk                (fpga_clk),
        .resetn             (resetn),

        // 指令存取口
        .inst_sram_en       (inst_en),
        .inst_sram_we       (inst_we),
        .inst_sram_addr     (inst_addr),
        .inst_sram_wdata    (inst_wdata),
        .inst_sram_rdata    (inst_rdata),

        // 数据/外设存取口
        .data_sram_en       (perip_en),     
        .data_sram_we       (perip_we),     
        .data_sram_addr     (perip_addr),     
        .data_sram_wdata    (perip_wdata),    
        .data_sram_rdata    (perip_rdata),    
        
        // 你的 mycpu_top 对外暴露的调试信号
        .debug_wb_pc        (debug_wb_pc),
        .debug_wb_rf_we     (debug_wb_rf_we), 
        .debug_wb_rf_wnum   (debug_wb_reg),
        .debug_wb_rf_wdata  (debug_wb_value),
        .debug_wb_valid     (debug_wb_valid)
    );

    // ====================================================================
    // 对齐探针要求：由于 C++ 只认单 bit 的使能，只要 4 根线不是全 0 代表就是要写
    // ====================================================================
    assign debug_wb_ena = (debug_wb_rf_we != 4'b0);

    // ====================================================================
    // C++ Csrc 测试框架要求一个 "当前是否有真指令提交" 的心跳线
    // 通常如果 CPU 在写寄存器，必然是有了指令；如果是 store/branch 指令它不写寄存器，
    // 最简单保险的方式是：只要 PC 不是 0 就是由于有效代码执行所致。
    // ====================================================================
    assign debug_wb_have_inst = (debug_wb_valid); // 亦可与其他有效信号做逻辑或是


    // ====================================================================
    // 实例化软件 RAM 模型 (使用 ram2.v 中的特性)
    // ====================================================================

    // 指令 BRAM：32 x 4096 (深度 4096 刚好对应 Word 地址 12 位)
    IROM #(
        .ADDR_BITS (12)
    ) mock_iram (
        .clka  (fpga_clk),
        .ena   (inst_en), 
        // RISC-V和龙芯都是按字节编址。32位字(Word)内部占4个字节。
        // 将外面的 32位字节地址 砍掉最末2位，传进 RAM 变作“第 N 个字”来取！
        .addra (inst_addr[13:2]),  // 截取 [13:2] 共 12 bit
        .douta (inst_rdata)
    );

`ifdef ENABLE_COPROC
    // ====================================================================
    // [新增] 协处理器(脉动阵列加速器) MMIO 接入
    // ----------------------------------------------------------------------
    // 复用 perip_bridge.sv 里那套"CPU 32位 ⇄ 加速器 128位"的 gather 逻辑，但去掉
    // 上板专用外设(seg/led/counter/BRAM-IP)，只留加速器，挂在数据地址空间：
    //   0x8020_0060  写：压入一个32位数据(连写4次=128位自动入shared_buffer, 地址+1)
    //   0x8020_0064  写：load_start(开始装填，复位gather地址/计数)
    //   0x8020_0068  写：load_done (装填完毕、启动计算)
    //   0x8020_006C  读：状态 {STATE[6:1], calc_done[0]}
    //   0x8020_0070  写：设置结果读出行号 result_raddr
    //   0x8020_0080+4j 读：C[row][j]，16bit 符号扩展为32位 (j=0..15)
    // 注意：权重需按逆序装入(先压 W[15] 那一行)，见独立仿真验证结论。
    // ====================================================================
    wire        accel_sel = perip_en && (perip_addr[31:12] == 20'h80200);
    wire        accel_v0_sel = accel_sel && (perip_addr[11:8] == 4'h0);
    wire        accel_v1_sel = accel_sel && (perip_addr[11:8] == 4'h1);

    // gather: 把 CPU 连写的4个32位拼成128位 (取自 perip_bridge)
    reg [127:0] cop_load_buf;
    reg [1:0]   cop_word_cnt;
    reg [12:0]  cop_load_addr;
    reg         cop_load_we;
    reg [127:0] cop_load_wdata;
    reg         cop_load_start;
    reg         cop_load_done;
    reg [3:0]   cop_result_row;

    always @(posedge fpga_clk) begin
        if (fpga_rst) begin
            cop_load_buf <= 0; cop_word_cnt <= 0; cop_load_addr <= 0;
            cop_load_we <= 0; cop_load_wdata <= 0; cop_load_start <= 0;
            cop_load_done <= 0; cop_result_row <= 0;
        end else begin
            cop_load_start <= 1'b0;   // 脉冲信号默认清零
            cop_load_done  <= 1'b0;
            // [修复] 地址自增推迟到"上一拍发生了写"之后：保证写 shared_buffer 用的是
            //   自增前的地址(与 cop_load_we 同拍呈现给加速器)，否则每行会错位写到下一地址。
            if (cop_load_we) cop_load_addr <= cop_load_addr + 1'b1;
            cop_load_we    <= 1'b0;
            if (accel_v0_sel && (perip_we != 4'b0)) begin
                case (perip_addr[7:0])
                    8'h60: begin   // PUSH
                        case (cop_word_cnt)
                            2'd0: cop_load_buf[31:0]   <= perip_wdata;
                            2'd1: cop_load_buf[63:32]  <= perip_wdata;
                            2'd2: cop_load_buf[95:64]  <= perip_wdata;
                            2'd3: cop_load_buf[127:96] <= perip_wdata;
                        endcase
                        if (cop_word_cnt == 2'd3) begin
                            cop_load_wdata <= {perip_wdata, cop_load_buf[95:0]};
                            cop_load_we    <= 1'b1;   // 下一拍写 shared[cop_load_addr]
                            cop_word_cnt   <= 2'd0;
                        end else
                            cop_word_cnt <= cop_word_cnt + 1'b1;
                    end
                    8'h64: begin cop_load_start <= 1'b1; cop_load_addr <= 0; cop_word_cnt <= 0; end
                    8'h68: cop_load_done <= 1'b1;
                    8'h70: cop_result_row <= perip_wdata[3:0];
                endcase
            end
        end
    end

    wire [5:0]   accel_state;
    wire         accel_calc_done;
    wire [255:0] accel_result_row;

    Accelerator u_accel (
        .CLK(fpga_clk), .RESET(~fpga_rst), .EN(1'b1),
        .IADDR(13'd16), .WADDR(13'd0), .OADDR(13'd0),
        .STATE(accel_state),
        .load_start(v1_busy ? v1_load_start : cop_load_start),
        .load_we(v1_busy ? v1_load_we : cop_load_we),
        .load_addr(v1_busy ? v1_load_addr : cop_load_addr),
        .load_wdata(v1_busy ? v1_load_wdata : cop_load_wdata),
        .load_done(v1_busy ? v1_load_done : cop_load_done),
        .calc_done(accel_calc_done),
        .result_data(),                       // 旧128位口不用
        .result_raddr(v1_busy ? v1_result_raddr : cop_result_row),
        .result_row(accel_result_row),
        // 调试口(独立仿真用)，集成时不接
        .dbg_out_sum(), .dbg_input_out(), .dbg_weight_out(),
        .dbg_share_out(), .dbg_w_en(), .dbg_selector()
    );

    wire         v1_mem_en;
    wire [3:0]   v1_mem_we;
    wire [31:0]  v1_mem_addr;
    wire [31:0]  v1_mem_wdata;
    wire [31:0]  v1_reg_rdata;
    wire         v1_load_start;
    wire         v1_load_we;
    wire [12:0]  v1_load_addr;
    wire [127:0] v1_load_wdata;
    wire         v1_load_done;
    wire         v1_busy;
    wire         v1_done;
    wire [3:0]   v1_result_raddr;
    wire         v1_store_fire;
    wire [31:0]  v1_store_addr;
    wire [31:0]  v1_store_data;

    coproc_v1_engine u_coproc_v1 (
        .clk        (fpga_clk),
        .rst        (fpga_rst),
        .reg_sel    (accel_v1_sel),
        .reg_addr   (perip_addr[11:0]),
        .rd_addr    (perip_addr_q[11:0]),
        .reg_we     (perip_we),
        .reg_wdata  (perip_wdata),
        .reg_rdata  (v1_reg_rdata),
        .mem_en     (v1_mem_en),
        .mem_we     (v1_mem_we),
        .mem_addr   (v1_mem_addr),
        .mem_wdata  (v1_mem_wdata),
        .mem_rdata  (dram_rdata),
        .dbg_store_fire(v1_store_fire),
        .dbg_store_addr(v1_store_addr),
        .dbg_store_data(v1_store_data),
        .load_start (v1_load_start),
        .load_we    (v1_load_we),
        .load_addr  (v1_load_addr),
        .load_wdata (v1_load_wdata),
        .load_done  (v1_load_done),
        .busy       (v1_busy),
        .done       (v1_done),
        .calc_done  (accel_calc_done),
        .result_raddr(v1_result_raddr),
        .result_row (accel_result_row)
    );

    // 读出：地址打一拍，与 DRAM 同步读(1拍延迟)对齐
    reg [31:0] perip_addr_q;
    reg        accel_sel_q;
    always @(posedge fpga_clk) begin
        perip_addr_q <= perip_addr;
        accel_sel_q  <= accel_sel && (perip_we == 4'b0);
    end
    wire [15:0] res_col = accel_result_row[16*perip_addr_q[5:2] +: 16];
    wire [31:0] accel_v0_rdata =
        (perip_addr_q[7:0] == 8'h6C) ? {25'b0, accel_state, accel_calc_done} :
        (perip_addr_q[7:6] == 2'b10) ? {{16{res_col[15]}}, res_col} :
        32'h0;

    wire [31:0] accel_rdata = (perip_addr_q[11:8] == 4'h1) ? v1_reg_rdata : accel_v0_rdata;

    // 数据 BRAM：accel 区不写 DRAM
    wire        dram_port_en = v1_mem_en || perip_en;
    wire [31:0] dram_addr_mux = v1_mem_en ? v1_mem_addr : perip_addr;
    wire [31:0] dram_wdata_mux = v1_mem_en ? v1_mem_wdata : perip_wdata;
    wire [3:0]  dram_we = v1_mem_en ? v1_mem_we : (accel_sel ? 4'b0 : perip_we);
    wire [31:0] dram_rdata;
    DRAM #(
        .ADDR_BITS (22)
    ) mock_dram (
        .clka  (fpga_clk),
        .addra (dram_addr_mux[23:2]),
        .wea   (dram_we),
        .dina  (dram_wdata_mux),
        .douta (dram_rdata)
    );

    assign perip_rdata = accel_sel_q ? accel_rdata : dram_rdata;
    assign debug_v1_store_fire = v1_store_fire;
    assign debug_v1_store_addr = v1_store_addr;
    assign debug_v1_store_data = v1_store_data;
`else
    // 基础 CPU 指令回归不依赖协处理器。协处理器 RTL 不在当前 miniRV
    // 工作区时，保留普通 DRAM 数据通路，并将专用调试探针安全置零。
    wire [31:0] dram_rdata;
    DRAM #(
        .ADDR_BITS (22)
    ) mock_dram (
        .clka  (fpga_clk),
        .addra (perip_addr[23:2]),
        .wea   (perip_we),
        .dina  (perip_wdata),
        .douta (dram_rdata)
    );

    assign perip_rdata          = dram_rdata;
    assign debug_v1_store_fire  = 1'b0;
    assign debug_v1_store_addr  = 32'b0;
    assign debug_v1_store_data  = 32'b0;
`endif

endmodule

// `timescale 1ns / 1ps

// module miniRV_SoC (
//     input  wire        fpga_clk,
//     input  wire        fpga_rst,

//     output wire        debug_wb_have_inst,
//     output wire [31:0] debug_wb_pc,
//     output wire        debug_wb_ena,
//     output wire [4:0]  debug_wb_reg,
//     output wire [31:0] debug_wb_value
// );

//     wire resetn = ~fpga_rst;

//     wire        inst_en;
//     wire [3:0]  inst_we;
//     wire [31:0] inst_addr;
//     wire [31:0] inst_wdata;
//     wire [31:0] inst_rdata;

//     wire        perip_en;
//     wire [3:0]  perip_we;
//     wire [31:0]  perip_addr;
//     wire [31:0]  perip_wdata;
//     wire [31:0]  perip_rdata;

//     wire [3:0]  debug_wb_rf_we;
//     wire        debug_wb_valid;

//     mycpu_top u_cpu (
//         .clk                (fpga_clk),
//         .resetn             (resetn),

//         .inst_sram_en       (inst_en),
//         .inst_sram_we       (inst_we),
//         .inst_sram_addr     (inst_addr),
//         .inst_sram_wdata    (inst_wdata),
//         .inst_sram_rdata    (inst_rdata),

//         .data_sram_en       (perip_en),
//         .data_sram_we       (perip_we),
//         .data_sram_addr     (perip_addr),
//         .data_sram_wdata    (perip_wdata),
//         .data_sram_rdata    (perip_rdata),

//         .debug_wb_pc        (debug_wb_pc),
//         .debug_wb_rf_we     (debug_wb_rf_we),
//         .debug_wb_rf_wnum   (debug_wb_reg),
//         .debug_wb_rf_wdata  (debug_wb_value),
//         .debug_wb_valid     (debug_wb_valid)
//     );

//     assign debug_wb_ena = (debug_wb_rf_we != 4'b0);
//     assign debug_wb_have_inst = debug_wb_valid;

//     IRAM #(
//         .ADDR_BITS (12)
//     ) mock_iram (
//         .clka  (fpga_clk),
//         .ena   (inst_en),
//         .addra (inst_addr[13:2]),
//         .douta (inst_rdata)
//     );

//     wire monitor_sel = perip_en && (
//         perip_addr == 32'h80000000 ||
//         perip_addr == 32'h80000004
//     );
//     wire digit_sel = perip_en && (perip_addr == 32'hFFFFF000);
//     wire dram_sel = perip_en && !monitor_sel && !digit_sel;

//     wire [31:0] dram_addr = perip_addr[31] ? (perip_addr - 32'h80000000) : perip_addr;
//     wire [19:0] dram_word_addr = dram_addr[21:2];
//     wire [3:0]  dram_wea = dram_sel ? perip_we : 4'b0;
//     wire [31:0] dram_rdata;

//     reg [31:0] monitor_0;
//     reg [31:0] monitor_1;
//     reg [31:0] digit_data;

//     wire [31:0] monitor_rdata = perip_addr[2] ? monitor_1 : monitor_0;
//     wire [31:0] digit_rdata   = digit_data;

//     assign perip_rdata = monitor_sel ? monitor_rdata :
//                          digit_sel   ? digit_rdata :
//                                        dram_rdata;

//     always @(posedge fpga_clk) begin
//         if (monitor_sel && (|perip_we)) begin
//             if (perip_addr[2] == 1'b0) begin
//                 if (perip_we[0]) monitor_0[ 7: 0] <= perip_wdata[ 7: 0];
//                 if (perip_we[1]) monitor_0[15: 8] <= perip_wdata[15: 8];
//                 if (perip_we[2]) monitor_0[23:16] <= perip_wdata[23:16];
//                 if (perip_we[3]) monitor_0[31:24] <= perip_wdata[31:24];
//             end else begin
//                 if (perip_we[0]) monitor_1[ 7: 0] <= perip_wdata[ 7: 0];
//                 if (perip_we[1]) monitor_1[15: 8] <= perip_wdata[15: 8];
//                 if (perip_we[2]) monitor_1[23:16] <= perip_wdata[23:16];
//                 if (perip_we[3]) monitor_1[31:24] <= perip_wdata[31:24];
//             end
//         end

//         if (digit_sel && (|perip_we)) begin
//             if (perip_we[0]) digit_data[ 7: 0] <= perip_wdata[ 7: 0];
//             if (perip_we[1]) digit_data[15: 8] <= perip_wdata[15: 8];
//             if (perip_we[2]) digit_data[23:16] <= perip_wdata[23:16];
//             if (perip_we[3]) digit_data[31:24] <= perip_wdata[31:24];
//         end
//     end

//     DRAM #(
//         .ADDR_BITS (20)
//     ) mock_dram (
//         .clka  (fpga_clk),
//         .addra (dram_word_addr),
//         .wea   (dram_wea),
//         .dina  (perip_wdata),
//         .douta (dram_rdata)
//     );

// endmodule
