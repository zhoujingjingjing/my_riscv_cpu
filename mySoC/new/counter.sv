`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/22/2025 03:04:25 PM
// Design Name: 
// Module Name: counter
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


module counter #(
    // [决赛计时修复] 计时基准来自平台固定的50MHz时钟，不再依赖CPU频率。
    // 参数仅描述固定参考时钟；以后只修改PLL的CPU输出时无需修改这里。
    parameter integer REF_CLK_FREQ_HZ = 50_000_000
)(
    input  logic         cpu_clk,
    input  logic         ref_clk,
    input  logic         rst,

    input  logic [31:0]  perip_wdata,
    input  logic         cnt_wen,
    output logic [31:0]  perip_rdata
);

    localparam integer REF_CYCLES_PER_MS = REF_CLK_FREQ_HZ / 1000;
    localparam integer REF_DIVIDER_WIDTH =
        (REF_CYCLES_PER_MS <= 1) ? 1 : $clog2(REF_CYCLES_PER_MS);

    // CPU域只保存“要求计时/要求停止”这个单比特状态。写数据本身不跨域。
    logic run_request_cpu;

    always_ff @(posedge cpu_clk) begin
        if (rst) begin
            run_request_cpu <= 1'b0;
        end else if (cnt_wen && perip_wdata == 32'h8000_0000) begin
            run_request_cpu <= 1'b1;
        end else if (cnt_wen && perip_wdata == 32'hFFFF_FFFF) begin
            run_request_cpu <= 1'b0;
        end
    end

    // 50MHz域用两级同步器接收运行状态。开始/停止写之间相隔远大于两个
    // 50MHz周期，因此状态不会丢失；同步延迟在开始和停止两端基本抵消。
    (* ASYNC_REG = "TRUE" *) logic run_sync1_ref;
    (* ASYNC_REG = "TRUE" *) logic run_sync2_ref;
    logic [REF_DIVIDER_WIDTH-1:0] ref_cycle_count;
    logic                         running_ref;
    logic [31:0]                  cnt_ms_ref;

    always_ff @(posedge ref_clk) begin
        if (rst) begin
            run_sync1_ref <= 1'b0;
            run_sync2_ref <= 1'b0;
        end else begin
            run_sync1_ref <= run_request_cpu;
            run_sync2_ref <= run_sync1_ref;
        end
    end

    // 真正的分频和毫秒累计都在固定50MHz域。每次重新开始时清零不足1ms的
    // 余数，保持原计时器“从开始命令重新累计完整毫秒”的语义。
    always_ff @(posedge ref_clk) begin
        if (rst) begin
            ref_cycle_count <= '0;
            cnt_ms_ref      <= 32'b0;
            running_ref     <= 1'b0;
        end else begin
            if (!running_ref && run_sync2_ref) begin
                running_ref     <= 1'b1;
                ref_cycle_count <= '0;
            end else if (running_ref && !run_sync2_ref) begin
                running_ref     <= 1'b0;
                ref_cycle_count <= '0;
            end else if (running_ref) begin
                if (ref_cycle_count == REF_CYCLES_PER_MS - 1) begin
                    ref_cycle_count <= '0;
                    cnt_ms_ref      <= cnt_ms_ref + 1'b1;
                end else begin
                    ref_cycle_count <= ref_cycle_count + 1'b1;
                end
            end else begin
                ref_cycle_count <= '0;
            end
        end
    end

    // 32位计时结果先转Gray码再跨回CPU域。Gray码相邻值只变化一位，
    // 两级同步后不会得到由多个不同时刻的二进制位拼成的错误数值。
    wire [31:0] cnt_ms_gray_ref = cnt_ms_ref ^ (cnt_ms_ref >> 1);
    (* ASYNC_REG = "TRUE" *) logic [31:0] cnt_ms_gray_sync1_cpu;
    (* ASYNC_REG = "TRUE" *) logic [31:0] cnt_ms_gray_sync2_cpu;

    always_ff @(posedge cpu_clk) begin
        if (rst) begin
            cnt_ms_gray_sync1_cpu <= 32'b0;
            cnt_ms_gray_sync2_cpu <= 32'b0;
        end else begin
            cnt_ms_gray_sync1_cpu <= cnt_ms_gray_ref;
            cnt_ms_gray_sync2_cpu <= cnt_ms_gray_sync1_cpu;
        end
    end

    function automatic [31:0] gray_to_binary(input [31:0] gray);
        integer bit_index;
        begin
            gray_to_binary[31] = gray[31];
            for (bit_index = 30; bit_index >= 0; bit_index = bit_index - 1)
                gray_to_binary[bit_index] =
                    gray_to_binary[bit_index + 1] ^ gray[bit_index];
        end
    endfunction

    assign perip_rdata = gray_to_binary(cnt_ms_gray_sync2_cpu);

endmodule
